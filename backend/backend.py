import os
from dotenv import load_dotenv
from dotenv import dotenv_values
import logging
from typing import Dict, List, Optional

# Load environment variables
config = dotenv_values(".env")
if not config:
    config = os.environ

# Set up logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Import after loading env vars to avoid circular import issues
from fastapi import FastAPI, HTTPException, Depends
from pydantic import BaseModel
from langchain_community.embeddings import HuggingFaceEmbeddings
from langchain_community.vectorstores import Qdrant
from langchain_community.llms import HuggingFaceHub
from langchain.chains import ConversationalRetrievalChain
import torch
from qdrant_client import QdrantClient
from qdrant_client.http import models
import uvicorn

from fastapi.middleware.cors import CORSMiddleware

# Initialize FastAPI app
app = FastAPI(
    title="Physical AI & Humanoid Robotics RAG Chatbot API",
    description="API for querying Physical AI & Humanoid Robotics textbook content using Retrieval Augmented Generation",
    version="1.0.0"
)

app.add_middleware(
    CORSMiddleware,
    allow_origins=["http://localhost:3000", "http://localhost:3000/physical-ai-book/"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Configuration constants
COLLECTION_NAME = config['COLLECTION_NAME']
QDRANT_URL = config['QDRANT_URL']
QDRANT_API_KEY = config['QDRANT_API_KEY']
HF_API_TOKEN = config['HF_API_TOKEN']
EMBEDDING_MODEL_NAME = config['EMBEDDING_MODEL_NAME']
GENERATION_MODEL_NAME = config['GENERATION_MODEL_NAME']

# Global variables for clients
qdrant_client: Optional[QdrantClient] = None
qdrant_vector_store: Optional[Qdrant] = None
llm = None
qa_chain = None


class QueryRequest(BaseModel):
    """Request model for query endpoint"""
    question: str
    top_k: int = 3


class QueryResponse(BaseModel):
    """Response model for query endpoint"""
    answer: str
    sources: List[Dict[str, str]]
    question: str


@app.on_event("startup")
def startup_event():
    """Initialize clients when the application starts"""
    global qdrant_client, qdrant_vector_store, llm, qa_chain

    logger.info("Initializing HuggingFace embeddings...")
    embeddings = HuggingFaceEmbeddings(
        model_name=config["EMBEDDING_MODEL_NAME"],
        huggingfacehub_api_token=config["HF_API_TOKEN"]
    )

    logger.info("Initializing Qdrant client for cloud...")
    qdrant_client = QdrantClient(
        url=config["QDRANT_URL"],
        api_key=config["QDRANT_API_KEY"]
    )

    logger.info("Initializing Qdrant vector store...")
    qdrant_vector_store = Qdrant(
        client=qdrant_client,
        embeddings=embeddings,
        collection_name=config["COLLECTION_NAME"],
    )

    logger.info(f"Initializing HuggingFaceHub LLM: {config['GENERATION_MODEL_NAME']}")
    llm = HuggingFaceHub(
        repo_id=config["GENERATION_MODEL_NAME"],
        huggingfacehub_api_token=config["HF_API_TOKEN"],
        model_kwargs={"temperature": 0.1, "max_length": 512}
    )

    logger.info("Initializing ConversationalRetrievalChain...")
    qa_chain = ConversationalRetrievalChain.from_llm(
        llm=llm,
        retriever=qdrant_vector_store.as_retriever(),
        return_source_documents=True
    )

    # Check if collection exists
    try:
        collections = qdrant_client.get_collections()
        collection_names = [col.name for col in collections.collections]

        if config["COLLECTION_NAME"] not in collection_names:
            logger.warning(f"Collection '{config['COLLECTION_NAME']}' does not exist. Please make sure to run the ingestion script first.")
        else:
            logger.info(f"Connected to collection '{config['COLLECTION_NAME']}' successfully.")
    except Exception as e:
        logger.error(f"Error connecting to Qdrant: {str(e)}")
        raise


def generate_answer_with_context(question: str, chat_history: List = []) -> Dict:
    """
    Generate an answer using the ConversationalRetrievalChain
    """
    if qa_chain is None:
        raise HTTPException(status_code=500, detail="QA chain not initialized")

    try:
        result = qa_chain({
            "question": question,
            "chat_history": chat_history
        })

        return result
    except Exception as e:
        logger.error(f"Error generating answer: {str(e)}")
        raise HTTPException(status_code=500, detail=f"Error generating answer: {str(e)}")


@app.get("/")
def read_root():
    """Root endpoint for health check"""
    return {"message": "Physical AI & Humanoid Robotics RAG Chatbot API is running", "status": "ok"}


@app.post("/api/query", response_model=QueryResponse)
def query_endpoint(request: QueryRequest):
    """
    Query endpoint that takes a question and returns an answer based only on the book content
    """
    try:
        # Validate inputs
        if not request.question.strip():
            raise HTTPException(status_code=400, detail="Question cannot be empty")

        if request.top_k <= 0 or request.top_k > 10:
            raise HTTPException(status_code=400, detail="top_k must be between 1 and 10")

        logger.info(f"Processing query: '{request.question[:50]}...' with top_k={request.top_k}")

        # Generate answer using the LLM with retrieved context
        result = generate_answer_with_context(request.question)

        # Extract answer and source documents
        answer = result.get("answer", "I couldn't find a specific answer to your question based on the textbook content.")
        source_docs = result.get("source_documents", [])

        # Prepare sources from source documents
        sources = []
        for doc in source_docs:
            sources.append({
                "text": doc.page_content[:200] + "..." if len(doc.page_content) > 200 else doc.page_content,
                "source": doc.metadata.get("source", "Unknown"),
                "relevance_score": doc.metadata.get("relevance_score", "N/A")
            })

        response = QueryResponse(
            answer=answer,
            sources=sources,
            question=request.question
        )

        logger.info(f"Query processed successfully. Found {len(source_docs)} source documents.")
        return response

    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Unexpected error processing query: {str(e)}")
        raise HTTPException(status_code=500, detail=f"Internal server error: {str(e)}")


@app.get("/api/health")
def health_check():
    """Health check endpoint"""
    return {"status": "healthy", "service": "RAG Chatbot API"}


if __name__ == "__main__":
    uvicorn.run(app, host="0.0.0.0", port=8000)