import os
from dotenv import load_dotenv
import logging
from typing import Dict, List, Optional

# Load environment variables
load_dotenv()

# Set up logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Import after loading env vars to avoid circular import issues
from fastapi import FastAPI, HTTPException, Depends
from pydantic import BaseModel
from sentence_transformers import SentenceTransformer
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
QDRANT_HOST = os.getenv("QDRANT_HOST", "localhost")
QDRANT_PORT = int(os.getenv("QDRANT_PORT", 6333))
COLLECTION_NAME = os.getenv("QDRANT_COLLECTION", "humanoid_robotics")
EMBEDDING_MODEL_NAME = os.getenv("EMBEDDING_MODEL", "all-MiniLM-L6-v2")  # Using Sentence Transformers model

# Global variables for clients
qdrant_client: Optional[QdrantClient] = None
embedder: Optional[SentenceTransformer] = None


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
    global qdrant_client, embedder

    logger.info("Initializing Qdrant client...")
    qdrant_client = QdrantClient(host=QDRANT_HOST, port=QDRANT_PORT)

    logger.info(f"Initializing sentence transformer model: {EMBEDDING_MODEL_NAME}")
    embedder = SentenceTransformer(EMBEDDING_MODEL_NAME)

    # Check if collection exists
    try:
        collections = qdrant_client.get_collections()
        collection_names = [col.name for col in collections.collections]

        if COLLECTION_NAME not in collection_names:
            logger.warning(f"Collection '{COLLECTION_NAME}' does not exist. Please make sure to run the ingestion script first.")
        else:
            logger.info(f"Connected to collection '{COLLECTION_NAME}' successfully.")
    except Exception as e:
        logger.error(f"Error connecting to Qdrant: {str(e)}")
        raise


def retrieve_relevant_chunks(query_vector: List[float], top_k: int = 3) -> List[Dict]:
    """
    Retrieve relevant chunks from Qdrant based on query vector
    """
    if qdrant_client is None:
        raise HTTPException(status_code=500, detail="Qdrant client not initialized")

    try:
        search_results = qdrant_client.query_points(
            collection_name=COLLECTION_NAME,
            query=query_vector,
            limit=top_k,
            with_payload=True
        ).points

        relevant_chunks = []
        for result in search_results:
            chunk_data = {
                "text": result.payload.get("text", ""),
                "source": result.payload.get("source", ""),
                "score": result.score
            }
            relevant_chunks.append(chunk_data)

        return relevant_chunks
    except Exception as e:
        logger.error(f"Error retrieving chunks from Qdrant: {str(e)}")
        raise HTTPException(status_code=500, detail=f"Error retrieving chunks: {str(e)}")


def generate_answer(context: str, question: str) -> str:
    """
    Generate an answer based on context and question.
    Uses a lightweight approach to synthesize information from the context.
    """
    # In a production environment, you would integrate with a powerful LLM like:
    # 1. OpenAI GPT API
    # 2. Anthropic Claude API
    # 3. Hugging Face transformers with a pre-trained model
    # 4. Local LLM with transformers
    #
    # For this implementation, we'll create a synthesized response from the context
    # that attempts to address the question directly.

    # Clean up the context
    clean_context = context.replace('\n', ' ').strip()

    # Extract sentences that seem most relevant to the question
    sentences = clean_context.split('.')
    question_lower = question.lower()

    # Find sentences that contain keywords from the question
    relevant_sentences = []
    question_words = [word for word in question_lower.split() if len(word) > 3]  # Only longer words

    for sentence in sentences:
        sentence_lower = sentence.lower()
        # Count how many question words appear in this sentence
        matches = sum(1 for word in question_words if word in sentence_lower)
        if matches > 0:
            relevant_sentences.append((sentence.strip(), matches))

    # Sort by relevance (number of matches)
    relevant_sentences.sort(key=lambda x: x[1], reverse=True)

    # Take top sentences that contribute to answering the question
    top_sentences = [sent[0] for sent in relevant_sentences[:3]]  # Top 3 most relevant

    # If no specific matches, take the first few sentences as a fallback
    if not top_sentences:
        top_sentences = [sent.strip() for sent in sentences[:3] if sent.strip()]

    # Join the selected sentences
    synthesized_content = '. '.join(top_sentences).strip()

    if not synthesized_content:
        return f"Based on the textbook content, I could not find specific information to answer: '{question}'. Please refer to the textbook for more details."

    # Formulate the response
    answer = f"Based on the Physical AI & Humanoid Robotics textbook:\n\n{synthesized_content}.\n\nThis information addresses your question: '{question}'."
    return answer


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

        # Generate embedding for the query using Sentence Transformers
        if embedder is None:
            raise HTTPException(status_code=500, detail="Embedding model not initialized")

        query_embedding = embedder.encode([request.question])[0].tolist()

        # Retrieve relevant chunks from Qdrant
        relevant_chunks = retrieve_relevant_chunks(query_embedding, request.top_k)

        if not relevant_chunks:
            # If no chunks found, return a non-RAG response
            answer = "I am an AI assistant designed to answer questions based on the Physical AI & Humanoid Robotics textbook. I couldn't find specific information to answer your question."
            sources = []

            response = QueryResponse(
                answer=answer,
                sources=sources,
                question=request.question
            )

            logger.info("No relevant content found in the textbook.")
            return response

        # Define relevance threshold
        RELEVANCE_THRESHOLD = 0.7

        # Filter chunks based on relevance threshold
        filtered_chunks = [chunk for chunk in relevant_chunks if chunk["score"] > RELEVANCE_THRESHOLD]

        # If no chunks pass the threshold, return a non-RAG answer
        if not filtered_chunks:
            answer = "I am an AI assistant designed to answer questions based on the Physical AI & Humanoid Robotics textbook. I couldn't find specific information to answer your question."
            sources = []

            response = QueryResponse(
                answer=answer,
                sources=sources,
                question=request.question
            )

            logger.info(f"No chunks passed the relevance threshold of {RELEVANCE_THRESHOLD}.")
            return response

        # Use filtered chunks for context
        context_parts = [chunk["text"] for chunk in filtered_chunks]
        context = "\n\n".join(context_parts)

        # Generate answer based on context and question
        answer = generate_answer(context, request.question)

        # Prepare response
        sources = [{"text": chunk["text"][:200] + "..." if len(chunk["text"]) > 200 else chunk["text"],
                   "source": chunk["source"],
                   "relevance_score": f"{chunk['score']:.4f}"}
                  for chunk in filtered_chunks]

        response = QueryResponse(
            answer=answer,
            sources=sources,
            question=request.question
        )

        logger.info(f"Query processed successfully. Found {len(filtered_chunks)} relevant chunks passing the threshold.")
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