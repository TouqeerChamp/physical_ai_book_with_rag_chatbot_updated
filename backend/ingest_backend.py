import os
import asyncio
from pathlib import Path
from dotenv import load_dotenv
import logging
from typing import List, Tuple
from qdrant_client import QdrantClient
from qdrant_client.http import models
from sentence_transformers import SentenceTransformer
import numpy as np

# Load environment variables
load_dotenv()

# Set up logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Configuration constants
QDRANT_HOST = os.getenv("QDRANT_HOST", "localhost")
QDRANT_PORT = int(os.getenv("QDRANT_PORT", 6333))
COLLECTION_NAME = os.getenv("QDRANT_COLLECTION", "humanoid_robotics")
EMBEDDING_MODEL_NAME = os.getenv("EMBEDDING_MODEL", "all-MiniLM-L6-v2")
DOCS_PATH = Path("./docs")  # Path to your textbook content

# Initialize clients
qdrant_client = QdrantClient(host=QDRANT_HOST, port=QDRANT_PORT)
embedder = SentenceTransformer(EMBEDDING_MODEL_NAME)


def chunk_text(text: str, chunk_size: int = 512, overlap: int = 50) -> List[str]:
    """
    Split text into overlapping chunks
    """
    chunks = []
    start = 0
    
    while start < len(text):
        end = start + chunk_size
        
        # Find the nearest sentence boundary if possible
        if end < len(text):
            # Look for period, exclamation mark, or question mark
            for punct in '.!?':
                last_punct = text.rfind(punct, start, end)
                if last_punct != -1 and last_punct > start + chunk_size // 2:
                    end = last_punct + 1
                    break
        
        chunk = text[start:end].strip()
        if chunk:
            chunks.append(chunk)
        
        # Move start position forward, considering overlap
        start = end - overlap if end < len(text) else len(text)
        
        # If no progress is being made, advance by chunk_size anyway
        if start == end:
            start = end
    
    return chunks


def read_docs(docs_path: Path) -> List[Tuple[str, str]]:
    """
    Read all markdown files from the docs directory
    Returns list of tuples (content, source_filename)
    """
    docs_data = []
    
    for file_path in docs_path.glob("*.md"):
        logger.info(f"Reading file: {file_path.name}")
        try:
            with open(file_path, 'r', encoding='utf-8') as f:
                content = f.read()
                
                # Skip if content is too short (probably just frontmatter)
                if len(content.strip()) < 50:
                    logger.warning(f"Skipping {file_path.name} due to short length")
                    continue
                    
                docs_data.append((content, file_path.name))
        except Exception as e:
            logger.error(f"Error reading file {file_path.name}: {str(e)}")
    
    return docs_data


def create_collection_if_not_exists():
    """
    Create the Qdrant collection if it doesn't exist
    """
    try:
        # Get existing collections
        collections = qdrant_client.get_collections()
        collection_names = [col.name for col in collections.collections]
        
        # Determine embedding dimension
        sample_embedding = embedder.encode(["sample text"])
        embedding_size = len(sample_embedding[0])
        
        if COLLECTION_NAME not in collection_names:
            logger.info(f"Creating collection: {COLLECTION_NAME}")
            qdrant_client.create_collection(
                collection_name=COLLECTION_NAME,
                vectors_config=models.VectorParams(
                    size=embedding_size,
                    distance=models.Distance.COSINE
                ),
            )
            logger.info(f"Collection {COLLECTION_NAME} created successfully")
        else:
            logger.info(f"Collection {COLLECTION_NAME} already exists")
            
    except Exception as e:
        logger.error(f"Error creating collection: {str(e)}")
        raise


async def ingest_documents():
    """
    Main ingestion function to process all documents and store embeddings in Qdrant
    """
    logger.info("Starting ingestion process...")
    
    # Create collection if it doesn't exist
    create_collection_if_not_exists()
    
    # Read documents
    docs_data = read_docs(DOCS_PATH)
    
    if not docs_data:
        logger.error("No documents found to ingest. Please check the docs directory.")
        return
    
    # Prepare points for uploading
    points = []
    point_id = 0
    
    for content, source in docs_data:
        logger.info(f"Processing chunks for {source}")
        
        # Split content into chunks
        chunks = chunk_text(content)
        
        for chunk_idx, chunk in enumerate(chunks):
            # Generate embedding for the chunk
            embedding = embedder.encode([chunk])[0].tolist()
            
            # Create a Qdrant point
            point = models.PointStruct(
                id=point_id,
                vector=embedding,
                payload={
                    "text": chunk,
                    "source": source,
                    "chunk_index": chunk_idx
                }
            )
            
            points.append(point)
            point_id += 1
            
            # Log progress every 10 points
            if point_id % 10 == 0:
                logger.info(f"Processed {point_id} points so far...")
    
    logger.info(f"Uploading {len(points)} points to collection {COLLECTION_NAME}")
    
    # Upload points to Qdrant in batches
    batch_size = 100
    for i in range(0, len(points), batch_size):
        batch = points[i:i + batch_size]
        
        try:
            qdrant_client.upsert(
                collection_name=COLLECTION_NAME,
                points=batch,
                wait=True
            )
            logger.info(f"Uploaded batch {i // batch_size + 1}/{(len(points) - 1) // batch_size + 1}")
        except Exception as e:
            logger.error(f"Error uploading batch {i // batch_size + 1}: {str(e)}")
            raise
    
    logger.info(f"Ingestion complete! Uploaded {len(points)} points to collection {COLLECTION_NAME}")


if __name__ == "__main__":
    asyncio.run(ingest_documents())