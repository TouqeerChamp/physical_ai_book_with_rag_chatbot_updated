import os
from qdrant_client import QdrantClient
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

# Configuration constants
QDRANT_HOST = os.getenv("QDRANT_HOST", "localhost")
QDRANT_PORT = int(os.getenv("QDRANT_PORT", 6333))

def test_qdrant_connection():
    """
    Test function to check if we can connect to Qdrant
    """
    print("Testing Qdrant connection...")

    try:
        # Initialize Qdrant client
        client = QdrantClient(host=QDRANT_HOST, port=QDRANT_PORT)

        # Try to get collections to verify connection
        collections = client.get_collections()

        print("[SUCCESS] Successfully connected to Qdrant!")
        print(f"Available collections: {[col.name for col in collections.collections]}")

        return True
    except Exception as e:
        print(f"[ERROR] Error connecting to Qdrant: {str(e)}")
        print("Make sure Docker is running and Qdrant container is started.")
        return False

if __name__ == "__main__":
    test_qdrant_connection()