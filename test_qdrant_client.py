from qdrant_client import QdrantClient
from qdrant_client.http import models

# Test script to validate QdrantClient functionality
def test_qdrant_client():
    client = QdrantClient(host="localhost", port=6333)

    print("Testing Qdrant client...")

    # Check if the search method exists
    if hasattr(client, 'search'):
        print("OK: search method exists")
    else:
        print("ERROR: search method does not exist")

    # Check available attributes
    print("Available methods/attributes:", [attr for attr in dir(client) if not attr.startswith('_')])

    # Try to get collections
    try:
        collections = client.get_collections()
        print("Collections:", collections)
    except Exception as e:
        print("Error getting collections:", e)

    # Try a simple search if the method exists
    if hasattr(client, 'search'):
        try:
            # Just check if the collection exists
            collection_name = "humanoid_robotics"
            records_count = client.count(collection_name=collection_name)
            print(f"Records count in {collection_name}:", records_count)
        except Exception as e:
            print("Error searching:", e)

    print("Test complete.")

if __name__ == "__main__":
    test_qdrant_client()