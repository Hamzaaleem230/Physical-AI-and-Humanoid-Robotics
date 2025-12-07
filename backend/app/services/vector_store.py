from qdrant_client import QdrantClient
from qdrant_client.models import Distance, VectorParams
import os

# Assumption: Embeddings ki dimension 128 hai jo aapke dummy client mein hai
# Agar aap OpenAI ya dusra model use karte, to yeh 1536 ya 768 ho sakti thi.
VECTOR_DIMENSION = 128
QDRANT_URL = os.getenv("QDRANT_URL")
QDRANT_API_KEY = os.getenv("QDRANT_API_KEY")

def get_qdrant_client() -> QdrantClient:
    """
    Initializes and returns the Qdrant client.
    Uses remote connection if QDRANT_URL and QDRANT_API_KEY are provided,
    otherwise initializes an in-memory client.
    """
    if QDRANT_URL and QDRANT_API_KEY:
        print("INFO: Initializing Qdrant in REMOTE mode.")
        return QdrantClient(
            url=QDRANT_URL,
            api_key=QDRANT_API_KEY,
        )
    else:
        print("INFO: Initializing Qdrant in LOCAL (In-Memory) mode.")
        # In-memory client will be used if environment variables are missing
        # NOTE: Agar aapke paas data files hain, toh aap path= argument bhi de sakte hain.
        return QdrantClient(":memory:") 

def initialize_collection(client: QdrantClient, collection_name: str):
    """
    Checks if a collection exists and creates it if it doesn't.
    """
    try:
        # Check if the collection already exists
        client.get_collection(collection_name=collection_name)
        print(f"INFO: Collection '{collection_name}' already exists.")
    except Exception:
        # If collection does not exist, create it
        print(f"INFO: Creating collection '{collection_name}'.")
        client.recreate_collection(
            collection_name=collection_name,
            vectors_config=VectorParams(size=VECTOR_DIMENSION, distance=Distance.COSINE),
        )

# Ye check karega ki client theek se init ho raha hai ya nahi. 
# Lekin aapka code ab ise call nahi karega.
if __name__ == "__main__":
    try:
        qdrant_client = get_qdrant_client()
        initialize_collection(qdrant_client, "test_collection")
        print("Qdrant initialization successful.")
    except Exception as e:
        print(f"Error during Qdrant initialization: {e}")