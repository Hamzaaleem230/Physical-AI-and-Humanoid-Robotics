import os
from pathlib import Path
from typing import List, Dict
from qdrant_client import QdrantClient, models
from backend.scripts.parse_docs import parse_docusaurus_docs, chunk_text
from backend.app.services.embedding_service import EmbeddingService
from backend.app.services.vector_store import get_qdrant_client

def upload_chunks_to_qdrant(
    qdrant_client: QdrantClient,
    collection_name: str,
    chunks_with_metadata: List[Dict],
):
    """
    Upserts embedded text chunks into the Qdrant collection.
    """
    points = []
    for i, chunk_data in enumerate(chunks_with_metadata):
        points.append(
            models.PointStruct(
                id=i,  # Simple incremental ID, consider more robust IDs for production
                vector=chunk_data["embedding"],
                payload={
                    "text": chunk_data["text"],
                    "file_path": chunk_data["file_path"],
                    # Add other metadata as needed
                },
            )
        )
    
    qdrant_client.upsert(
        collection_name=collection_name,
        points=points,
        wait=True,
    )
    print(f"Upserted {len(points)} points to Qdrant collection '{collection_name}'.")

def full_ingestion_pipeline(
    docs_root_path: Path,
    collection_name: str = "physical_ai_book",
    max_chunk_size: int = 500,
    overlap: int = 50,
):
    """
    Orchestrates the full ingestion pipeline: parse -> chunk -> embed -> upsert.
    """
    print(f"Starting ingestion pipeline for docs at: {docs_root_path}")
    
    # 1. Parse documents
    documents = parse_docusaurus_docs(docs_root_path)
    print(f"Found {len(documents)} documents.")

    if not documents:
        print("No documents found to ingest.")
        return

    embedding_service = EmbeddingService()
    qdrant_client = get_qdrant_client()

    # Ensure collection exists
    qdrant_client.recreate_collection(
        collection_name=collection_name,
        vectors_config=models.VectorParams(size=128, distance=models.Distance.COSINE), # Assuming 128 dimensions from dummy embedding
    )
    print(f"Collection '{collection_name}' ensured.")

    all_chunks_with_metadata = []
    for doc in documents:
        # 2. Chunk content
        chunks = chunk_text(doc["content"], max_chunk_size, overlap)
        print(f"Document '{doc['file_path']}' generated {len(chunks)} chunks.")

        for chunk in chunks:
            # 3. Generate embeddings
            embedding = embedding_service.get_embedding(chunk)
            all_chunks_with_metadata.append(
                {
                    "text": chunk,
                    "file_path": doc["file_path"],
                    "embedding": embedding,
                }
            )
    
    # 4. Upsert embeddings to Qdrant
    if all_chunks_with_metadata:
        upload_chunks_to_qdrant(qdrant_client, collection_name, all_chunks_with_metadata)
    else:
        print("No chunks generated for ingestion.")

if __name__ == "__main__":
    # Ensure QDRANT_URL and QDRANT_API_KEY are set in your environment
    # For local testing, ensure Qdrant is running (e.g., via Docker)
    os.environ["QDRANT_URL"] = "http://localhost:6333"  # Replace with your Qdrant instance
    os.environ["QDRANT_API_KEY"] = "your_qdrant_api_key"  # Replace with your API key

    docusaurus_docs_root = Path("../Physical-AI-and-Humanoid-Robotics-Book/docs")
    if not docusaurus_docs_root.exists():
        print(f"Error: Docusaurus docs path not found at {docusaurus_docs_root}")
    else:
        full_ingestion_pipeline(docusaurus_docs_root)
