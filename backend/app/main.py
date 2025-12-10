import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..')))


from fastapi import FastAPI, Request, HTTPException, BackgroundTasks
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import StreamingResponse
from typing import AsyncGenerator
import asyncio
from pathlib import Path

from .services.rag_pipeline import rag_pipeline
from backend.scripts.upload_to_qdrant import full_ingestion_pipeline # Import the ingestion pipeline

app = FastAPI(
    title="RAG Chatbot API",
    description="API for the Physical AI book's RAG chatbot.",
    version="1.0.0",
)

origins = [
    # Aapka Docusaurus development server yahan chalta hai
    "http://localhost:3000",
    "http://127.0.0.1:3000",
    # Agar aapne Docusaurus ko kisi aur port par chalaaya tha (jaise 3001) to woh bhi add karein
    "http://localhost:3001",
]

app.add_middleware(
    CORSMiddleware,
    allow_origins=origins, # Sirf allowed origins
    allow_credentials=True,
    allow_methods=["*"],    # Sabhi HTTP methods (GET, POST, OPTIONS, etc.) allow karein
    allow_headers=["*"],    # Sabhi headers allow karein
)

@app.get("/health", summary="Health Check")
def health_check():
    """
    Returns the operational status of the API.
    """
    return {"status": "ok"}

@app.get("/")
def read_root():
    return {"message": "Welcome to the RAG Chatbot API"}


@app.post("/chat", summary="Chat with RAG Chatbot", response_class=StreamingResponse)
async def chat_with_rag(request: Request) -> StreamingResponse:
    """
    Receives a user message, processes it through the RAG pipeline, and streams the response.
    """
    try:
        data = await request.json()
        user_message = data.get("message")
        context = data.get("context") # Extract optional context
        if not user_message:
            return StreamingResponse(
                content=iter(["Error: No message provided."]),
                media_type="text/plain",
                status_code=400,
            )

        async def generate_response():
            full_response = rag_pipeline(user_message, context) # Pass context to rag_pipeline
            yield full_response

        # Add CORS headers if necessary, especially during development
        response = StreamingResponse(generate_response(), media_type="text/plain")
        return response

    except Exception as e:
        return StreamingResponse(
            content=iter([f"Error: {str(e)}"]),
            media_type="text/plain",
            status_code=500,
        )

# Define the path to the Docusaurus docs relative to the project root
# Assuming 'backend' is at the project root and 'Physical-AI-and-Humanoid-Robotics-Book' is a sibling
DOCUSAURUS_DOCS_ROOT = Path(__file__).parent.parent.parent / "Physical-AI-and-Humanoid-Robotics-Book" / "docs"

@app.post("/ingest-book", summary="Ingest Docusaurus Book Content")
async def ingest_book(background_tasks: BackgroundTasks):
    """
    Triggers the ingestion pipeline to parse, chunk, embed, and upsert
    the Docusaurus book content into Qdrant.
    """
    if not DOCUSAURUS_DOCS_ROOT.exists():
        raise HTTPException(status_code=404, detail=f"Docusaurus docs path not found at {DOCUSAURUS_DOCS_ROOT}")
    
    # Run the ingestion pipeline in a background task to avoid blocking the API
    background_tasks.add_task(full_ingestion_pipeline, DOCUSAURUS_DOCS_ROOT)
    
    return {"message": "Ingestion pipeline triggered successfully. Check logs for progress."}