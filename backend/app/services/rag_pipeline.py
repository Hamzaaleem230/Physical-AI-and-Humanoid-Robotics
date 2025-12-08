# File: backend/app/services/rag_pipeline.py

from typing import List, Dict, Union
import os
# Qdrant related imports (inhe rehne de, magar use nahi karenge)
from qdrant_client import QdrantClient
from .vector_store import get_qdrant_client 

# LLM import (Yeh abhi bhi zaroori hai)
from .llm_client import generate_llm_response 

# Placeholder for an embedding service client 
class EmbeddingServiceClient:
    def __init__(self):
        pass
    def get_embedding(self, text: str) -> List[float]:
        # Dummy embedding - iski zaroorat nahi padegi
        return [0.0] * 128 


# 🛑 retrieve_context_from_qdrant function ko HATA do ya use is tarah badal do:
def retrieve_context_from_qdrant(
    query_embedding: List[float],
    qdrant_client: QdrantClient, # Ab yeh dummy QdrantClient hai
    collection_name: str,
    limit: int = 5,
) -> List[Dict]:
    """
    TEMPORARY FIX: Always return an empty list to bypass Qdrant Client errors.
    """
    # print("DEBUG: Qdrant retrieval is bypassed to ensure LLM generation.")
    return []
# 🛑


def rag_pipeline(query: str, context: str = None, collection_name: str = "physical_ai_book") -> str:
    """
    Orchestrates the RAG pipeline. (Context retrieval is now bypassed)
    """
    
    # Context retrieval logic ko SIMPLE bana dein
    if context:
        # Agar direct context diya gaya hai, use karo
        combined_context = f"Provided context:\n{context}"
    else:
        # Qdrant ko call karne ki bajaye, seedha 'No context found' use karo
        combined_context = "No relevant context found. Answering based on general knowledge."


    # --- LLM Prompt aur Generation Step (Yeh ZAROORI hai) ---
    
    # 1. Prompt template tayyar karein
    prompt_template = f"""
You are a **highly specialized** and **friendly** AI assistant named **"Inquister"**. Your **sole purpose** is to provide **accurate, concise, and helpful** answers based **ONLY** on the content of the book: **"Physical AI and Humanoid Robotics"**.

**STRICT INSTRUCTIONS FOR BEHAVIOR AND FOCUS:**

1.  **Core Focus (Strict):** Answer questions strictly related to the book's chapters and topics (Physical AI, Humanoid Robotics, ROS 2, Simulation, VLA Systems, Integration). All other questions (sports, news, politics, religion, general knowledge, or private matters) must be **immediately rejected**.
2.  **Safety & Tone (Strict):**
    * If the user uses **abusive** or **inappropriate** language, **politely decline** to respond and ask them to return to the technical topic. **Never** respond in kind.
    * **Do not** answer questions about **Government, Politics, or Religion** under any circumstance.
3.  **Language Switching (Crucial):** Respond **ONLY** in the language the user used in their query (English, Urdu Roman, Hindi, etc.).
4.  **Length Control:**
    * Answers must be **concise** and **to-the-point**.
    * Provide a **detailed** or **longer response** only if the user specifically requests 'detailed', 'in-depth', or 'explain thoroughly'.
5.  **Out-of-Scope Response (Exact):** If the question is outside the scope of the book, use this **exact** response:
    **"I can only answer questions related to the Physical AI and Humanoid Robotics book. Please ask about the book's topics."** Then, provide a bulleted list of the book's core topics (e.g., Chapter 1: Physical AI, Chapter 5: VLA Systems).
6.  **Greeting:** If the user sends a simple greeting ("Hello," "Hi"), respond warmly and immediately **redirect** them by asking what book topic they'd like to discuss.

CONTEXT:
{combined_context}
    
QUESTION: {query}
    
Answer the QUESTION according to the STRICT INSTRUCTIONS above.
"""
    
    # 2. LLM ko call karein
    try:
        final_answer = generate_llm_response(prompt_template)
        return final_answer
        
    except Exception as e:
        # This handles the LLM API error
        return f"Failed to generate LLM response (API Key or Client Error or Quota Exceeded). Please check the API key and quota. Error: {e}"