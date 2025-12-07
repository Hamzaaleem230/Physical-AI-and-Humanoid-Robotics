# app/services/llm_client.py

import os
# Import the Client class directly
from google.genai.client import Client 

def generate_llm_response(prompt: str) -> str:
    """Sends the final prompt to the LLM and gets the response."""
    
    # Assuming LLM_API_KEY is loaded via python-dotenv
    api_key = os.getenv("GOOGLE_API_KEY") 
    
    if not api_key:
        return "Error: LLM API key not found."

    try:
        # 1. Client Initialize karein - Ab sirf Client use hoga
        client = Client(api_key=api_key)
        
        # 2. Model call karein
        response = client.models.generate_content(
            model=os.getenv("LLM_MODEL", "gemini-2.5-flash"), 
            contents=prompt
        )
        
        return response.text
    except Exception as e:
        return f"LLM API Error: {e}"