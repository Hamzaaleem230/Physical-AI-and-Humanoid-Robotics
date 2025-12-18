from fastapi import HTTPException, Security, Request
from fastapi.security import HTTPBearer, HTTPAuthorizationCredentials
import httpx
import os
from dotenv import load_dotenv

load_dotenv()

# HF ki settings mein AUTH_SERVICE_URL lazmi dalna (e.g., https://username-auth-space.hf.space)
AUTH_SERVICE_URL = os.getenv("AUTH_SERVICE_URL")
SECRET_KEY = os.getenv("SECRET_KEY")

security = HTTPBearer()

async def validate_token(credentials: HTTPAuthorizationCredentials = Security(security)) -> str:
    token = credentials.credentials
    if not token:
        raise HTTPException(status_code=401, detail="Not authenticated")

    # Production logic for Hugging Face
    try:
        async with httpx.AsyncClient() as client:
            # Aapki doosri Space (Auth) ko call karega
            response = await client.post(
                f"{AUTH_SERVICE_URL}/validate_token",
                json={"token": token},
                timeout=10.0
            )
            
            if response.status_code != 200:
                # Agar token invalid hai ya auth service fail ho gayi
                raise HTTPException(status_code=401, detail="Invalid token or Auth service error")
                
            user_info = response.json()
            user_id = user_info.get("user_id") or user_info.get("id")
            
            if not user_id:
                raise HTTPException(status_code=401, detail="User ID missing in token response")
            
            return str(user_id)

    except httpx.RequestError as e:
        raise HTTPException(status_code=500, detail=f"Cannot reach Auth service: {e}")
    except Exception as e:
        raise HTTPException(status_code=401, detail=f"Auth failed: {e}")