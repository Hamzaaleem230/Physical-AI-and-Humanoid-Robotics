import os
from dotenv import load_dotenv # Zaroorat hai agar aap .env file se load kar rahe hain
from fastapi import FastAPI, Depends, HTTPException, status
from fastapi.middleware.cors import CORSMiddleware
from sqlalchemy.orm import Session
from pydantic import BaseModel, EmailStr
from typing import Optional

# 🛑 FIX: Database aur Models ke imports ko theek karna zaroori hai.
# Agar database.py aur models.py ek hi 'app' folder mein hain,
# toh unhe relative path se import karna chahiye.

# Isko do baar import karne ki zaroorat nahi hai: from .database import get_db, create_db_tables
from .database import get_db, create_db_tables
from .models import User, Profile # User aur Profile models yahan se aane chahiye

# --- Load environment variables (.env) ---
load_dotenv()

app = FastAPI()

# --- CORS Configuration ---
# Get CORS_ORIGIN from environment variable
CORS_ORIGIN = os.getenv("CORS_ORIGIN", "http://localhost:3000") # Default to localhost for dev

origins = [
    CORS_ORIGIN,
    # You might add other origins for specific development/staging environments
]

app.add_middleware(
    CORSMiddleware,
    allow_origins=origins,
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)
# --- End CORS Configuration ---

# Pydantic models for request bodies
class UserCreate(BaseModel):
    email: EmailStr
    password: str
    full_name: str
    skill_level: Optional[str] = None
    hardware: Optional[str] = None
    robotics_experience: Optional[str] = None
    os: Optional[str] = None
    learning_mode: Optional[str] = None

class UserLogin(BaseModel):
    email: EmailStr
    password: str

# Pydantic model for User Profile response
class UserProfileResponse(BaseModel):
    email: EmailStr
    full_name: str
    skill_level: Optional[str] = None
    hardware: Optional[str] = None
    robotics_experience: Optional[str] = None
    os: Optional[str] = None
    learning_mode: Optional[str] = None

    class Config:
        from_attributes = True # V2 Pydantic: orm_mode ko rename karke from_attributes kar diya gaya hai.

# Pydantic model for Profile Update request
class ProfileUpdate(BaseModel):
    full_name: Optional[str] = None
    skill_level: Optional[str] = None
    hardware: Optional[str] = None
    robotics_experience: Optional[str] = None
    os: Optional[str] = None
    learning_mode: Optional[str] = None

# --- Startup Event Handler ---
@app.on_event("startup")
def on_startup():
    # Tables ab Uvicorn ke start hone par hamesha ban jayengi
    create_db_tables() 
# --- End Startup Event Handler ---

@app.get("/")
def read_root():
    return {"message": "Auth Service is running!"}

@app.post("/auth/signup")
async def signup(user_data: UserCreate, db: Session = Depends(get_db)):
    # Check if user already exists
    db_user = db.query(User).filter(User.email == user_data.email).first()
    if db_user:
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail="Email already registered"
        )

    # TODO: Hash password using better-auth or a suitable utility
    # For now, storing as plain text (DANGER: DO NOT USE IN PRODUCTION)
    hashed_password = user_data.password 

    # Create new user
    new_user = User(
        email=user_data.email,
        password_hash=hashed_password # Store hashed password
    )
    db.add(new_user)
    db.commit()
    db.refresh(new_user)

    # Create user profile
    new_profile = Profile(
        user_id=new_user.id,
        full_name=user_data.full_name,
        skill_level=user_data.skill_level,
        hardware=user_data.hardware,
        robotics_experience=user_data.robotics_experience,
        os=user_data.os,
        learning_mode=user_data.learning_mode,
    )
    db.add(new_profile)
    db.commit()
    db.refresh(new_profile)

    return {"message": "User and profile created successfully", "user_id": new_user.id}

@app.post("/auth/signin")
async def signin(user_credentials: UserLogin, db: Session = Depends(get_db)):
    db_user = db.query(User).filter(User.email == user_credentials.email).first()
    if not db_user:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Incorrect email or password"
        )

    # TODO: Verify password using better-auth or a suitable utility
    # For now, simply comparing plain text (DANGER: DO NOT USE IN PRODUCTION)
    if db_user.password_hash != user_credentials.password:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Incorrect email or password"
        )

    # TODO: Implement session management (e.g., create JWT and set HTTP-only cookie)
    # For now, return a success message
    return {"message": "User signed in successfully", "user_id": db_user.id}

# C:\Users\syeda\...\auth-server\app\main.py

@app.get("/auth/me", response_model=UserProfileResponse)
async def get_current_user_profile(db: Session = Depends(get_db)):
    # TODO: Implement actual authentication to get current user ID from session/token
    # NOTE: Since signin is implemented using cookies, the logic needs to be updated 
    # to retrieve the user based on the session/cookie ID. 
    
    # FOR NOW (Placeholder fix): Still fetch the first user, but correctly combine data
    current_user = db.query(User).join(Profile).first() 
    
    if not current_user:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail="No user found (authentication not implemented yet)"
        )
    
    # Agar current_user.profile mojood nahi hai, toh raise error
    if not current_user.profile:
         raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail="User profile not found in DB"
        )
        
    # 🔥 FIX: User aur Profile data ko combine karke AuthUserProfile format mein return karein
    # taake response validation pass ho sake (email field shaamil ho).
    return {
        "email": current_user.email,  # ✅ User object se email field liya
        "full_name": current_user.profile.full_name,
        "skill_level": current_user.profile.skill_level,
        "hardware": current_user.profile.hardware,
        "robotics_experience": current_user.profile.robotics_experience,
        "os": current_user.profile.os,
        "learning_mode": current_user.profile.learning_mode,
        # Agar UserProfileResponse mein aur fields hain, toh unko bhi yahan add karein.
    }
@app.post("/auth/signout")
async def signout():
    # TODO: Implement actual session termination (e.g., clear JWT cookie)
    return {"message": "User signed out successfully"}

@app.post("/auth/profile-update")
async def update_user_profile(profile_data: ProfileUpdate, db: Session = Depends(get_db)):
    # TODO: Implement actual authentication to get current user ID from session/token
    # For now, assume a user is authenticated and update the first user's profile
    current_user = db.query(User).join(Profile).first() # Get user with profile
    if not current_user:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail="No user found (authentication not implemented yet)"
        )
    
    # Update profile fields
    # .dict() is Pydantic V1 style, V2 mein .model_dump() istemal hota hai.
    # Maine isko .dict() hi rehne diya hai kyunki aapka code V1 style use kar raha hai.
    for field, value in profile_data.dict(exclude_unset=True).items():
        setattr(current_user.profile, field, value)
    
    db.commit()
    db.refresh(current_user.profile)

    return {"message": "Profile updated successfully", "user_id": current_user.id}