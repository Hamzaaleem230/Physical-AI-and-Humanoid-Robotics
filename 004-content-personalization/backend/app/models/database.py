import os
import uuid
from dotenv import load_dotenv
from sqlalchemy import create_engine, Column, String, Integer, DateTime, Text
from sqlalchemy.ext.declarative import declarative_base
from sqlalchemy.orm import sessionmaker
from sqlalchemy.sql import func

load_dotenv()

DATABASE_URL = os.getenv("DATABASE_URL")

if not DATABASE_URL:
    raise ValueError("DATABASE_URL not found in environment variables.")

# --- Engine Setup (Neon/Postgres vs SQLite) ---
if DATABASE_URL.startswith("postgresql"):
    # Neon/PostgreSQL ke liye simple engine kaafi hai
    engine = create_engine(DATABASE_URL)
else:
    # SQLite (Local) ke liye check_same_thread zaroori hai
    engine = create_engine(DATABASE_URL, connect_args={"check_same_thread": False})

SessionLocal = sessionmaker(autocommit=False, autoflush=False, bind=engine)
Base = declarative_base()

def create_db_tables():
    """Database tables create karne ke liye startup par call karein."""
    Base.metadata.create_all(bind=engine)

def get_db():
    """Database session dependency."""
    db = SessionLocal()
    try:
        yield db
    finally:
        db.close()

# --- Models ---

class UserPreference(Base):
    __tablename__ = "user_preferences"

    id = Column(String, primary_key=True, default=lambda: str(uuid.uuid4()))
    user_id = Column(String, index=True, nullable=False) 
    skill_level = Column(String, nullable=False)
    # Note: Service mein json.dumps aur json.loads use karein is column ke liye
    interests = Column(Text, default="[]") 
    learning_style = Column(String, nullable=False)
    created_at = Column(DateTime(timezone=True), server_default=func.now())
    updated_at = Column(DateTime(timezone=True), onupdate=func.now(), server_default=func.now())

class ReadingHistory(Base):
    __tablename__ = "reading_history"

    id = Column(String, primary_key=True, default=lambda: str(uuid.uuid4()))
    user_id = Column(String, index=True, nullable=False)
    content_id = Column(String, nullable=False)
    engagement_score = Column(Integer, nullable=False)
    timestamp = Column(DateTime(timezone=True), server_default=func.now())