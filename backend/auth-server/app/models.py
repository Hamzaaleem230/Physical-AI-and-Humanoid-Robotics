from sqlalchemy import Column, String, TIMESTAMP, ForeignKey, Uuid # <--- FIX: Uuid import kiya
from sqlalchemy.orm import declarative_base, relationship
from sqlalchemy.sql import func
from .database import Base
import uuid

# Base ko yahan define kiya gaya hai, jo theek hai agar yeh file pehle import ho rahi ho.
# Base = declarative_base()

class User(Base):
    __tablename__ = "users"

    # FIX: Column type ko Uuid() kar diya gaya hai
    id = Column(Uuid(), primary_key=True, default=uuid.uuid4) 
    email = Column(String(255), unique=True, nullable=False)
    password_hash = Column(String(255), nullable=False)
    created_at = Column(TIMESTAMP(timezone=True), server_default=func.now())

    profile = relationship("Profile", back_populates="user", uselist=False)

    def __repr__(self):
        return f"<User(id='{self.id}', email='{self.email}')>"

class Profile(Base):
    __tablename__ = "profiles"

    # FIX: Column type ko Uuid() kar diya gaya hai
    id = Column(Uuid(), primary_key=True, default=uuid.uuid4) 
    # FIX: ForeignKey column type ko bhi Uuid() kar diya gaya hai
    user_id = Column(Uuid(), ForeignKey("users.id"), nullable=False) 
    full_name = Column(String(255))
    skill_level = Column(String(50))
    hardware = Column(String(255))
    robotics_experience = Column(String(255))
    os = Column(String(50))
    learning_mode = Column(String(50))

    user = relationship("User", back_populates="profile")

    def __repr__(self):
        return (
            f"<Profile(id='{self.id}', user_id='{self.user_id}', "
            f"full_name='{self.full_name}')>"
        )