# C:\Users\syeda\...\backend\auth-server\app\models.py

from sqlalchemy import Column, String, TIMESTAMP, ForeignKey
from sqlalchemy.orm import relationship
from sqlalchemy.sql import func
from .database import Base
import uuid

# NOTE:
# Using String(36) to store UUIDs as text makes this compatible with SQLite
# and avoids dialect-specific UUID column issues.
def gen_uuid_str():
    return str(uuid.uuid4())

class User(Base):
    __tablename__ = "users"

    id = Column(String(36), primary_key=True, default=gen_uuid_str)
    email = Column(String(255), unique=True, nullable=False)
    password_hash = Column(String(255), nullable=False)
    created_at = Column(TIMESTAMP(timezone=True), server_default=func.now())

    profile = relationship("Profile", back_populates="user", uselist=False)

    def __repr__(self):
        return f"<User(id='{self.id}', email='{self.email}')>"

class Profile(Base):
    __tablename__ = "profiles"

    id = Column(String(36), primary_key=True, default=gen_uuid_str)
    user_id = Column(String(36), ForeignKey("users.id"), nullable=False)
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
