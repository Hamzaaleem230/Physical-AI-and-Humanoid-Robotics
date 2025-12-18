from pydantic import BaseModel
from typing import List, Optional
from uuid import UUID

class UserPreferenceSchema(BaseModel):
    skill_level: str # Enum: 'Beginner', 'Intermediate', 'Expert'
    interests: List[str] = []
    learning_style: str # Enum: 'Theoretical', 'Practical', 'Balanced'

    class Config:
        from_attributes = True

class ReadingHistorySchema(BaseModel):
    content_id: str
    engagement_score: int

    class Config:
        from_attributes = True

class UserPreferenceCreateUpdateSchema(BaseModel):
    skill_level: str
    interests: Optional[List[str]] = None
    learning_style: str

class RecommendationItemSchema(BaseModel):
    contentId: str
    title: str

class RecommendationResponseSchema(BaseModel):
    recommendations: List[RecommendationItemSchema]
