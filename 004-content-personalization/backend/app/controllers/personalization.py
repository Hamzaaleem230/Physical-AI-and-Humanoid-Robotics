from fastapi import APIRouter, Depends, HTTPException, status
from sqlalchemy.orm import Session
from uuid import UUID
from typing import List

from ..middleware import validate_token
from ..models.database import get_db
from ..services.preference_service import PreferenceService
from ..services.tracking_service import TrackingService
from ..services.recommendation_service import RecommendationService
from ..schemas import UserPreferenceSchema, UserPreferenceCreateUpdateSchema, ReadingHistorySchema, RecommendationResponseSchema

router = APIRouter()

@router.get("/preferences", response_model=UserPreferenceSchema)
async def get_preferences(
    user_id: UUID = Depends(validate_token),
    db: Session = Depends(get_db)
):
    """
    Retrieve personalization preferences for the currently authenticated user.
    """
    preference_service = PreferenceService(db)
    preferences = preference_service.get_user_preferences(user_id)

    if not preferences:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail="User preferences not found"
        )
    return preferences

@router.post("/preferences", response_model=UserPreferenceSchema)
async def create_or_update_preferences(
    preference_data: UserPreferenceCreateUpdateSchema,
    user_id: UUID = Depends(validate_token),
    db: Session = Depends(get_db)
):
    """
    Create new user preferences or update existing ones.
    """
    preference_service = PreferenceService(db)
    preferences = preference_service.create_or_update_preferences(
        user_id,
        preference_data.skill_level,
        preference_data.interests if preference_data.interests is not None else [],
        preference_data.learning_style
    )
    return preferences

@router.post("/track", status_code=status.HTTP_202_ACCEPTED)
async def track_user_interaction(
    tracking_data: ReadingHistorySchema,
    user_id: UUID = Depends(validate_token),
    db: Session = Depends(get_db)
):
    """
    Records a user's interaction with a piece of content.
    """
    tracking_service = TrackingService(db)
    tracking_service.record_reading_history(
        user_id,
        tracking_data.content_id,
        tracking_data.engagement_score
    )
    return {"message": "Interaction tracked successfully"}

@router.get("/recommendations", response_model=RecommendationResponseSchema)
async def get_recommendations(
    user_id: UUID = Depends(validate_token),
    db: Session = Depends(get_db)
):
    """
    Provides a list of content recommendations tailored to the authenticated user's profile and reading history.
    """
    recommendation_service = RecommendationService(db)
    recommendations = recommendation_service.get_recommendations(user_id)
    return {"recommendations": recommendations}
