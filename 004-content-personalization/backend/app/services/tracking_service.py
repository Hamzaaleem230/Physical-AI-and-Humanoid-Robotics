from sqlalchemy.orm import Session
from ..models.database import ReadingHistory
from uuid import UUID

class TrackingService:
    def __init__(self, db: Session):
        self.db = db

    def record_reading_history(self, user_id: UUID, content_id: str, engagement_score: int) -> ReadingHistory:
        """
        Records a user's reading history entry.
        """
        reading_entry = ReadingHistory(
            user_id=user_id,
            content_id=content_id,
            engagement_score=engagement_score
        )
        self.db.add(reading_entry)
        self.db.commit()
        self.db.refresh(reading_entry)
        return reading_entry
