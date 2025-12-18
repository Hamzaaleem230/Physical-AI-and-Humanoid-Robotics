import json
from sqlalchemy.orm import Session
from ..models.database import UserPreference

class PreferenceService:
    def __init__(self, db: Session):
        self.db = db

    def get_user_preferences(self, user_id: str) -> UserPreference | None:
        """Retrieves user preferences and converts interests back to a list."""
        pref = self.db.query(UserPreference).filter(UserPreference.user_id == str(user_id)).first()
        if pref and isinstance(pref.interests, str):
            try:
                # Database ki string ko wapas list banayein
                pref.interests = json.loads(pref.interests)
            except:
                pref.interests = []
        return pref

    def create_or_update_preferences(self, user_id: str, skill_level: str, interests: list[str], learning_style: str) -> UserPreference:
        interests_json = json.dumps(interests) if isinstance(interests, list) else interests
        user_id_str = str(user_id)
        
        # Check if exists
        preferences = self.db.query(UserPreference).filter(UserPreference.user_id == user_id_str).first()
        
        if preferences:
            preferences.skill_level = skill_level
            preferences.interests = interests_json
            preferences.learning_style = learning_style
        else:
            preferences = UserPreference(
                user_id=user_id_str,
                skill_level=skill_level,
                interests=interests_json,
                learning_style=learning_style
            )
            self.db.add(preferences)
            
        self.db.commit()
        self.db.refresh(preferences)
        
        # Return karne se pehle wapas list banayein taake Fastapi validation fail na ho
        if isinstance(preferences.interests, str):
            preferences.interests = json.loads(preferences.interests)
            
        return preferences