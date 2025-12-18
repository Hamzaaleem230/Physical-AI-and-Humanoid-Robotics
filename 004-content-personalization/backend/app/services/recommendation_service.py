from sqlalchemy.orm import Session
from ..models.database import UserPreference, ReadingHistory
from uuid import UUID
from typing import List, Dict

class RecommendationService:
    def __init__(self, db: Session):
        self.db = db
        # Placeholder for available content. In a real application, this would come from a content management system.
        self.all_content = [
            {"contentId": "ros2-architecture", "title": "ROS 2 Architecture Overview", "tags": ["ROS2", "Beginner"]},
            {"contentId": "ros2-nodes", "title": "Understanding ROS 2 Nodes", "tags": ["ROS2", "Beginner"]},
            {"contentId": "ros2-topics", "title": "ROS 2 Topics for Communication", "tags": ["ROS2", "Intermediate"]},
            {"contentId": "isaac-sim-omniverse-introduction", "title": "Introduction to Isaac Sim & Omniverse", "tags": ["Simulation", "Intermediate", "NVIDIA"]},
            {"contentId": "reinforcement-learning-for-control", "title": "Reinforcement Learning for Robot Control", "tags": ["AI", "Expert", "Control Systems"]},
            {"contentId": "urdf-for-humanoids", "title": "URDF for Humanoid Robots", "tags": ["Robotics", "Intermediate"]},
            {"contentId": "computer-vision-basics", "title": "Computer Vision Basics for Robotics", "tags": ["Computer Vision", "Beginner"]},
            {"contentId": "advanced-path-planning", "title": "Advanced Path Planning Algorithms", "tags": ["Robotics", "Expert"]},
            {"contentId": "motor-commands-sensors-control-loops", "title": "Motor Commands, Sensors, and Control Loops", "tags": ["Robotics", "Practical", "Intermediate"]},
            {"contentId": "llm-planning-natural-language-to-ros2-tasks", "title": "LLM Planning: Natural Language to ROS 2 Tasks", "tags": ["AI", "ROS2", "Expert"]},
            {"contentId": "sim-to-real-transfer-design", "title": "Sim-to-Real Transfer Design Principles", "tags": ["Simulation", "Practical", "Expert"]},
        ]

    def get_recommendations(self, user_id: UUID, limit: int = 5) -> List[Dict]:
        """
        Generates personalized content recommendations based on user preferences and reading history.
        """
        user_preference = self.db.query(UserPreference).filter(UserPreference.user_id == user_id).first()
        reading_history = self.db.query(ReadingHistory).filter(ReadingHistory.user_id == user_id).order_by(ReadingHistory.timestamp.desc()).limit(10).all()

        if not user_preference and not reading_history:
            # If no preferences or history, return popular or random content
            return self.all_content[:limit] # Simple fallback

        # Collect factors for recommendation
        user_tags = set()
        if user_preference:
            user_tags.add(user_preference.skill_level)
            user_tags.update(user_preference.interests)
            user_tags.add(user_preference.learning_style)
        
        read_content_ids = {entry.content_id for entry in reading_history}

        # Score content
        scored_content = []
        for content_item in self.all_content:
            if content_item["contentId"] in read_content_ids:
                continue # Do not recommend already read content

            score = 0
            # Boost score for matching interests/skill/learning style
            for tag in content_item.get("tags", []):
                if tag in user_tags:
                    score += 1

            # Future enhancement: boost for similarity to recently read content

            scored_content.append({"content": content_item, "score": score})

        # Sort by score (descending) and return top N
        scored_content.sort(key=lambda x: x["score"], reverse=True)
        
        recommendations = [item["content"] for item in scored_content if item["score"] > 0] # Only recommend if score > 0
        return recommendations[:limit]
