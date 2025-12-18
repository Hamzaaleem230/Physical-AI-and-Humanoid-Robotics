# Data Model for Content Personalization

**Feature**: `004-content-personalization`  
**Created**: 2025-12-17
**Source**: `specs/004-content-personalization/spec.md`, `specs/004-content-personalization/plan.md`

## Overview

The personalization data model consists of two main entities: `UserPreference` and `ReadingHistory`. These entities capture explicit user preferences and implicit behavioral data to enable personalized content delivery and recommendations. Both entities are linked to the user's identity managed by the `003-auth-system`.

## Entities

### UserPreference

This entity stores the explicit learning preferences set by the user.

-   **Fields**:
    -   `id`:
        -   Type: Primary Key (UUID)
        -   Description: Unique identifier for the preference record.
    -   `userId`:
        -   Type: UUID
        -   Description: Foreign Key referencing the user table in `003-auth-system`. This links preferences to a specific authenticated user.
        -   Constraints: Indexed, Required.
    -   `skillLevel`:
        -   Type: String (Enum)
        -   Description: The user's self-declared skill level.
        -   Enum Values: 'Beginner', 'Intermediate', 'Expert'
        -   Constraints: Required.
    -   `interests`:
        -   Type: Array of Strings
        -   Description: A list of topics the user is interested in (e.g., ['ROS2', 'Computer Vision']).
        -   Constraints: Can be empty.
    -   `learningStyle`:
        -   Type: String (Enum)
        -   Description: The user's preferred learning style.
        -   Enum Values: 'Theoretical', 'Practical', 'Balanced'
        -   Constraints: Required.
    -   `createdAt`:
        -   Type: Timestamp
        -   Description: Timestamp of when the record was created.
        -   Constraints: Auto-generated.
    -   `updatedAt`:
        -   Type: Timestamp
        -   Description: Timestamp of the last update to the record.
        -   Constraints: Auto-updated.

### ReadingHistory

This entity tracks the implicit behavior of the user, specifically their interaction with content.

-   **Fields**:
    -   `id`:
        -   Type: Primary Key (UUID)
        -   Description: Unique identifier for the reading history record.
    -   `userId`:
        -   Type: UUID
        -   Description: Foreign Key referencing the user table in `003-auth-system`. Links history to a specific authenticated user.
        -   Constraints: Indexed, Required.
    -   `contentId`:
        -   Type: String
        -   Description: A unique, human-readable slug for each chapter or page read (e.g., 'ros2-architecture'). This acts as an identifier for the content itself.
        -   Constraints: Required.
    -   `timestamp`:
        -   Type: Timestamp
        -   Description: The time when the content interaction was recorded.
        -   Constraints: Auto-generated.
    -   `engagementScore`:
        -   Type: Integer
        -   Description: A metric representing user engagement with the `contentId`. Initially defined as time spent on the page in seconds.
        -   Constraints: Required, Non-negative.

## Relationships

-   **User → UserPreference**: One-to-one relationship. Each user can have one preference profile.
-   **User → ReadingHistory**: One-to-many relationship. Each user can have multiple reading history entries.
-   **No direct relationships with `001-physical-ai-book` or `002-rag-chatbot`**: Integration is purely at the application layer as defined in the system architecture.
