# Implementation Plan: Content Personalization

**Feature Branch**: `004-content-personalization`  
**Created**: 2025-12-17
**Status**: Draft
**Spec**: [specs/004-content-personalization/spec.md](specs/004-content-personalization/spec.md)

## 1. SYSTEM ARCHITECTURE

The Content Personalization feature will be implemented as a new, independent service that integrates with the existing project components in a read-only manner. The architecture is designed to be fully contained within the `004-content-personalization` directory, ensuring zero modification to other project folders.

- **Overall Layout**:
    - A new backend service (FastAPI) will be created inside `004-content-personalization/backend`.
    - A new set of frontend components (React) will be created inside `004-content-personalization/frontend`.
    - A new database schema will be used to store personalization data, managed by the new backend service.
- **Integration**:
    - The new frontend components will be integrated into the main Docusaurus application.
    - The backend service will expose a REST API for managing personalization data.
    - The system will rely on the `003-auth-system` for user authentication and identity.
- **Interaction Flow**:
    1.  The user logs in using the `003-auth-system`.
    2.  The frontend retrieves the user's JWT token.
    3.  The frontend calls the new Personalization API, passing the JWT token for authentication.
    4.  The backend service validates the token and retrieves the user's personalization data.
    5.  The frontend adapts the content and chatbot interaction based on the retrieved data.
- **RAG Chatbot Integration**:
    - The integration will be done client-side. The frontend will fetch the user's personalization profile (e.g., skill level) and prepend a context-setting instruction to the user's query before sending it to the existing RAG chatbot API. This ensures no modification to the chatbot's core logic.
- **Boundaries**:
    - The system will not have write access to the databases or file systems of `001`, `002`, or `003`.
    - All network requests to existing services will be read-only.

## 2. DATA MODEL DESIGN

The data model for the personalization feature will consist of two main tables:

-   **UserPreference**:
    -   `id`: Primary Key (UUID)
    -   `userId`: UUID, Foreign Key to the user table in `003-auth-system` (indexed)
    -   `skillLevel`: String (Enum: 'Beginner', 'Intermediate', 'Expert')
    -   `interests`: Array of Strings
    -   `learningStyle`: String (Enum: 'Theoretical', 'Practical', 'Balanced')
    -   `createdAt`: Timestamp
    -   `updatedAt`: Timestamp
-   **ReadingHistory**:
    -   `id`: Primary Key (UUID)
    -   `userId`: UUID, Foreign Key to the user table in `003-auth-system` (indexed)
    -   `contentId`: String (A unique, human-readable slug for each chapter or page)
    -   `engagementScore`: Integer (Time spent on page in seconds)
    -   `timestamp`: Timestamp

## 3. API DESIGN

-   **`GET /personalization/preferences`**:
    -   **Method**: `GET`
    -   **Authentication**: Required (JWT token)
    -   **Request**: None
    -   **Response**:
        ```json
        {
          "skillLevel": "Intermediate",
          "interests": ["ROS2", "Computer Vision"],
          "learningStyle": "Practical"
        }
        ```
    -   **Errors**: `401 Unauthorized`, `404 Not Found`
-   **`POST /personalization/preferences`**:
    -   **Method**: `POST`
    -   **Authentication**: Required (JWT token)
    -   **Request**:
        ```json
        {
          "skillLevel": "Expert",
          "interests": ["ROS2", "Control Systems"],
          "learningStyle": "Theoretical"
        }
        ```
    -   **Response**: `200 OK`
    -   **Errors**: `400 Bad Request`, `401 Unauthorized`
-   **`POST /personalization/track`**:
    -   **Method**: `POST`
    -   **Authentication**: Required (JWT token)
    -   **Request**:
        ```json
        {
          "contentId": "ros2-architecture",
          "engagementScore": 300
        }
        ```
    -   **Response**: `202 Accepted`
    -   **Errors**: `400 Bad Request`, `401 Unauthorized`
-   **`GET /personalization/recommendations`**:
    -   **Method**: `GET`
    -   **Authentication**: Required (JWT token)
    -   **Request**: None
    -   **Response**:
        ```json
        {
          "recommendations": [
            { "contentId": "ros2-nodes", "title": "Understanding ROS 2 Nodes" },
            { "contentId": "ros2-topics", "title": "ROS 2 Topics for Communication" }
          ]
        }
        ```
    -   **Errors**: `401 Unauthorized`

## 4. PERSONALIZATION ENGINE LOGIC

-   **Preference Extraction**: The system will retrieve user preferences directly from the `UserPreference` table.
-   **History Tracking**: The `/personalization/track` endpoint will be called from the frontend to log user interactions.
-   **Content Adaptation**:
    -   The frontend will fetch user preferences and use client-side logic to adapt the UI (e.g., collapsing sections, showing tooltips).
    -   For the chatbot, the frontend will prepend a context to the user's prompt, as described in the architecture.
-   **Recommendation Scoring**:
    -   A simple content-based filtering approach will be used initially.
    -   Recommendations will be based on the user's declared interests and reading history.
    -   A score will be calculated for unread content based on its similarity to the user's interests and recently read content.

## 5. FRONTEND IMPLEMENTATION PLAN

-   **UI Screens**:
    -   A new "Personalization Settings" page will be created.
-   **Components**:
    -   `PreferenceForm.js`: A form for setting user preferences.
    -   `RecommendationPanel.js`: A component to display recommended content.
    -   `withPersonalization.js`: A Higher-Order Component (HOC) to wrap existing content pages and apply personalization logic.
-   **State Management**:
    -   User preferences will be fetched on login and stored in a React Context or a state management library like Zustand.
-   **Data Fetching**:
    -   The `fetch` API will be used to interact with the Personalization API.

## 6. BACKEND IMPLEMENTATION PLAN

-   **Framework**: FastAPI
-   **Controllers**:
    -   `personalization.py`: Will contain the API endpoints.
-   **Services**:
    -   `preference_service.py`: Business logic for managing preferences.
    -   `tracking_service.py`: Business logic for tracking user history.
    -   `recommendation_service.py`: Business logic for generating recommendations.
-   **Database Handlers**:
    -   `database.py`: Will manage the database connection and session.
    -   `models.py`: Will contain the SQLAlchemy models for `UserPreference` and `ReadingHistory`.
-   **Authentication**:
    -   A middleware will be used to validate the JWT token from the `003-auth-system`.

## 7. FULL FOLDER AND FILE STRUCTURE

```
004-content-personalization/
├── backend/
│   ├── app/
│   │   ├── __init__.py
│   │   ├── main.py
│   │   ├── controllers/
│   │   │   ├── __init__.py
│   │   │   └── personalization.py
│   │   ├── services/
│   │   │   ├── __init__.py
│   │   │   ├── preference_service.py
│   │   │   ├── tracking_service.py
│   │   │   └── recommendation_service.py
│   │   └── models/
│   │       ├── __init__.py
│   │       └── database.py
│   ├── requirements.txt
│   └── Dockerfile
└── frontend/
    ├── components/
    │   ├── PreferenceForm.js
    │   └── RecommendationPanel.js
    └── hocs/
        └── withPersonalization.js
```

## 8. STEP-BY-STEP EXECUTION SEQUENCE

1.  **Backend Setup**: Initialize the FastAPI project, set up the database connection, and create the SQLAlchemy models.
2.  **Authentication Middleware**: Implement the JWT authentication middleware to secure the backend endpoints.
3.  **Preferences API**: Implement the `GET` and `POST` endpoints for managing user preferences.
4.  **Tracking API**: Implement the `POST` endpoint for tracking user history.
5.  **Frontend Settings Page**: Create the "Personalization Settings" page with the `PreferenceForm` component.
6.  **Frontend Integration**: Integrate the settings page with the backend API.
7.  **Recommendation Engine**: Implement the recommendation service on the backend.
8.  **Recommendation API**: Implement the `GET` endpoint for retrieving recommendations.
9.  **Frontend Recommendation UI**: Create the `RecommendationPanel` component and display the recommendations.
10. **Content Adaptation**: Implement the `withPersonalization` HOC to adapt the content based on user preferences.

## 9. CRITICAL RULES

-   Do NOT modify any code in `001`, `002`, or `003`.
-   All new code must live ONLY inside `004-content-personalization`.
-   The plan must be followed exactly as written.
-   No implementation in this step — only planning.
-   Wait for the `/sp.tasks` command after this.