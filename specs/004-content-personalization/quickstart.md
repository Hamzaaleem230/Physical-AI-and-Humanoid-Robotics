# Quickstart: Content Personalization Feature

**Feature**: `004-content-personalization`  
**Created**: 2025-12-17
**Source**: `specs/004-content-personalization/spec.md`, `specs/004-content-personalization/plan.md`

## Overview

This guide provides a quick overview of how to set up and interact with the Content Personalization feature. It covers setting up the backend service, understanding the frontend integration points, and basic API interactions.

## 1. Prerequisites

Before you begin, ensure you have:

- Python 3.9+
- `pip` (Python package installer)
- `Docker` (for running the backend service)
- `Node.js` (v18 or higher) and `npm` (for frontend development)
- Access to a Neon Serverless Postgres database instance (or a compatible PostgreSQL database).
- A running instance of the `003-auth-system` to provide JWT tokens for authentication.

## 2. Backend Setup

The personalization backend is a FastAPI application.

### a. Clone the repository

First, ensure you have the full project repository cloned. The `004-content-personalization` directory is part of this.

### b. Environment Variables

Create a `.env` file in the `004-content-personalization/backend` directory based on `.env.example`. This file should include:

- `DATABASE_URL`: Connection string for your PostgreSQL database.
- `AUTH_SERVICE_URL`: URL of the `003-auth-system` for JWT validation.
- `SECRET_KEY`: A strong secret key for your FastAPI application.

### c. Run with Docker (Recommended)

Navigate to the `004-content-personalization/backend` directory and build/run the Docker container:

```bash
cd 004-content-personalization/backend
docker build -t personalization-backend .
docker run -p 8000:8000 personalization-backend
```

The backend API will be available at `http://localhost:8000`.

### d. Run Locally (Python)

If you prefer to run the backend directly:

```bash
cd 004-content-personalization/backend
pip install -r requirements.txt
python -m uvicorn app.main:app --host 0.0.0.0 --port 8000
```

## 3. Database Setup

Ensure your PostgreSQL database is running and accessible via the `DATABASE_URL` provided in your `.env` file. The backend service will handle the creation of `UserPreference` and `ReadingHistory` tables on startup if they don't exist (via SQLAlchemy models).

## 4. Frontend Integration

The frontend components for personalization are designed to be integrated into the existing Docusaurus book (`Physical-AI-and-Humanoid-Robotics-Book`).

### a. Preference Settings Page

- **Location**: The `PreferenceForm.js` component (from `004-content-personalization/frontend/components/PreferenceForm.js`) should be integrated into a new or existing user profile/settings page within the Docusaurus application.
- **Data Flow**: On component mount, fetch user preferences from `/personalization/preferences`. On form submission, `POST` updated preferences to `/personalization/preferences`.

### b. Content Adaptation

- The `withPersonalization.js` Higher-Order Component (HOC) (from `004-content-personalization/frontend/hocs/withPersonalization.js`) is intended to wrap Docusaurus content pages.
- **Usage**: Import and wrap your Docusaurus page components:
    ```javascript
    import withPersonalization from '@site/src/hocs/withPersonalization'; // Adjust path
    
    function MyContentPage() {
      // ... page content
    }
    
    export default withPersonalization(MyContentPage);
    ```
- This HOC will fetch user preferences and apply client-side logic to adapt the content.

### c. RAG Chatbot Integration

- **Mechanism**: The frontend will be responsible for fetching user context from the personalization API and prepending it to user queries before sending them to the `002-rag-chatbot` API.
- **Example**: If a user is "Beginner" in "ROS2", a query "What are ROS2 nodes?" might be transformed to "Explain to a beginner in ROS2: What are ROS2 nodes?".

## 5. Basic API Interaction (Example with `curl`)

Ensure your backend service is running and you have a valid JWT token (from `003-auth-system`).

### a. Set Preferences

```bash
curl -X POST http://localhost:8000/personalization/preferences \
  -H "Authorization: Bearer <YOUR_JWT_TOKEN>" \
  -H "Content-Type: application/json" \
  -d 
  {
        "skillLevel": "Intermediate",
        "interests": ["ROS2", "Computer Vision"],
        "learningStyle": "Practical"
      }
```

### b. Get Preferences

```bash
curl -X GET http://localhost:8000/personalization/preferences \
  -H "Authorization: Bearer <YOUR_JWT_TOKEN>"
```

### c. Track Reading History

```bash
curl -X POST http://localhost:8000/personalization/track \
  -H "Authorization: Bearer <YOUR_JWT_TOKEN>" \
  -H "Content-Type: application/json" \
  -d 
  {
        "contentId": "ros2-architecture",
        "engagementScore": 120
      }
```

### d. Get Recommendations

```bash
curl -X GET http://localhost:8000/personalization/recommendations \
  -H "Authorization: Bearer <YOUR_JWT_TOKEN>"
```
