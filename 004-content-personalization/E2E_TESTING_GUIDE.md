# End-to-End Testing Guide for Content Personalization Feature

This guide outlines the steps to perform end-to-end testing for the Content Personalization feature. This involves running the backend service, a Docusaurus frontend application (where the personalization components are integrated), and interacting with them to verify the personalization flow.

## 1. Prerequisites

-   A running instance of the `003-auth-system` (or a way to obtain a valid JWT token).
-   A PostgreSQL database accessible via the `DATABASE_URL`.
-   Docker installed and running (recommended for backend).
-   Node.js and npm installed (for frontend).
-   The Docusaurus book project (`Physical-AI-and-Humanoid-Robotics-Book`) where the frontend components are integrated.

## 2. Backend Setup & Start

1.  **Navigate to the backend directory**:
    ```bash
    cd 004-content-personalization/backend
    ```
2.  **Create `.env` file**: Based on `.env.example`, create a `.env` file and fill in the required values:
    -   `DATABASE_URL=postgres://user:password@host:port/database_name`
    -   `AUTH_SERVICE_URL=http://localhost:8001` (or your actual auth service URL)
    -   `SECRET_KEY=your_super_secret_key`
3.  **Build and run the Docker container**:
    ```bash
    docker build -t personalization-backend .
    docker run -p 8000:8000 --env-file .env personalization-backend
    ```
    Alternatively, to run locally with Python:
    ```bash
    pip install -r requirements.txt
    uvicorn app.main:app --host 0.0.0.0 --port 8000
    ```
    Verify the backend is running by navigating to `http://localhost:8000` in your browser. You should see `{"message": "Content Personalization Service is running!"}`.

## 3. Frontend Setup & Start (Docusaurus Integration)

This assumes you have integrated the personalization frontend components into your Docusaurus application as described in the respective `.js` files.

1.  **Integrate Frontend Components**:
    -   Copy `004-content-personalization/frontend/pages/PersonalizationSettingsPage.js` to `Physical-AI-and-Humanoid-Robotics-Book/src/pages/PersonalizationSettingsPage.js`.
    -   Copy `004-content-personalization/frontend/pages/RecommendationsPage.js` to `Physical-AI-and-Humanoid-Robotics-Book/src/pages/RecommendationsPage.js`.
    -   (Optional) Integrate `<RecommendationPanel />` directly into existing content pages as desired.
    -   (Optional) Update `sidebars.js` or `docusaurus.config.mjs` in the Docusaurus project to link to these new pages.

2.  **Navigate to the Docusaurus book directory**:
    ```bash
    cd Physical-AI-and-Humanoid-Robotics-Book
    ```
3.  **Install dependencies and start the Docusaurus app**:
    ```bash
    npm install
    npm start
    ```
    The Docusaurus application should now be running, typically at `http://localhost:3000`.

## 4. End-to-End Test Scenarios

These scenarios require a valid JWT token. You can obtain one by logging in through the `003-auth-system`'s frontend or by directly interacting with its login API.

### Scenario 1: Set and Retrieve User Preferences

1.  **Login**: Ensure you are logged into the Docusaurus application (via `003-auth-system`) to obtain a JWT token.
2.  **Navigate to Personalization Settings**: Go to the "Personalization Settings" page (e.g., `http://localhost:3000/personalization-settings`).
3.  **Set Preferences**:
    -   Select a "Skill Level" (e.g., "Beginner").
    -   Choose some "Interests" (e.g., "ROS2", "Computer Vision").
    -   Select a "Learning Style" (e.g., "Practical").
    -   Click "Save Preferences".
    -   **Expected**: A success message (from the mock currently) or the form fields should reflect the saved preferences after a refresh (if re-fetched).
4.  **Verify Preferences (Refresh)**: Refresh the page or navigate away and back.
    -   **Expected**: The form should load with the previously saved preferences.
5.  **Verify Preferences (Direct API Call - Optional)**:
    ```bash
    curl -X GET http://localhost:8000/personalization/preferences \
      -H "Authorization: Bearer <YOUR_JWT_TOKEN>"
    ```
    -   **Expected**: The JSON response should match the preferences you set.

### Scenario 2: Track Reading History

1.  **Login**: Ensure you are logged into the Docusaurus application.
2.  **Navigate to a Content Page**: Go to any documentation page (e.g., `http://localhost:3000/docs/ros2-architecture`).
3.  **Stay on the page**: Remain on the page for at least 30-60 seconds (the `useReadingHistoryTracker` sends data periodically).
4.  **Simulate leaving the page**: Close the tab or navigate to another page. The tracker should send final data on unmount or visibility change.
5.  **Verify Tracking (Direct API Call - Manual Database Check)**:
    -   Since there's no direct API to retrieve reading history, you would need to inspect the `reading_history` table in your PostgreSQL database to confirm entries for your `userId` and `contentId`.
    -   **Expected**: Entries should show `contentId` and `engagementScore` corresponding to your activity.

### Scenario 3: Receive Personalized Recommendations

1.  **Login**: Ensure you are logged into the Docusaurus application.
2.  **Ensure Preferences are Set**: Complete Scenario 1 to set your preferences.
3.  **Navigate to Recommendations Page**: Go to the "Recommendations" page (e.g., `http://localhost:3000/recommendations`).
4.  **Expected**: You should see a list of recommendations in the `RecommendationPanel`. The recommendations should ideally reflect the interests and skill level set in your preferences.
    -   For example, if your `skillLevel` is 'Beginner' and `interests` include 'ROS2', you might see "ROS 2 Architecture Overview" or "Understanding ROS 2 Nodes".

## 5. Troubleshooting

-   **Authentication Issues**:
    -   Ensure your JWT token is valid and correctly passed in the `Authorization` header.
    -   Check that `AUTH_SERVICE_URL` in your backend `.env` is correct and the `003-auth-system` is running.
-   **Database Connection Issues**:
    -   Verify `DATABASE_URL` in your backend `.env` file is correct and the PostgreSQL database is accessible.
    -   Check backend logs for database connection errors.
-   **Frontend API Calls Failing**:
    -   Ensure the backend service is running and accessible at `http://localhost:8000`.
    -   Check browser console for network errors.
-   **Recommendations Not Appearing/Relevant**:
    -   Ensure your preferences are saved correctly (Scenario 1).
    -   Check backend logs for `RecommendationService` errors.
    -   Verify the dummy content data in `RecommendationService` includes tags that match your preferences.
