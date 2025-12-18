# Personalization API Contracts

**Feature**: `004-content-personalization`  
**Created**: 2025-12-17
**Source**: `specs/004-content-personalization/spec.md`, `specs/004-content-personalization/plan.md`

## Overview

This document outlines the API contracts for the Personalization Service. These APIs facilitate the management of user preferences, tracking of reading history, and generation of content recommendations. All endpoints require authentication via a JWT token issued by the `003-auth-system`.

## Endpoints

### 1. Retrieve User Preferences

-   **Route**: `/personalization/preferences`
-   **Method**: `GET`
-   **Description**: Retrieves the personalization preferences for the currently authenticated user.
-   **Authentication**: Required (JWT token in `Authorization` header).
-   **Request Body**: None.
-   **Response (200 OK)**:
    ```json
    {
      "skillLevel": "Intermediate",
      "interests": ["ROS2", "Computer Vision"],
      "learningStyle": "Practical"
    }
    ```
-   **Error Responses**:
    -   `401 Unauthorized`: If no valid JWT token is provided.
    -   `404 Not Found`: If preferences for the user do not exist.

### 2. Create or Update User Preferences

-   **Route**: `/personalization/preferences`
-   **Method**: `POST`
-   **Description**: Creates a new preference profile or updates an existing one for the currently authenticated user.
-   **Authentication**: Required (JWT token in `Authorization` header).
-   **Request Body**:
    ```json
    {
      "skillLevel": "Expert",
      "interests": ["ROS2", "Control Systems"],
      "learningStyle": "Theoretical"
    }
    ```
    -   **Validation Rules**:
        -   `skillLevel`: Must be one of 'Beginner', 'Intermediate', 'Expert'.
        -   `interests`: Must be an array of strings.
        -   `learningStyle`: Must be one of 'Theoretical', 'Practical', 'Balanced'.
-   **Response (200 OK)**:
    -   Indicates successful creation or update.
-   **Error Responses**:
    -   `400 Bad Request`: If the request body is invalid or missing required fields.
    -   `401 Unauthorized`: If no valid JWT token is provided.

### 3. Track User Interaction

-   **Route**: `/personalization/track`
-   **Method**: `POST`
-   **Description**: Records a user's interaction with a piece of content, such as viewing a page or chapter.
-   **Authentication**: Required (JWT token in `Authorization` header).
-   **Request Body**:
    ```json
    {
      "contentId": "ros2-architecture",
      "engagementScore": 300
    }
    ```
    -   **Validation Rules**:
        -   `contentId`: Must be a non-empty string.
        -   `engagementScore`: Must be a non-negative integer.
-   **Response (202 Accepted)**:
    -   Indicates that the interaction tracking request has been accepted for processing.
-   **Error Responses**:
    -   `400 Bad Request`: If the request body is invalid or missing required fields.
    -   `401 Unauthorized`: If no valid JWT token is provided.

### 4. Retrieve Personalized Content Recommendations

-   **Route**: `/personalization/recommendations`
-   **Method**: `GET`
-   **Description**: Provides a list of content recommendations tailored to the authenticated user's profile and reading history.
-   **Authentication**: Required (JWT token in `Authorization` header).
-   **Request Body**: None.
-   **Response (200 OK)**:
    ```json
    {
      "recommendations": [
        { "contentId": "ros2-nodes", "title": "Understanding ROS 2 Nodes" },
        { "contentId": "ros2-topics", "title": "ROS 2 Topics for Communication" }
      ]
    }
    ```
-   **Error Responses**:
    -   `401 Unauthorized`: If no valid JWT token is provided.
    -   `500 Internal Server Error`: If an issue occurs during recommendation generation.
