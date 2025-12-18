# Tasks: Content Personalization

**Input**: Design documents from `/specs/004-content-personalization/`

## Phase 1: Setup (Shared Infrastructure)

- [X] T001 [P] Create the backend folder structure in `004-content-personalization/backend/app` as defined in `plan.md`.
- [X] T002 [P] Create the frontend folder structure in `004-content-personalization/frontend` as defined in `plan.md`.
- [X] T003 Initialize the FastAPI project in `004-content-personalization/backend` and create `requirements.txt`.
- [X] T004 Set up environment variable templates (e.g., `.env.example`) for the backend service.

## Phase 2: Foundational (Blocking Prerequisites)

- [X] T005 [P] Implement the database models for `UserPreference` and `ReadingHistory` in `004-content-personalization/backend/app/models/database.py`.
- [X] T006 [P] Create database migration scripts.
- [X] T007 Implement the JWT authentication middleware in the backend to validate tokens from `003-auth-system`.
- [X] T008 [P] Set up the database connection and session management in `004-content-personalization/backend/app/models/database.py`.

## Phase 3: User Story 1 - Setting and Managing Preferences (P1) 🎯 MVP

**Goal**: Allow users to set and update their learning preferences.
**Independent Test**: A user can log in, navigate to a settings page, set their preferences, save them, and see them reflected in their profile.

- [X] T009 [US1] Implement the `Preference` service in `004-content-personalization/backend/app/services/preference_service.py` to handle business logic for preferences.
- [X] T010 [US1] Implement the `GET /personalization/preferences` endpoint in `004-content-personalization/backend/app/controllers/personalization.py`.
- [X] T011 [US1] Implement the `POST /personalization/preferences` endpoint in `004-content-personalization/backend/app/controllers/personalization.py`.
- [X] T012 [P] [US1] Create the `PreferenceForm.js` component in `004-content-personalization/frontend/components/PreferenceForm.js`.
- [X] T013 [US1] Create the "Personalization Settings" page and integrate the `PreferenceForm.js` component.
- [X] T014 [US1] Implement frontend logic to fetch and save user preferences.

## Phase 4: User Story 2 - Content Adaptation Based on Profile (P2)

**Goal**: Adapt book content and chatbot answers to the user's profile.
**Independent Test**: Two users with different skill levels see different views of the same content or receive different chatbot answers.

- [X] T015 [US2] Implement the `withPersonalization.js` HOC in `004-content-personalization/frontend/hocs/withPersonalization.js` to wrap content pages.
- [X] T016 [US2] Implement client-side logic in the `withPersonalization.js` HOC to adapt content based on user preferences (e.g., collapsing sections).
- [X] T017 [US2] Implement client-side logic to enrich the RAG chatbot prompt with user context before sending it to the chatbot API.
- [X] T018 [US2] Implement the tracking service in `004-content-personalization/backend/app/services/tracking_service.py`.
- [X] T019 [US2] Implement the `POST /personalization/track` endpoint in `004-content-personalization/backend/app/controllers/personalization.py`.
- [X] T020 [P] [US2] Implement frontend hooks to track user reading history and call the `/personalization/track` endpoint.

## Phase 5: User Story 3 - Personalized Content Recommendations (P2)

**Goal**: Provide users with personalized recommendations for what to read next.
**Independent Test**: A user sees a "Recommended for you" section with relevant suggestions based on their reading history.

- [X] T021 [US3] Implement the recommendation engine logic in `004-content-personalization/backend/app/services/recommendation_service.py`.
- [X] T022 [US3] Implement the `GET /personalization/recommendations` endpoint in `004-content-personalization/backend/app/controllers/personalization.py`.
- [X] T023 [P] [US3] Create the `RecommendationPanel.js` component in `004-content-personalization/frontend/components/RecommendationPanel.js`.
- [X] T024 [US3] Integrate the `RecommendationPanel.js` component into the UI to display recommendations.

## Phase 6: Polish & Cross-Cutting Concerns

- [X] T025 [P] Add request validation to all backend endpoints.
- [X] T026 [P] Add error handling and logging to the backend service.
- [X] T027 [P] Ensure the backend is ready for deployment (e.g., Dockerfile is complete and working).
- [X] T028 Perform end-to-end testing of the entire personalization flow.
- [X] T029 Review and validate that all rules and constraints from the spec and plan have been met.
