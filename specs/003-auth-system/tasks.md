# Tasks: Authentication System

**Input**: Design documents from `/specs/003-auth-system/`
**Prerequisites**: plan.md (required), spec.md (required for user stories)

**Tests**: No explicit test tasks are generated as they were not requested in the spec. The implementation tasks should be validated against the acceptance criteria in `spec.md`.

## Format: `[ID] [P?] [Story] Description`
- **[P]**: Can run in parallel (different files, no dependencies).
- **[Story]**: Which user story this task belongs to (e.g., US1, US2).

---

## Phase 1: Project Setup & Configuration

**Purpose**: Create the initial directory structure and configuration files for both frontend and backend. This phase addresses User Story 3.

- [x] T001 Create backend directory `backend/auth-server/`.
- [x] T002 [P] [US3] Create backend environment file template `backend/auth-server/.env.example` based on `quickstart.md`.
- [x] T003 [P] Create frontend directory `Physical-AI-and-Humanoid-Robotics-Book/src/auth/`.
- [x] T004 [P] [US3] Add `REACT_APP_API_BASE_URL` to a new file `Physical-AI-and-Humanoid-Robotics-Book/.env.local.example` based on `quickstart.md`.

---

## Phase 2: Foundational Backend & DB

**Purpose**: Set up the core FastAPI application and database schema.

- [x] T005 Initialize the FastAPI application in `backend/auth-server/app/main.py`.
- [x] T006 [P] Create the `requirements.txt` file in `backend/auth-server/` with dependencies: `fastapi`, `uvicorn`, `python-dotenv`, `better-auth`, `psycopg2-binary`.
- [x] T007 [P] Create data models for `User` and `Profile` in `backend/auth-server/app/models.py` to represent the DB schema from `data-model.md`.
- [x] T008 Implement a database connection service in `backend/auth-server/app/database.py` to connect to the Neon Postgres DB using `DATABASE_URL` from the environment.
- [x] T009 Create a script or utility function to initialize the `users` and `profiles` tables in the database based on the schema in `data-model.md`.

---

## Phase 3: Foundational Frontend

**Purpose**: Set up the global authentication context provider in the Docusaurus app.

- [x] T010 [P] Create the directory structure for the auth context: `Physical-AI-and-Humanoid-Robotics-Book/src/auth/context/`.
- [x] T011 Create a placeholder `AuthProvider.tsx` in `Physical-AI-and-Humanoid-Robotics-Book/src/auth/context/`.
- [x] T012 Swizzle the Docusaurus `Root` component and wrap the application with the `AuthProvider` in `Physical-AI-and-Humanoid-Robotics-Book/src/theme/Root.js`.

---

## Phase 4: User Story 1 - Account Creation (Priority: P1) 🎯 MVP

**Goal**: A new user can create an account and their profile information is saved.
**Independent Test**: A user can successfully complete the signup form, see a success state, and verify that a new `users` and `profiles` record is created in the database.

- [x] T013 [P] [US1] Create the React component `SignupModal.tsx` in `Physical-AI-and-Humanoid-Robotics-Book/src/auth/components/`.
- [x] T014 [P] [US1] Add all required form fields to `SignupModal.tsx` (Full Name, Email, Password, Skill Level, etc.).
- [x] T015 [US1] Implement the `/auth/signup` endpoint in the FastAPI backend at `backend/auth-server/app/main.py`, using `better-auth` to handle user creation and hashing the password.
- [x] T016 [US1] In the `/auth/signup` endpoint, after creating the `users` record, create the corresponding `profiles` record with the additional form data.
- [x] T017 [US1] Create a frontend API service in `Physical-AI-and-Humanoid-Robotics-Book/src/auth/services/api.ts` to connect the `SignupModal.tsx` component to the `/auth/signup` backend endpoint.

---

## Phase 5: User Story 2 - Sign-in & Session Management (Priority: P1)

**Goal**: An existing user can sign in, and their session state is reflected globally.
**Independent Test**: A user can log in, see their name in the navbar, close the tab, reopen it, and remain logged in.

- [x] T018 [P] [US2] Create the React component `SigninModal.tsx` in `Physical-AI-and-Humanoid-Robotics-Book/src/auth/components/`.
- [x] T019 [P] [US2] Implement the `/auth/signin` endpoint in `backend/auth-server/app/main.py` using `better-auth`.
- [x] T020 [P] [US2] Implement the `/auth/me` endpoint in `backend/auth-server/app/main.py` to return the current authenticated user's profile data.
- [x] T021 [US2] Connect the `SigninModal.tsx` component to the `/auth/signin` endpoint via the frontend API service.
- [x] T022 [US2] In the global `AuthProvider.tsx`, implement logic to fetch the current user via the `/auth/me` endpoint and store the user state (name, login status).
- [x] T023 [US2] Create a component in `Physical-AI-and-Humanoid-Robotics-Book/src/auth/components/` that displays the "Sign In" button or "Signed in as <name>" based on the auth context.
- [x] T024 [US2] Integrate the component from T023 into the Docusaurus navbar.

---

## Phase 6: Polish & Deployment

**Purpose**: Finalize remaining endpoints, prepare for production, and deploy.

- [x] T025 [P] Implement the `/auth/signout` endpoint in `backend/auth-server/app/main.py`.
- [x] T026 [P] Implement the `/auth/profile-update` endpoint in `backend/auth-server/app/main.py`.
- [x] T027 [P] Connect the frontend to the `/auth/signout` endpoint.
- [x] T028 Add a read-only integration point to expose the auth context to the RAG chatbot components.
- [x] T029 Prepare the backend for deployment on HuggingFace Spaces, ensuring `app.py` and `requirements.txt` are correct.
- [x] T030 Configure CORS in the FastAPI backend to allow requests from the production Vercel domain.
- [x] T031 Deploy the backend to HuggingFace Spaces and set environment variables.
- [x] T032 Deploy the Docusaurus frontend to Vercel and validate the build and auth functionality.
- [x] T033 Perform end-to-end testing of the full signup and signin flows on the deployed production environment.

---

## Dependencies & Execution Order

- **Setup (Phase 1)** must be complete before any other phase.
- **Foundational Backend & DB (Phase 2)** and **Foundational Frontend (Phase 3)** can run in parallel. Both must be complete before user story phases.
- **User Story 1 (Phase 4)** and **User Story 2 (Phase 5)** can begin after the foundational phases are complete. They can be developed in parallel.
- **Polish & Deployment (Phase 6)** depends on the completion of the primary user stories (Phase 4 and 5).
