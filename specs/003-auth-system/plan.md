# Implementation Plan: Authentication System

**Branch**: `003-auth-system` | **Date**: 2025-12-09 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/003-auth-system/spec.md`

**Note**: This template is filled in by the `/sp.plan` command. See `.specify/templates/commands/plan.md` for the execution workflow.

## Summary

This plan outlines the integration of a full-featured authentication system into the existing Docusaurus application. The system will be built using a FastAPI backend with `better-auth` for user management and a Neon Serverless Postgres database for data storage. The frontend will consist of new React components for signup and signin, integrated globally. The goal is to provide a complete, secure, and isolated authentication and user profile management feature.

## Technical Context

**Language/Version**: Python 3.10+, TypeScript/JavaScript (React)
**Primary Dependencies**: FastAPI, `better-auth`, `psycopg2-binary`, React, Docusaurus
**Storage**: Neon Serverless Postgres
**Testing**: `pytest` (backend), `Jest`/`React Testing Library` (frontend)
**Target Platform**: HuggingFace Spaces (backend), Vercel (frontend)
**Project Type**: Web application (backend + frontend)
**Performance Goals**: p99 latency for all auth API endpoints < 500ms.
**Constraints**: Must not break Docusaurus static site generation (`npm run build`).
**Scale/Scope**: Designed for an initial scale of up to 10,000 users.

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

The plan adheres to all principles outlined in the project constitution, version 1.2.0.

- **Principle VII (User Authentication System)**: The plan directly implements the requirements of this principle, including the specified technology stack (FastAPI, Neon, `better-auth`), strict file scoping, and user data collection.
- **Strict Scoping**: The plan explicitly defines file paths for frontend (`Physical-AI-and-Humanoid-Robotics-Book/src/auth/`) and backend (`backend/auth-server/`), respecting the constitutional requirement to avoid modifying other features.
- **Reproducibility**: The plan emphasizes environment-based configuration (`quickstart.md`) to ensure developers can reproduce the setup easily.

**Result**: ✅ PASS

## Project Structure

### Documentation (this feature)

```text
specs/003-auth-system/
├── plan.md              # This file
├── research.md          # Phase 0 output
├── data-model.md        # Phase 1 output
├── quickstart.md        # Phase 1 output
├── contracts/           # Phase 1 output
│   └── openapi.yml
└── tasks.md             # Phase 2 output (NOT created by this command)
```

### Source Code (repository root)

```text
# Web application (frontend + backend)
backend/
└── auth-server/         # New backend service for this feature
    ├── app/
    │   ├── main.py
    │   ├── api/
    │   ├── models/
    │   └── services/
    ├── requirements.txt
    └── .env.example

Physical-AI-and-Humanoid-Robotics-Book/
└── src/
    ├── auth/            # New frontend components for this feature
    │   ├── components/
    │   │   ├── SignupModal.tsx
    │   │   └── SigninModal.tsx
    │   ├── services/
    │   └── context/
    │       └── AuthProvider.tsx
    └── theme/
        └── Root.js      # Entry point for global provider injection
```

**Structure Decision**: The plan adopts a standard web application structure with a distinct backend service and dedicated frontend components, fully respecting the strict file-scope requirements from the constitution and the feature specification.

## Implementation Phases

### Phase 1 — Frontend Setup
1. Create folder: `Physical-AI-and-Humanoid-Robotics-Book/src/auth/`
2. Create `SignupModal` + `SigninModal` components.
3. Add user background form fields (Full Name, Email, Skill Level, Hardware, Robotics Experience, OS, Learning Mode).
4. Connect to `better-auth` client SDK.
5. Add navbar auth button and persist "Signed in as <name>" (store session in `localStorage`/`sessionStorage`).

### Phase 2 — Backend Setup
1. Create folder: `backend/auth-server/`.
2. Setup `better-auth` server SDK (FastAPI).
3. Connect to Neon Serverless Postgres.
4. Create DB schema for users + profiles.
5. Implement `/auth` routes: `signup`, `signin`, `signout`, `me`, `profile-update`.
6. Add robust input validation, error handling, and logging.

### Phase 3 — Integration
1. Add global `AuthProvider` in Docusaurus via `src/theme/Root.js`.
2. Sync session to frontend and expose auth context.
3. Expose user profile to the existing RAG chatbot through a read-only integration point (do NOT modify chatbot internals).

### Phase 4 — Deployment
1. Deploy backend to HuggingFace Spaces.
2. Ensure `requirements.txt`, `app.py`, and Neon connection environment variables are present in the Space.
3. Configure CORS to allow requests from the Vercel-deployed frontend.
4. Deploy frontend and validate static export does not break.
5. Run end-to-end tests (signup, signin, profile-save, navbar display).

## Complexity Tracking

No constitution violations detected. This section is not required.