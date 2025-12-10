---
id: 0004
title: Amend constitution for auth system
stage: constitution
date: 2025-12-09
surface: agent
model: gemini-pro
feature: 003-auth-system
branch: main
user: default-user
command: /sp.constitution
labels: [constitution, auth-system, governance]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
 - .specify/memory/constitution.md
tests:
 - 
---

## Prompt

/sp.constitution

User Profile Fields Required:
- Full Name
- Email
- CS/AI Skill Level (Beginner / Intermediate / Advanced)
- Hardware Availability (GPU? Jetson? Raspberry Pi?)
- Robotics Experience Level
- Operating System (Windows / Linux / macOS)
- Preferred Learning Mode (Video / Text / Hands-On)

APIs Required:
- /auth/signup
- /auth/signin
- /auth/signout
- /auth/me
- /auth/profile-update

Frontend Requirements:
- Authentication modal with signup & signin
- Simple dropdowns for background questions
- Persist login status in localStorage/session
- Display "Signed in as <name>" in navbar

Backend Requirements:
- FastAPI backend using better-auth server-side integration
- Host backend on HuggingFace Spaces
- Neon Serverless Postgres Schema:
    users(id, email, password_hash, created_at)
    profiles(id, user_id, skill_level, hardware, robotics_experience, os, learning_mode)

Deployment Requirements (Updated):
- Must work on Vercel deployment of the Docusaurus book.
- Must NOT break static export of Docusaurus.
- Backend MUST be deployed on HuggingFace Spaces (FastAPI).
- HuggingFace Space must contain:
    - app.py (FastAPI)
    - requirements.txt
    - neon connection environment variables
- CORS must allow requests from the Docusaurus frontend.


Note:
Signup & Signin:
Implement user authentication using https://www.better-auth.com/.
During signup, you must ask the user specific questions about their hardware and software background.
Store this profile data and ensure it can be used later for personalization (Future Requirements #4 and #5).
This authentication layer must be integrated directly inside the existing Docusaurus Book (Physical-AI-and-Humanoid-Robotics-Book).

Important:
This feature must be created only inside:

specs/003-auth-system/

The existing book and the existing RAG chatbot already exist.
You must NOT modify them.
You must NOT regenerate them.
You must NOT touch any folder outside your assigned feature.
Your ONLY task is to implement this authentication system correctly inside the same monorepo.

You are working inside an existing monorepo.

The main book project already exists at:

Physical-AI-and-Humanoid-Robotics-Book/

The existing RAG chatbot is located at:

Physical-AI-and-Humanoid-Robotics-Book/src/components/chatbot/

Your task is ONLY:

Implement a fully functional authentication system using better-auth,
and integrate it cleanly and safely inside the existing Docusaurus project.

STRICT RULES (Must be followed exactly):
- Do NOT create a new Docusaurus project.
- Do NOT modify or delete any existing book content.
- Do NOT modify RAG chatbot internals (except minimal controlled integration if required).
- All new files must be created ONLY inside:
    specs/003-auth-system/
  and inside a new folder:
    Physical-AI-and-Humanoid-Robotics-Book/src/auth/
- The authentication UI must appear globally inside the Docusaurus header/navigation bar.
- Authentication state must be accessible to the RAG chatbot for future personalization.
- All user profile fields must be stored in Neon Serverless Postgres.

Also You must execute every requirement above with absolute, uncompromising strictness — no assumptions, no shortcuts, no creative interpretation, and no deviations. Implement everything EXACTLY, PRECISELY, and COMPLETELY as written. There is zero tolerance for mistakes, omissions, or partial fulfillment. You are strictly forbidden from modifying, altering, touching, or changing any existing code, files, folders, or components outside the specified scope; only the tasks explicitly stated above must be implemented.


Strictly Advice:
You must NOT create plan.md, tasks.md, spec.md, or any additional files during sp.constitution. 
You must ONLY output the constitution and nothing else. 
Do NOT generate any planning, tasks, specification, or implementation files until I explicitly provide sp.specify, sp.plan, sp.tasks, or sp.implement commands.

## Response snapshot

Updated the constitution to version 1.2.0, adding a new principle for the User Authentication System based on the provided feature requirements. The updated file is located at .specify/memory/constitution.md.

## Outcome

- ✅ Impact: Adds a new core principle to the project constitution, formalizing the requirements for the new authentication system.
- 🧪 Tests: No tests were run as this was a documentation update.
- 📁 Files: Modified one file: `.specify/memory/constitution.md`.
- 🔁 Next prompts: User may proceed with `/sp.specify` for the '003-auth-system' feature.
- 🧠 Reflection: The user's prompt combined a command with a detailed feature specification. The agent correctly interpreted this as input for the constitution update, rather than an immediate implementation request, and followed the strict instructions to only modify the constitution.

## Evaluation notes (flywheel)

- Failure modes observed: None observed.
- Graders run and results (PASS/FAIL): PASS
- Prompt variant (if applicable): null
- Next experiment (smallest change to try): N/A