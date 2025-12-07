---
id: 0002
title: Create Technical Plan for RAG Chatbot
stage: plan
date: 2025-12-07
surface: agent
model: gemini-pro
feature: 002-rag-chatbot
branch: 002-rag-chatbot
user: placeholder_user
command: /sp.plan
labels: ["plan", "chatbot", "rag", "architecture"]
links:
  spec: specs/002-rag-chatbot/spec.md
  plan: specs/002-rag-chatbot/plan.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/002-rag-chatbot/plan.md
  - specs/002-rag-chatbot/research.md
  - specs/002-rag-chatbot/data-model.md
  - specs/002-rag-chatbot/contracts/openapi.yaml
  - specs/002-rag-chatbot/quickstart.md
  - GEMINI.md
tests:
  - none
---

## Prompt

/sp.plan Plan Directory: specs/002-rag-chatbot/

Project: Integrate RAG Chatbot into existing Docusaurus book located at:
Physical-AI-and-Humanoid-Robotics-Book/

DO NOT create a new Docusaurus project.

Phase 1 — Frontend Setup
1. Create folder:
   Physical-AI-and-Humanoid-Robotics-Book/src/components/chatbot/
2. Build floating chat widget (React)
3. Add global injection in:
   Physical-AI-and-Humanoid-Robotics-Book/src/theme/Root.js
4. Add selected-text collection logic
5. Add streaming messages

Phase 2 — Backend Setup (FastAPI)
1. Create backend/ folder at repository root
2. Create FastAPI skeleton
3. Implement:
   - embed endpoint (OpenAI/Gemini/Claude)
   - ingest-book endpoint (parse /docs folder)
   - query endpoint (Qdrant search)
   - chat endpoint (LLM with RAG)
4. Add .env.template for keys
5. Add Qdrant client + embeddings pipeline
6. Add book ingestion script

Phase 3 — RAG Engine
1. Chunk markdown files
2. Clean markdown
3. Generate embeddings
4. Upsert into Qdrant collection "physical_ai_book"

Phase 4 — Integration
1. Connect frontend to backend
2. Test RAG with real book data
3. Implement fallback mode (if backend offline)

Phase 5 — Deployment
1. Deploy FastAPI backend to Render/Railway
2. Update Docusaurus config with backend URL
3. Deploy frontend with build → GitHub Pages or Vercel

Deliverables:
- Full RAG engine
- Bottom-right chatbot
- Book untouched
- All specs saved inside specs/002-rag-chatbot/

## Response snapshot

A comprehensive technical plan for the RAG chatbot was created.

1.  **Setup**: The planning phase was initiated, and the `plan.md` file was created from the template.
2.  **Planning**: The `plan.md` file was filled with a detailed technical context, project structure, and a constitutional compliance check.
3.  **Research & Design**: Key design artifacts were generated:
    -   `research.md`: Documented decisions on frontend integration (Docusaurus Swizzling) and backend communication (CORS).
    -   `data-model.md`: Defined the core data entities like `ContentChunk` and `ChatMessage`.
    -   `contracts/openapi.yaml`: Created an OpenAPI specification for the backend API.
    -   `quickstart.md`: Wrote a developer setup guide.
4.  **Agent Context**: The agent's context was updated with the new technologies being used.

## Outcome

- ✅ Impact: A full set of design and planning documents for the RAG chatbot feature now exists. This provides a clear and actionable blueprint for implementation.
- 🧪 Tests: No tests were run as this was a planning and design phase.
- 📁 Files: Created `plan.md`, `research.md`, `data-model.md`, `contracts/openapi.yaml`, and `quickstart.md` in the feature's spec directory.
- 🔁 Next prompts: `/sp.tasks` to break down the plan into concrete implementation tasks.
- 🧠 Reflection: The `/sp.plan` command involves multiple file creations. It's crucial to follow the prescribed workflow to ensure all artifacts are generated correctly and in the right location.

## Evaluation notes (flywheel)

- Failure modes observed: None.
- Graders run and results (PASS/FAIL): PASS
- Prompt variant (if applicable): null
- Next experiment (smallest change to try): n/a