---
id: 0003
title: Amend Constitution for RAG Chatbot
stage: constitution
date: 2025-12-07
surface: agent
model: gemini-pro
feature: 002-rag-chatbot
branch: main
user: placeholder_user
command: /sp.constitution
labels: ["constitution", "chatbot", "rag"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - specs/002-rag-chatbot/
  - .specify/memory/constitution.md
tests:
  - none
---

## Prompt

/sp.constitution ur ONLY task is to integrate a complete RAG-based intelligent chatbot inside the existing Docusaurus book.

The chatbot must appear as a floating chat bubble/button at the bottom-right corner on every page.

Frontend Location:
- Integrate React chatbot UI inside “Physical-AI-and-Humanoid-Robotics-Book/src/components/chatbot/”
- Add global injection inside “Physical-AI-and-Humanoid-Robotics-Book/src/theme/Root.js” or via Layout wrapper.

Backend Requirements:
- Build a FastAPI backend
- Store user-selected and book-text embeddings inside Qdrant Cloud Free Tier
- Store user accounts (optional) inside Neon Serverless Postgres
- Use OpenAI or Gemini 2.0 Flash or Claude 3.7 for RAG inference
- Create endpoints:
  /embed
  /query
  /chat
  /health
  /ingest-book

RAG Features:
- Chatbot must answer only from book content
- Chatbot must support “answer from selected text” mode
- Chatbot must stream responses
- Chatbot must have memory inside session

Important Rules:
- Place ALL new specs inside a NEW directory:
  specs/002-rag-chatbot/
- Do NOT touch the previous book specs in specs/001-physical-ai-book/
- The existing book must remain intact.

This constitution governs ONLY the chatbot feature.

Follow every instruction with extreme attention to detail — perform every task with maximum depth, accuracy, and precision, leaving no ambiguity, no shortcuts, and no skipped steps. Take all the time required and complete all requirements exactly as stated, without altering, ignoring, or simplifying anything.

## Response snapshot

The project constitution was amended to include a new principle for a RAG-based chatbot.

- A new directory `specs/002-rag-chatbot/` was created.
- The constitution file at `.specify/memory/constitution.md` was updated to version `1.1.0`.
- A new principle, "VI. RAG Chatbot Integration," was added, detailing the frontend, backend, data, AI, and feature requirements as specified in the prompt.
- The language and structure of the existing constitution were maintained.

## Outcome

- ✅ Impact: A new guiding principle for the RAG chatbot feature was added to the project's constitution, ensuring future development aligns with the specified architecture and requirements.
- 🧪 Tests: No tests were run as this was a documentation change.
- 📁 Files: Created `specs/002-rag-chatbot/` and modified `.specify/memory/constitution.md`.
- 🔁 Next prompts: `/sp.spec` or `/sp.plan` for the `002-rag-chatbot` feature.
- 🧠 Reflection: The user's prompt combined a feature specification with a constitution update command. I correctly interpreted this as a request to add the feature's requirements as a new principle in the constitution file, following the established format.

## Evaluation notes (flywheel)

- Failure modes observed: None.
- Graders run and results (PASS/FAIL): PASS
- Prompt variant (if applicable): null
- Next experiment (smallest change to try): n/a