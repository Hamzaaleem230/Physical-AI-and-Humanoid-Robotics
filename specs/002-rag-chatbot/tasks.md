# Tasks: RAG Chatbot Integration

**Input**: Design documents from `specs/002-rag-chatbot/`
**Prerequisites**: `plan.md`, `spec.md`

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Create the basic directory structure for the backend and frontend components.

- [X] T001 Create backend directory at repository root: `backend/`
- [X] T002 Create frontend chatbot directory in the Docusaurus project: `Physical-AI-and-Humanoid-Robotics-Book/src/components/chatbot/`

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST be complete before ANY user story can be implemented.

- [X] T003 [P] Initialize a new Python project in `backend/` (e.g., with Poetry or a simple `venv` and `requirements.txt`).
- [X] T004 [P] Set up a basic FastAPI application skeleton in `backend/app/main.py`.
- [X] T005 [P] Create a `.env.template` file in `backend/` for required API keys and environment variables (QDRANT_API_KEY, LLM_API_KEY, etc.).
- [X] T006 Configure the Docusaurus frontend to inject a placeholder component from the `chatbot/` directory into the main layout, ensuring the injection mechanism works. Modify `Physical-AI-and-Humanoid-Robotics-Book/src/theme/Root.js`.

---

## Phase 3: User Story 1 - View and Open Chatbot (Priority: P1) 🎯 MVP

**Goal**: A user can see the chat widget on all pages and open it.

**Independent Test**: Navigate to any page in the deployed Docusaurus book and confirm that a chat button appears and that clicking it opens an empty chat window.

- [X] T007 [P] [US1] Implement the floating chat button UI component in `Physical-AI-and-Humanoid-Robotics-Book/src/components/chatbot/ChatButton.js`.
- [X] T008 [P] [US1] Implement the main chat window UI component (as an empty shell) in `Physical-AI-and-Humanoid-Robotics-Book/src/components/chatbot/ChatWindow.js`.
- [X] T009 [US1] Implement the open/close state logic for the chat widget, connecting the `ChatButton` to the `ChatWindow`.

---

## Phase 4: User Story 2 - Ask General Questions (Priority: P1)

**Goal**: A user can ask a question and get a streamed answer from the RAG pipeline.

**Independent Test**: Open the chatbot, ask a question about the book's content, and verify that a streamed, accurate answer appears in the chat window.

- [X] T010 [P] [US2] Implement the message list and text input components in the chat window UI.
- [X] T011 [P] [US2] Implement frontend logic to handle streaming responses and display them in the message list.
- [X] T012 [P] [US2] Implement the Qdrant client and connection logic in the backend: `backend/app/services/vector_store.py`.
- [X] T013 [P] [US2] Implement the core RAG search logic (query embedding, Qdrant search, context retrieval) in `backend/app/services/rag_pipeline.py`.
- [X] T014 [US2] Implement the `/chat` API endpoint in the backend to orchestrate the RAG pipeline and stream responses back to the client.
- [X] T015 [US2] Connect the frontend input to the `/chat` backend endpoint.
- [X] T016 [P] [US2] Implement basic session memory on the client-side to keep track of the current conversation history.

---

## Phase 5: User Story 3 - Ask About Selected Text (Priority: P2)

**Goal**: A user can highlight text on the page and ask the chatbot to explain it.

**Independent Test**: Highlight a paragraph, trigger the "explain" action, and verify the chatbot's answer is based only on the selected text.

- [X] T017 [P] [US3] Implement JavaScript logic to detect and capture user-highlighted text on the page.
- [X] T018 [P] [US3] Add a UI element (e.g., a small button that appears near highlighted text) to trigger the "explain selected text" feature.
- [X] T019 [US3] Modify the frontend to send the selected text along with the query to the backend.
- [X] T020 [US3] Modify the `/chat` endpoint or create a new one to handle queries that are scoped to a specific piece of context provided in the request.

---

## Phase 6: User Story 4 - Content Ingestion (Priority: P2)

**Goal**: An administrator can trigger the re-ingestion of the book's content.

**Independent Test**: Add a new markdown file to the book's `/docs` directory, run the ingestion process, and then confirm that the chatbot can answer questions about the new content.

- [X] T021 [P] [US4] Create a script to parse all `.md` and `.mdx` files from the `Physical-AI-and-Humanoid-Robotics-Book/docs/` directory: `backend/scripts/parse_docs.py`.
- [X] T022 [P] [US4] Implement content chunking logic within the parsing script.
- [X] T023 [P] [US4] Implement the embedding generation pipeline for the text chunks: `backend/app/services/embedding_service.py`.
- [X] T024 [US4] Implement the logic to upsert the embedded chunks into the Qdrant collection: `backend/scripts/upload_to_qdrant.py`.
- [X] T025 [US4] Implement the `/ingest-book` endpoint in the backend to orchestrate the full ingestion pipeline (parse -> chunk -> embed -> upsert).

---

## Phase 7: Polish & Cross-Cutting Concerns

**Purpose**: Finalize the feature with deployment configurations and error handling.

- [X] T026 [P] Implement graceful error handling on the frontend for backend API failures (e.g., display a "backend offline" message).
- [X] T027 [P] Create a `Dockerfile` and necessary deployment configuration files (e.g., `render.yaml`) for the FastAPI backend.
- [X] T028 [P] Add a production start script to the backend's `package.json` or `pyproject.toml`.
- [X] T029 Update the Docusaurus configuration to ensure the backend URL is correctly used in the production build.
- [ ] T030 Perform a full build and end-to-end test of the feature in a production-like environment. (Manual verification required)

## Dependencies & Execution Order

- **Phase 1 & 2 (Setup & Foundational)**: Must be completed first. Can be done in parallel.
- **User Stories (Phase 3+)**: Depend on Phase 1 & 2 completion.
  - **US1 (P1)**: Can be implemented first to get the UI shell in place.
  - **US4 (P2)**: Can be implemented in parallel with US1/US2, as the backend ingestion pipeline is independent of the chat UI.
  - **US2 (P1)**: Depends on US1 for the UI and US4 for the data.
  - **US3 (P2)**: Depends on US2, as it's an enhancement of the core chat functionality.
- **Phase 7 (Polish)**: Should be done last.
