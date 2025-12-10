---
id: 0002
title: Create Plan for Auth System
stage: plan
date: 2025-12-09
surface: agent
model: gemini-pro
feature: 003-auth-system
branch: 003-auth-system
user: default-user
command: /sp.plan
labels: [plan, auth-system, design]
links:
  spec: specs/003-auth-system/spec.md
  ticket: null
  adr: null
  pr: null
files:
 - specs/003-auth-system/plan.md
 - specs/003-auth-system/research.md
 - specs/003-auth-system/data-model.md
 - specs/003-auth-system/contracts/openapi.yml
 - specs/003-auth-system/quickstart.md
tests:
 - 
---

## Prompt

---
description: Execute the implementation planning workflow using the plan template to generate design artifacts.
---

## User Input

```text
$ARGUMENTS
```

You **MUST** consider the user input before proceeding (if not empty).

## Outline

1. **Setup**: Run `.specify/scripts/bash/setup-plan.sh --json` from repo root and parse JSON for FEATURE_SPEC, IMPL_PLAN, SPECS_DIR, BRANCH. For single quotes in args like "I'm Groot", use escape syntax: e.g 'I'm Groot' (or double-quote if possible: "I'm Groot").

2. **Load context**: Read FEATURE_SPEC and `.specify/memory/constitution.md`. Load IMPL_PLAN template (already copied).

3. **Execute plan workflow**: Follow the structure in IMPL_PLAN template to:
   - Fill Technical Context (mark unknowns as "NEEDS CLARIFICATION")
   - Fill Constitution Check section from constitution
   - Evaluate gates (ERROR if violations unjustified)
   - Phase 0: Generate research.md (resolve all NEEDS CLARIFICATION)
   - Phase 1: Generate data-model.md, contracts/, quickstart.md
   - Phase 1: Update agent context by running the agent script
   - Re-evaluate Constitution Check post-design

4. **Stop and report**: Command ends after Phase 2 planning. Report branch, IMPL_PLAN path, and generated artifacts.

## Phases

### Phase 0: Outline & Research

1. **Extract unknowns from Technical Context** above:
   - For each NEEDS CLARIFICATION → research task
   - For each dependency → best practices task
   - For each integration → patterns task

2. **Generate and dispatch research agents**:

   ```text
   For each unknown in Technical Context:
     Task: "Research {unknown} for {feature context}"
   For each technology choice:
     Task: "Find best practices for {tech} in {domain}"
   ```

3. **Consolidate findings** in `research.md` using format:
   - Decision: [what was chosen]
   - Rationale: [why chosen]
   - Alternatives considered: [what else evaluated]

**Output**: research.md with all NEEDS CLARIFICATION resolved

### Phase 1: Design & Contracts

**Prerequisites:** `research.md` complete

1. **Extract entities from feature spec** → `data-model.md`:
   - Entity name, fields, relationships
   - Validation rules from requirements
   - State transitions if applicable

2. **Generate API contracts** from functional requirements:
   - For each user action → endpoint
   - Use standard REST/GraphQL patterns
   - Output OpenAPI/GraphQL schema to `/contracts/`

3. **Agent context update**:
   - Run `.specify/scripts/bash/update-agent-context.sh gemini`
   - These scripts detect which AI agent is in use
   - Update the appropriate agent-specific context file
   - Add only new technology from current plan
   - Preserve manual additions between markers

**Output**: data-model.md, /contracts/*, quickstart.md, agent-specific file

## Key rules

- Use absolute paths
- ERROR on gate failures or unresolved clarifications

---

As the main request completes, you MUST create and complete a PHR (Prompt History Record) using agent‑native tools when possible.

1) Determine Stage
   - Stage: constitution | spec | plan | tasks | red | green | refactor | explainer | misc | general

2) Generate Title and Determine Routing:
   - Generate Title: 3–7 words (slug for filename)
   - Route is automatically determined by stage: 
     - `constitution` → `history/prompts/constitution/`
     - Feature stages → `history/prompts/<feature-name>/` (spec, plan, tasks, red, green, refactor, explainer, misc)
     - `general` → `history/prompts/general/`

3) Create and Fill PHR (Shell first; fallback agent‑native)
   - Run: `.specify/scripts/bash/create-phr.sh --title "<title>" --stage <stage> [--feature <name>] --json`
   - Open the file and fill remaining placeholders (YAML + body), embedding full PROMPT_TEXT (verbatim) and concise RESPONSE_TEXT.
   - If the script fails:
     - Read `.specify/templates/phr-template.prompt.md` (or `templates/...`)
     - Allocate an ID; compute the output path based on stage from step 2; write the file
     - Fill placeholders and embed full PROMPT_TEXT and concise RESPONSE_TEXT

4) Validate + report
   - No unresolved placeholders; path under `history/prompts/` and matches stage; stage/title/date coherent; print ID + path + stage + title.
   - On failure: warn, don't block. Skip only for `/sp.phr`.


/sp.plan

Plan Directory: specs/003-auth-system/

Phase 1 — Frontend Setup
1. Create folder: Physical-AI-and-Humanoid-Robotics-Book/src/auth/
2. Create SignupModal + SigninModal components
3. Add user background form fields (Full Name, Email, Skill Level, Hardware, Robotics Experience, OS, Learning Mode)
4. Connect to better-auth client SDK
5. Add navbar auth button and persist "Signed in as <name>" (store session in localStorage/session)

Phase 2 — Backend Setup
1. Create backend/auth-server/ folder
2. Setup better-auth server SDK (FastAPI)
3. Connect to Neon Serverless Postgres
4. Create DB schema for users + profiles
   - users(id, email, password_hash, created_at)
   - profiles(id, user_id, skill_level, hardware, robotics_experience, os, learning_mode)
5. Implement /auth routes:
   - POST /auth/signup
   - POST /auth/signin
   - POST /auth/signout
   - GET  /auth/me
   - POST /auth/profile-update
6. Add robust input validation, error handling, and logging

Phase 3 — Integration
1. Add global AuthProvider in Docusaurus via src/theme/Root.js (or Layout wrapper if required)
2. Sync session to frontend and expose auth context
3. Expose user profile to the existing RAG chatbot through a read-only integration point (do NOT modify chatbot internals)

Phase 4 — Deployment
1. Deploy backend to HuggingFace Spaces (FastAPI)
2. Ensure requirements.txt, app.py, and neon connection environment variables are present in the Space
3. Configure CORS to allow requests from the Docusaurus frontend (Vercel domain)
4. Deploy frontend to Vercel and validate static export does not break
5. Run end-to-end tests (signup, signin, profile-save, navbar display)

Deliverables:
- Signup & Signin fully working
- User background stored in Neon Postgres
- Auth state available in navbar and exposed to RAG chatbot (read-only)
- All spec/plan/tasks files saved in specs/003-auth-system/

Note:

You must execute every requirement above with absolute, uncompromising strictness — no assumptions, no shortcuts, no creative interpretation, and no deviations. Implement everything EXACTLY, PRECISELY, and COMPLETELY as written. There is zero tolerance for mistakes, omissions, or partial fulfillment. You are strictly forbidden from modifying, altering, touching, or changing any existing code, files, folders, or components outside the specified scope; only the tasks explicitly stated above must be implemented.

## Response snapshot

The implementation plan for feature '003-auth-system' is complete. All design artifacts have been generated in the `specs/003-auth-system/` directory, including the plan, research notes, data model, API contract, and a quickstart guide. The project is ready for task breakdown via `/sp.tasks`.

## Outcome

- ✅ Impact: Successfully generated a comprehensive implementation plan and all associated design artifacts, providing a clear blueprint for development.
- 🧪 Tests: No tests were run. This was a planning and design documentation task.
- 📁 Files: Created `plan.md`, `research.md`, `data-model.md`, `contracts/openapi.yml`, and `quickstart.md`.
- 🔁 Next prompts: The user can now proceed with `/sp.tasks` to break down the plan into specific, actionable development tasks.
- 🧠 Reflection: The process was smooth. The user provided a clear, phased implementation plan, which was successfully translated into the formal design artifacts required by the project's workflow.

## Evaluation notes (flywheel)

- Failure modes observed: None observed.
- Graders run and results (PASS/FAIL): PASS
- Prompt variant (if applicable): null
- Next experiment (smallest change to try): N/A