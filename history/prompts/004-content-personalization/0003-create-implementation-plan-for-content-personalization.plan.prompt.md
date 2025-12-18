---
id: 0003
title: Create Implementation Plan for Content Personalization
stage: plan
date: 2025-12-17
surface: agent
model: gemini
feature: 004-content-personalization
branch: 004-content-personalization
user: Syed Hamza Aleem
command: /sp.plan
labels: [plan, architecture, personalization]
links:
  spec: specs/004-content-personalization/spec.md
  plan: specs/004-content-personalization/plan.md
  ticket: null
  adr: null
  pr: null
files:
 - specs/004-content-personalization/plan.md
tests:
 - none
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

1. **Setup**: Run `.specify/scripts/bash/setup-plan.sh --json` from repo root and parse JSON for FEATURE_SPEC, IMPL_PLAN, SPECS_DIR, BRANCH. For single quotes in args like "I'm Groot", use escape syntax: e.g 'I''m Groot' (or double-quote if possible: "I'm Groot").

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

   `text
   For each unknown in Technical Context:
     Task: "Research {unknown} for {feature context}"
   For each technology choice:
     Task: "Find best practices for {tech} in {domain}"
   `

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
     - Read `.specify/templates/phr-template.prompt.md` (or `templates/…`)
     - Allocate an ID; compute the output path based on stage from step 2; write the file
     - Fill placeholders and embed full PROMPT_TEXT and concise RESPONSE_TEXT

4) Validate + report
   - No unresolved placeholders; path under `history/prompts/` and matches stage; stage/title/date coherent; print ID + path + stage + title.
   - On failure: warn, don't block. Skip only for `/sp.phr`.


/sp.plan 


You will now create the complete technical implementation plan for:

📁 004-content-personalization

This plan must be fully aligned with the specification created in /sp.specify.  
Do NOT re-invent requirements.  
Do NOT relax constraints.  
Do NOT ignore any boundary.

Your /sp.plan output MUST contain:

────────────────────────────────────────
1. SYSTEM ARCHITECTURE
   – Overall layout of the personalization feature  
   – How it plugs into the existing project  
   – How it avoids modifying folders 001, 002, 003  
   – How frontend, backend, database, and RAG layers interact  
   – Request/response flows  
   – Session dependency with 003-auth-system  
   – Zero interference with RAG logic  
   – Zero interference with book content

────────────────────────────────────────
2. DATA MODEL DESIGN
   – Tables, fields, types, constraints  
   – User preference model  
   – Reading history model  
   – Personalization profiles  
   – Recommendation scoring metadata  
   – Any joins or relationships  
   – Full ERD-level structure (textual)

────────────────────────────────────────
3. API DESIGN
   For every endpoint required by the spec:
   – Full route  
   – Method  
   – Request shape  
   – Response shape  
   – Authentication rules  
   – Validation rules  
   – Expected errors  
   – Dependencies  
   – No placeholders allowed

────────────────────────────────────────
4. PERSONALIZATION ENGINE LOGIC
   – Preference extraction  
   – History tracking  
   – Dynamic content selection rules  
   – User profile enrichment  
   – Recommendation scoring system  
   - Integration with RAG without changing RAG code  
   – Ranking algorithm  
   – Caching (if needed)  
   – Edge case handling

────────────────────────────────────────
5. FRONTEND IMPLEMENTATION PLAN
   – UI screens  
   – Settings page  
   – Preference inputs  
   – How personalization data interacts with UI  
   – How content adapts  
   – State management  
   – Data fetching plan  
   – Error/empty states

────────────────────────────────────────
6. BACKEND IMPLEMENTATION PLAN
   – Controllers  
   – Services  
   – Database handlers  
   – Middlewares  
   – Integration points with 003-auth-system  
   – Ensuring full isolation inside folder 004  
   – Performance considerations  
   – Security considerations

────────────────────────────────────────
7. FULL FOLDER AND FILE STRUCTURE
   – All directories  
   – All files  
   – Purpose of each  
   – No placeholders such as “something.js”  
   – Every file path must be final and real

────────────────────────────────────────
8. STEP-BY-STEP EXECUTION SEQUENCE
   – Ordered list of how to build everything  
   – Clear dependencies  
   – What to do first, second, next  
   – Zero ambiguity

────────────────────────────────────────
9. CRITICAL RULES
   – Do NOT modify any code in 001, 002, or 003  
   – All new code must live ONLY inside 004-content-personalization  
   – No assumptions, no creative freedom  
   – Follow the spec EXACTLY as written  
   – If something in the spec is ambiguous, resolve it using safe defaults  
   – No implementation in this step — only planning  
   – Wait for /sp.tasks command after this

────────────────────────────────────────

STRICT NOTE:
Your plan must be extremely detailed, highly technical, deeply structured, and fully actionable.  
No high-level fluff.  
No generic statements.  
Every step must be specific, concrete, and precise.

Produce ONLY the technical plan.  
Do NOT generate tasks.  
Do NOT start coding.  
Do NOT modify any files.  
Do NOT auto-implement anything.  
Wait for /sp.tasks.

## Response snapshot

The implementation plan for the 'Content Personalization' feature has been created successfully.

## Outcome

- ✅ Impact: A detailed technical implementation plan has been created, covering system architecture, data model, API design, and a step-by-step execution sequence.
- 🧪 Tests: none
- 📁 Files: specs/004-content-personalization/plan.md
- 🔁 Next prompts: /sp.tasks
- 🧠 Reflection: The detailed prompt from the user was crucial for generating a comprehensive and actionable plan.

## Evaluation notes (flywheel)

- Failure modes observed: none
- Graders run and results (PASS/FAIL): PASS
- Prompt variant (if applicable): null
- Next experiment (smallest change to try): none
