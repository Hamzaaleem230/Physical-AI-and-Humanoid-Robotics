<!-- Sync Impact Report:
Version change: 1.1.0 → 1.2.0
Modified principles: None
Added sections:
- Principle VII: User Authentication System
Removed sections: None
Templates requiring updates:
- .specify/templates/plan-template.md: ✅ up-to-date (dynamic)
- .specify/templates/spec-template.md: ✅ up-to-date (dynamic)
- .specify/templates/tasks-template.md: ✅ up-to-date (dynamic)
- .gemini/commands/*.toml: ✅ checked, no changes needed
Follow-up TODOs: RATIFICATION_DATE
-->
# Physical AI and Humanoid Robotics Constitution

## Core Principles

### I. Accuracy
Saari information authentic academic sources, research papers, robotics documentation aur primary technical references se verify ki jayegi.

### II. Clarity
Writing aisi hogi jo computer science / engineering background ke readers ko asani se samajh aaye.

### III. Rigor
Scientific, engineering aur robotics concepts ko peer-reviewed sources se support kiya jayega.

### IV. Reproducibility
Har experiment, workflow, architecture aur robotics pipeline ko reproducible tariqe se explain kiya jayega.

### V. Consistency
Har chapter aik hi technical tone, format aur structure follow karega.

### VI. RAG Chatbot Integration
Docusaurus book ke andar ek RAG-based intelligent chatbot integrate kiya jayega.

- **UI/UX**: Chatbot har page par bottom-right corner mein ek floating chat bubble/button ke form me dikhega.
- **Frontend**: Frontend code `Physical-AI-and-Humanoid-Robotics-Book/src/components/chatbot/` mein rakha jayega. Global injection `Physical-AI-and-Humanoid-Robotics-Book/src/theme/Root.js` ya Layout wrapper ke through hoga.
- **Backend**: Backend FastAPI use karke banaya jayega aur `/embed`, `/query`, `/chat`, `/health`, `/ingest-book` endpoints provide karega.
- **Data Stores**: Embeddings Qdrant Cloud (Free Tier) mein store honge. Optional user accounts Neon Serverless Postgres mein store honge.
- **AI Model**: RAG inference ke liye OpenAI, Gemini 2.0 Flash, ya Claude 3.7 use hoga.
- **Core Features**: Chatbot sirf book ke content se jawab dega, "answer from selected text" mode ko support karega, responses stream karega, aur session ke andar memory rakhega.
- **Project Structure**: Saare naye specs `specs/002-rag-chatbot/` directory mein rakhe jayenge. Purane book specs (`specs/001-physical-ai-book/`) ko touch nahi kiya jayega.

### VII. User Authentication System
A user authentication and profile system will be integrated into the Docusaurus book.

- **Strict Scoping**: All work related to this feature will be strictly contained. Specifications and related documents will reside in `specs/003-auth-system/`. Frontend components will be located in `Physical-AI-and-Humanoid-Robotics-Book/src/auth/`. No modifications shall be made to existing book content or other features (`001-physical-ai-book`, `002-rag-chatbot`).
- **Technology Stack**:
    - **Authentication**: The system will use `better-auth` for user management.
    - **Backend**: The backend will be a FastAPI application hosted on HuggingFace Spaces.
    - **Database**: User and profile data will be stored in a Neon Serverless Postgres database.
    - **Frontend**: The authentication UI will be integrated into the existing Docusaurus application deployed on Vercel.
- **User Profile Data**: The signup process must collect user information, including skill level, hardware availability, robotics experience, OS, and learning mode. This data is intended for future personalization.
- **UI/UX Integration**: The authentication components (signup/signin) must be accessible from the global navigation bar of the Docusaurus site and must not interfere with the static site generation (`npm run build`).
- **API Requirements**: The backend will expose standard authentication endpoints (`/signup`, `/signin`, `/signout`, `/me`) and a profile update endpoint (`/profile-update`). CORS must be configured to allow requests from the frontend.

## Key Standards

- Saare factual claims traceable sources ke sath honge.
- Citation format: APA style.
- Source types: Minimum 50% peer-reviewed (journals, conferences).
- Code examples: Test-run hone chahiye (Python, ROS, simulations, control systems).
- Visuals: Diagrams / flowcharts need to be technically accurate.
- Plagiarism: 0% tolerance.

## Constraints

- Word count: 5,000–7,000 words.
- Minimum 15 academic sources.
- Format: Docusaurus markdown (.md / .mdx) files.
- Output buildable as a static site with valid Docusaurus build.
- Deployment: GitHub Pages per successful CI/CD workflow.
- Writing clarity: Flesch-Kincaid grade 10–12 target.

## Governance

All PRs/reviews must verify compliance. Complexity must be justified.
Success criteria:
- Saare claims sources ke sath verify ho jayen.
- Zero plagiarism.
- Docusaurus build bina error ke compile ho.
- GitHub Pages deployment first attempt me pass ho jae.
- Robotics concepts (AI control, sensing, locomotion, embodiment) academically correct hon.
- Peer-review check pass ho jae (accuracy, clarity, reproducibility).

**Version**: 1.2.0 | **Ratified**: TODO(RATIFICATION_DATE): Original adoption date is unknown. | **Last Amended**: 2025-12-09