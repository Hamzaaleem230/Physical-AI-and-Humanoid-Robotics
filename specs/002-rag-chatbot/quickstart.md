# Quickstart: RAG Chatbot Development

**Date**: 2025-12-07
**Feature**: RAG Chatbot Integration

This guide explains how to set up and run the RAG chatbot feature locally for development.

## Prerequisites

1.  **Python**: 3.10 or higher.
2.  **Node.js**: 18.x or higher (for the Docusaurus frontend).
3.  **Git**: For cloning the repository.
4.  **API Keys**: You will need API keys from the following services:
    -   An LLM provider (OpenAI, Google AI Studio, or Anthropic).
    -   Qdrant Cloud (for the vector database).
    -   (Optional) Neon aiven for the Postgres database if you are working on user authentication.

## 1. Backend Setup (FastAPI)

The backend service is located in the `backend/` directory at the root of the repository.

```bash
# 1. Navigate to the backend directory
cd backend

# 2. Create and activate a Python virtual environment
python -m venv venv
source venv/bin/activate  # On Windows, use `venv\Scripts\activate`

# 3. Install the required Python packages
pip install -r requirements.txt

# 4. Set up environment variables
# Copy the template to a new .env file
cp .env.template .env

# 5. Edit the .env file with your editor and fill in the API keys
# Example:
# OPENAI_API_KEY="sk-..."
# QDRANT_API_KEY="..."
# QDRANT_URL="..."
# ALLOWED_ORIGINS="http://localhost:3000"

# 6. Run the backend server
uvicorn app.main:app --reload
```

The backend API will now be running at `http://127.0.0.1:8000`.

## 2. Frontend Setup (Docusaurus Integration)

The frontend components are integrated into the existing Docusaurus project.

```bash
# 1. Navigate to the Docusaurus project directory
cd Physical-AI-and-Humanoid-Robotics-Book

# 2. Install the required Node.js packages
npm install

# 3. Set up environment variables for the frontend
# You may need to configure the backend API URL if it's not the default.
# This will likely be done in a Docusaurus configuration file or a new .env file.
# Example for a .env file in this directory:
# BACKEND_API_URL="http://127.0.0.1:8000"

# 4. Run the Docusaurus development server
npm run start
```

The Docusaurus book will now be running at `http://localhost:3000`, and you should see the chatbot widget on the pages.

## 3. First-Time Content Ingestion

Before the chatbot can answer questions, you need to populate the vector database with the book's content.

1.  Make sure the backend server is running.
2.  Use an API client (like Postman, Insomnia, or `curl`) to send a POST request to the ingestion endpoint:

```bash
curl -X POST http://127.0.0.1:8000/ingest-book
```

This will trigger the backend process to read all the markdown files, chunk them, generate embeddings, and store them in your Qdrant collection. This may take a few minutes. You can monitor the backend server's logs for progress.

After ingestion is complete, the chatbot is ready to use.
