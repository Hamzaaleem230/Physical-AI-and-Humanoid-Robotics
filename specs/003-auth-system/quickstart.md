# Quickstart: Authentication System

**Branch**: `003-auth-system` | **Date**: 2025-12-09 | **Spec**: [spec.md](./spec.md)

This guide provides the necessary steps to set up the environment for the Authentication System feature.

## Overview

The authentication system consists of two main parts:

1.  **Backend**: A FastAPI application running on a HuggingFace Space.
2.  **Frontend**: React components integrated into the Docusaurus application, deployed on Vercel.

To run this system locally or connect to deployed instances, you need to configure environment variables for both the backend and frontend.

## Backend Setup (`backend/auth-server/`)

Create a file named `.env` in the `backend/auth-server/` directory. This file will store the secrets and configuration for the FastAPI server.

```sh
# .env file for the backend

# 1. Neon Serverless Postgres Connection URL
# Replace with your actual connection string from the Neon console.
DATABASE_URL="postgresql://user:password@host:port/dbname"

# 2. Better-Auth Secret Key
# This is a secret key used to sign JWTs. Generate a strong random string.
# You can use `openssl rand -hex 32` to generate one.
BETTER_AUTH_SECRET_KEY="your-strong-random-secret-key"

# 3. CORS Origin
# The URL of your deployed Docusaurus frontend. For local development,
# you can use "http://localhost:3000". For production, use your Vercel URL.
CORS_ORIGIN="https://your-docusaurus-site.vercel.app"

```

### Running the Backend Locally

1.  Install dependencies: `pip install -r requirements.txt`
2.  Run the server: `uvicorn app.main:app --reload`

## Frontend Setup (`Physical-AI-and-Humanoid-Robotics-Book/`)

Create a file named `.env.local` in the root of the `Physical-AI-and-Humanoid-Robotics-Book/` directory. Docusaurus will automatically load these variables.

```sh
# .env.local file for the frontend

# 1. Backend API Base URL
# The URL of your deployed FastAPI backend on HuggingFace Spaces.
# For local development, use "http://localhost:8000".
REACT_APP_API_BASE_URL="https://your-huggingface-space.hf.space"
```

### Running the Frontend Locally

1.  Navigate to the book directory: `cd Physical-AI-and-Humanoid-Robotics-Book/`
2.  Install dependencies: `npm install`
3.  Run the development server: `npm start`

The application should now be running locally, with the frontend able to communicate with the backend.
