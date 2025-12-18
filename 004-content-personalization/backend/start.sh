#!/bin/bash

# Exit immediately if a command exits with a non-zero status.
set -e

# Load environment variables from .env if it exists
if [ -f .env ]; then
    export $(cat .env | xargs)
fi

# Check for required environment variables
if [ -z "$DATABASE_URL" ]; then
    echo "Error: DATABASE_URL is not set."
    exit 1
fi
if [ -z "$AUTH_SERVICE_URL" ]; then
    echo "Error: AUTH_SERVICE_URL is not set."
    exit 1
fi
if [ -z "$SECRET_KEY" ]; then
    echo "Error: SECRET_KEY is not set."
    exit 1
fi

echo "Starting FastAPI application..."
# Start the FastAPI application using Uvicorn
exec uvicorn app.main:app --host 0.0.0.0 --port 8000
