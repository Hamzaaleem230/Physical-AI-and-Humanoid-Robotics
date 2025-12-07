#!/bin/bash
# Start script for FastAPI backend in production

# Activate virtual environment if using venv
# source venv/bin/activate

# For production, typically use a process manager like Gunicorn
# to run Uvicorn workers.
# Example: gunicorn -w 4 -k uvicorn.workers.UvicornWorker app.main:app --bind 0.0.0.0:8000

# For now, directly run Uvicorn for simplicity, assuming a single process.
# This should be replaced with Gunicorn or similar in a true production setup.
uvicorn app.main:app --host 0.0.0.0 --port 8000
