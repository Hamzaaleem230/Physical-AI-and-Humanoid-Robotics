# File: backend/Dockerfile (Updated for UV and FastAPI)

FROM python:3.10-slim-buster
# 1. Install necessary packages (curl, git) in the lightweight image
RUN apt-get update && apt-get install -y \
    curl \
    git \
    --no-install-recommends && rm -rf /var/lib/apt/lists/*

# 1. UV installation (Fast package manager)
WORKDIR /app
RUN curl -LsSf https://astral.sh/uv/install.sh | sh
ENV PATH="/root/.local/bin:${PATH}"

# 2. Copy requirements and install dependencies
COPY requirements.txt .
RUN uv pip install --system -r requirements.txt

# 3. Copy the rest of the application code
COPY . .

# 4. FastAPI Start Command
CMD ["uvicorn", "app.main:app", "--host", "0.0.0.0", "--port", "7860"]