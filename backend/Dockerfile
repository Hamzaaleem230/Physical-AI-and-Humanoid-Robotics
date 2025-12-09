# File: backend/Dockerfile (Updated for UV and FastAPI)

FROM python:3.10-slim-buster

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