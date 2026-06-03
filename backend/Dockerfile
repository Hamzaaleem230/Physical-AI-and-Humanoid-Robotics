FROM python:3.10-slim-bookworm

WORKDIR /app

# Pehle apt dependencies install karein
RUN apt-get update && apt-get install -y \
    curl \
    git \
    --no-install-recommends && rm -rf /var/lib/apt/lists/*

# Requirements install karein
COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

# APNA SAARA CODE COPY KAREIN (Isse app aur scripts dono /app ke andar chale jayenge)
COPY . .

# Uvicorn run karne ka sahi command
CMD ["uvicorn", "app.main:app", "--host", "0.0.0.0", "--port", "8080"]