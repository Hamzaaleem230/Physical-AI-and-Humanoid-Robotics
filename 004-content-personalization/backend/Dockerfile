# Python ka stable version use karein
FROM python:3.10-slim-buster

# Container ke andar 'app' naam ka folder banayein
WORKDIR /app

# Sabse pehle requirements copy karein aur install karein
COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

# Ab baki sara code (app folder aur baki files) copy karein
COPY . .

# Hugging Face default port 7860 use karta hai
EXPOSE 7860

# Application ko run karne ki command
# Host 0.0.0.0 lazmi hai taake bahar se access ho sakay
CMD ["uvicorn", "app.main:app", "--host", "0.0.0.0", "--port", "7860"]