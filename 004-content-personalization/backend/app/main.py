import logging
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from contextlib import asynccontextmanager
from .models.database import create_db_tables
from .controllers import personalization

logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(name)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

@asynccontextmanager
async def lifespan(app: FastAPI):
    logger.info("Application startup: Creating database tables.")
    create_db_tables()
    yield
    logger.info("Application shutdown.")

app = FastAPI(
    title="Content Personalization Service",
    lifespan=lifespan
)

# HF par CORS ko flexible rakhna parta hai
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"], # Production mein specific domain bhi dal sakte hain
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

app.include_router(personalization.router, prefix="/personalization")

@app.get("/")
async def root():
    return {"message": "Content Personalization Service is running on HF!"}