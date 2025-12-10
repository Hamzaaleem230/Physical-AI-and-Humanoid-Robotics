from .database import create_db_tables, engine
from .models import Base # Ensure models are imported so Base knows about them

if __name__ == "__main__":
    print("Creating database tables...")
    create_db_tables()
    print("Tables created successfully.")
