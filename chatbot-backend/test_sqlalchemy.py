from sqlalchemy import create_engine
from sqlalchemy.orm import sessionmaker
from sqlalchemy.ext.declarative import declarative_base
from sqlalchemy import Column, Integer, String
import os
from dotenv import load_dotenv

load_dotenv()
DATABASE_URL = os.getenv("NEON_DATABASE_URL")
# Ensure using psycopg2
if DATABASE_URL and not DATABASE_URL.startswith("postgresql+psycopg2://"):
     DATABASE_URL = DATABASE_URL.replace("postgresql://", "postgresql+psycopg2://")

print(f"Using DB URL: {DATABASE_URL}")

try:
    engine = create_engine(DATABASE_URL)
    connection = engine.connect()
    print("SQLAlchemy Connection successful!")
    connection.close()
except Exception as e:
    import traceback
    traceback.print_exc()
