import os
import psycopg2
from dotenv import load_dotenv

load_dotenv()
url = os.getenv("NEON_DATABASE_URL")
print(f"Connecting to: {url}")

try:
    conn = psycopg2.connect(url)
    print("Connection successful!")
    conn.close()
except Exception as e:
    print(f"Connection failed: {e}")
