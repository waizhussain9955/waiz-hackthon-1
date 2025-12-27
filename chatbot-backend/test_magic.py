import os
from qdrant_client import QdrantClient
from dotenv import load_dotenv

load_dotenv()

client = QdrantClient(url=os.getenv("QDRANT_URL"), api_key=os.getenv("QDRANT_API_KEY"))
print(f"Client type: {type(client)}")
print(f"Has search: {'search' in dir(client)}")
print(f"Has query_points: {'query_points' in dir(client)}")

try:
    # Just try a dummy search to see if it works despite not being in dir (magic method?)
    print("Testing search...")
    # res = client.search(collection_name="textbook_vectors", query_vector=[0.1]*384, limit=1)
    # print("Search worked!")
except Exception as e:
    print(f"Search failed: {e}")
