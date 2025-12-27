import os
from qdrant_client import QdrantClient
from dotenv import load_dotenv

load_dotenv()

client = QdrantClient(url=os.getenv("QDRANT_URL"), api_key=os.getenv("QDRANT_API_KEY"))
try:
    res = client.query_points(
        collection_name="textbook_vectors",
        query=[0.1]*384,
        limit=1
    )
    print("Query Points Result:", res)
    print("Point Payload:", res.points[0].payload)
except Exception as e:
    import traceback
    traceback.print_exc()
