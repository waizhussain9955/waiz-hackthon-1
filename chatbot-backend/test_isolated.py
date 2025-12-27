import os
import google.generativeai as genai
from qdrant_client import QdrantClient
from sentence_transformers import SentenceTransformer
from dotenv import load_dotenv

load_dotenv()

def test_everything():
    print("Initializing Model...")
    model = SentenceTransformer('all-MiniLM-L6-v2')
    
    print("Connecting to Qdrant...")
    client = QdrantClient(url=os.getenv("QDRANT_URL"), api_key=os.getenv("QDRANT_API_KEY"))
    
    print("Checking dir(client)...")
    if 'search' in dir(client):
        print("Search exists!")
    else:
        print("Search MISSING!")
    
    print("Generating Vector...")
    vector = model.encode(["test"])[0].tolist()
    
    print("Searching...")
    try:
        res = client.search(collection_name="textbook_vectors", query_vector=vector, limit=1)
        print("Search Success!")
    except Exception as e:
        print(f"Search Failed: {e}")

if __name__ == "__main__":
    test_everything()
