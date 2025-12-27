import os
import glob
import re
from embeddings import EmbeddingEngine
from database import q_client, ensure_qdrant_collection
from qdrant_client.models import PointStruct
import uuid

embedder = EmbeddingEngine()
COLLECTION_NAME = "textbook_vectors_gemini"

def clean_mdx(content):
    # Remove frontmatter
    content = re.sub(r'---.*?---', '', content, flags=re.DOTALL)
    # Remove Docusaurus imports
    content = re.sub(r'import.*?from.*?;', '', content)
    # Remove HTML-like tags
    content = re.sub(r'<[^>]+>', '', content)
    return content.strip()

def chunk_text(text, chunk_size=1000, overlap=100):
    chunks = []
    for i in range(0, len(text), chunk_size - overlap):
        chunks.append(text[i:i + chunk_size])
    return chunks

def ingest():
    ensure_qdrant_collection(COLLECTION_NAME)
    
    # Path to your Docusaurus docs
    docs_path = os.path.join("..", "my-website", "docs")
    files = glob.glob(f"{docs_path}/**/*.md", recursive=True)
    
    print(f"Found {len(files)} files to ingest.")
    
    points = []
    
    for file_path in files:
        with open(file_path, "r", encoding="utf-8") as f:
            content = f.read()
            cleaned = clean_mdx(content)
            chunks = chunk_text(cleaned)
            
            relative_path = os.path.relpath(file_path, docs_path)
            
            for i, chunk in enumerate(chunks):
                vector = embedder.generate(chunk)
                points.append(PointStruct(
                    id=str(uuid.uuid4()),
                    vector=vector,
                    payload={
                        "text": chunk,
                        "source": relative_path,
                        "chunk_index": i
                    }
                ))
    
    if points:
        q_client.upsert(
            collection_name=COLLECTION_NAME,
            points=points
        )
        print(f"Successfully ingested {len(points)} points into Qdrant.")

if __name__ == "__main__":
    ingest()
