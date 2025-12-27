import google.generativeai as genai
import os
from dotenv import load_dotenv

load_dotenv()

class EmbeddingEngine:
    def __init__(self):
        self.api_key = os.getenv("GOOGLE_API_KEY")
        if not self.api_key:
            print("Warning: GOOGLE_API_KEY not found in environment variables")
        else:
            genai.configure(api_key=self.api_key)
            print("Embedding Engine initialized with Google Gemini")

    def generate(self, text):
        if not self.api_key:
            raise ValueError("GOOGLE_API_KEY is required for embeddings")
            
        if isinstance(text, str):
            text = [text]
            
        # Gemini embedding-001 output dimension is 768
        result = genai.embed_content(
            model="models/embedding-001",
            content=text,
            task_type="retrieval_document",
            title="Embedding of list of strings"
        )
        
        # result['embedding'] is a list of embeddings. 
        # If we passed a list, we get a list of lists.
        # But wait, embed_content usually takes a single string or a list. 
        # If input is list, result['embedding'] is list of lists.
        
        embeddings = result['embedding']
        
        # Match previous behavior: return list of floats if 1 input, else list of lists
        return embeddings[0] if len(embeddings) == 1 else embeddings
