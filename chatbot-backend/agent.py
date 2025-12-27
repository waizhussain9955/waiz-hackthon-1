import os
import json
import google.generativeai as genai
from embeddings import EmbeddingEngine
from database import q_client
from dotenv import load_dotenv

load_dotenv()

# Configure Gemini
genai.configure(api_key=os.getenv("GOOGLE_API_KEY"))
embedder = EmbeddingEngine()

SYSTEM_PROMPT = """
You are the "Physical AI Textbook Humanoid Assistant". 
Your tone is human-like, helpful, and sophisticated. 
You answer questions ONLY based on the textbook content and the user's selected text if provided.

SOFT-CODING PRINCIPLES:
1. Infer the user's intent from their query schema.
2. If they ask for concepts, search for high-level module summaries.
3. If they ask for code, search for implementation details.
4. If they provide 'selected_text', prioritize it as the immediate context.
5. NEVER hard-code a response. Always use your search tool if you're unsure.

STRICT GROUNDING:
- Only speak from the provided context. 
- If the textbook doesn't cover it, admit it in a polite, human way.
- Maintain conversation context (history).
"""

def textbook_search(query: str):
    """
    Retrieves relevant passages from the Physical AI textbook vector database.
    
    Args:
        query: The semantic search query based on user intent.
    """
    from qdrant_client import QdrantClient
    local_q_client = QdrantClient(
        url=os.getenv("QDRANT_URL"), 
        api_key=os.getenv("QDRANT_API_KEY")
    )
    vector = embedder.generate(query)
    # Use query_points as fallback for missing search attribute in some qdrant-client versions
    try:
        results = local_q_client.query_points(
            collection_name="textbook_vectors",
            query=vector,
            limit=3
        )
        return [p.payload for p in results.points]
    except Exception as e:
        print(f"Query points failed: {e}")
        # Last resort: try classic search if it somehow exists
        try:
            results = local_q_client.search(
                collection_name="textbook_vectors",
                query_vector=vector,
                limit=3
            )
            return [r.payload for r in results]
        except:
            return []

class SoftAgent:
    def __init__(self):
        # Tools configuration
        tools = [textbook_search]
        
        self.model = genai.GenerativeModel(
            model_name='gemini-2.5-flash',
            tools=tools,
            system_instruction=SYSTEM_PROMPT
        )

    def chat(self, user_query, history=[], selected_text=None):
        # Map history to Gemini format
        mapped_history = []
        for msg in history:
            role = "user" if msg["role"] == "user" else "model"
            content = msg.get("content", "")
            if content:
                # Handle potential mixed types if content was Welcome JSX (just skip it)
                if isinstance(content, str):
                    mapped_history.append({"role": role, "parts": [content]})

        # Enable automatic function calling for Gemini
        chat = self.model.start_chat(
            history=mapped_history,
            enable_automatic_function_calling=True
        )
        
        full_query = user_query
        if selected_text:
            full_query = f"CONTEXT FROM USER SELECTION: {selected_text}\n\nUSER QUESTION: {user_query}"

        response = chat.send_message(full_query)
        
        # Collect sources for the response
        sources = []
        for history_item in chat.history:
            for part in history_item.parts:
                if part.function_response:
                    content = part.function_response.response
                    # If it's the list of payloads from textbook_search
                    if isinstance(content, list):
                        sources.extend(content)
                    elif isinstance(content, dict) and "result" in content:
                        sources.extend(content["result"])

        return response.text, sources

