# Humanoid RAG Chatbot Backend

This is the "Soft-Coded" RAG agent backend for the Physical AI textbook.

## Features
- **Soft-Coded Agent**: Uses OpenAI Agents to dynamically infer query intent.
- **Local Embeddings**: Uses `all-MiniLM-L6-v2` for privacy and cost-efficiency.
- **Agentic Retrieval**: The agent decides when and how to search the textbook.
- **Contextual Selection**: Supports text selection context from the browser.
- **Neon Cloud Logs**: Stores conversation metadata in Neon Postgres.

## Setup
1. Create a `.env` file based on `.env.example`.
2. Install dependencies:
   ```bash
   pip install -r requirements.txt
   ```
3. Ingest book content:
   ```bash
   python ingest.py
   ```
4. Start the backend:
   ```bash
   python main.py
   ```

## API Endpoints
- `POST /chat`: Main agentic chat endpoint.
  - Body: `{"query": "...", "selected_text": "...", "history": []}`
