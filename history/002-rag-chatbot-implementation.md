# Project History: 002 - RAG Chatbot Implementation

**Date:** December 27, 2025
**Context:** 
We proceeded to implement "Module 05: RAG Chatbot" to separate the advanced AI agent integration from the static textbook content. The goal was a "soft-coded" agent using Google Gemini, Qdrant Cloud, and Neon Postgres.

**Actions Taken:**
1.  **Backend Infrastructure (`chatbot-backend/`):**
    *   Setup FastAPI with `uvicorn`.
    *   Integrated `google-generativeai` for the LLM.
    *   Integrated `qdrant-client` for vector search.
    *   Integrated `sqlalchemy` + `psycopg2` for conversation logging in Neon Postgres.
2.  **frontend Integration:**
    *   Created `HumanoidChatbot` React component in Docusaurus.
    *   Styled with a dark glassmorphism theme (`styles.module.css`).
    *   Injected globally via `Root.js`.
3.  **Agent Logic (`agent.py`):**
    *   Implemented `SoftAgent` class.
    *   Defined `textbook_search` tool using `sentence-transformers` (`all-MiniLM-L6-v2`).
    *   Configured Gemini to use automatic function calling for dynamic search.
4.  **Ingestion:**
    *   Created `ingest.py` to index Docusaurus markdown files into Qdrant.

**Files Affected:**
*   `chatbot-backend/main.py`, `agent.py`, `database.py`, `ingest.py`
*   `my-website/src/components/HumanoidChatbot/`
*   `my-website/src/theme/Root.js`

**Reasoning:**
Separating the backend allows for independent scaling and ensures the Docusaurus site remains static/fast while the "brain" runs dynamically. Using Gemini 1.5/2.0 allows for long-context understanding.

**Outcomes:**
*   Backend structure established.
*   Frontend component verified visually.
*   Initial Qdrant ingestion successful.
