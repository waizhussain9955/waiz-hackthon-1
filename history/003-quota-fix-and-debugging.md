# Project History: 003 - Debugging Quota and Client Issues

**Date:** December 28, 2025
**Context:** 
We encountered significant issues with the Gemini API (Quota limits on `1.5-flash` and `2.0-flash`) and the `qdrant-client` (missing `search` attribute in some contexts). We also faced port conflicts on 8000.

**Actions Taken:**
1.  **Quota Debugging:**
    *   Tested multiple models (`1.5-flash`, `2.0-flash`, `2.0-flash-exp`).
    *   Identified that `gemini-2.0-flash` was hitting 429 errors.
    *   Switched to `gemini-2.5-flash` which showed availability.
2.  **Qdrant Client Debugging:**
    *   Encountered `AttributeError: 'QdrantClient' object has no attribute 'search'`.
    *   Extensive introspection revealed the method might be `query_points` in newer clients or there was an import conflict.
    *   **FIX:** Re-instantiated `QdrantClient` locally inside the `textbook_search` function and implemented a fallback chain: try `query_points` -> catch -> try `search`.
3.  **Port Conflict:**
    *   Port 8000 was locked. Switched to `8001` and updated `main.py` and the frontend `index.js`.
    *   Changed host to `127.0.0.1` locally to resolve Windows binding issues.
4.  **Database Driver:**
    *   Updated `database.py` to correctly format the Neon URL with `postgresql+psycopg2://`.

**Files Affected:**
*   `chatbot-backend/agent.py` (Model switch, local import, fallback logic)
*   `chatbot-backend/main.py` (Host/Port change)
*   `chatbot-backend/database.py` (Connection string fix)
*   `my-website/src/components/HumanoidChatbot/index.js` (Port update)

**Reasoning:**
The local Qdrant client instantiation avoids global scope pollution or stale references. The fallback logic ensures resilience against client version differences. `gemini-2.5-flash` was the only model accepting requests.

**Outcomes:**
*   Chatbot backend successfully running on `127.0.0.1:8001`.
*   Search functionality confirmed via `test_endpoint.py`.
*   Frontend successfully communicating with backend.
