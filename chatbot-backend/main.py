from fastapi import FastAPI, Depends, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
from typing import List, Optional
from agent import SoftAgent
from database import get_db, Conversation
from sqlalchemy.orm import Session
import uuid

app = FastAPI(title="Humanoid RAG API")

# Enable CORS for Docusaurus
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_methods=["*"],
    allow_headers=["*"],
)

agent = SoftAgent()

class ChatRequest(BaseModel):
    query: str
    selected_text: Optional[str] = None
    history: List[dict] = []

class ChatResponse(BaseModel):
    response: str
    sources: List[dict]

@app.post("/chat", response_model=ChatResponse)
async def chat_endpoint(request: ChatRequest, db: Session = Depends(get_db)):
    try:
        response_text, context = agent.chat(
            user_query=request.query,
            history=request.history,
            selected_text=request.selected_text
        )
        
        # Save to Neon
        conv = Conversation(
            id=str(uuid.uuid4()),
            user_query=request.query,
            agent_response=response_text,
            context_used=context
        )
        db.add(conv)
        db.commit()
        
        return ChatResponse(
            response=response_text,
            sources=context
        )
    except Exception as e:
        import traceback
        traceback.print_exc()
        raise HTTPException(status_code=500, detail=str(e))

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="127.0.0.1", port=8001)
