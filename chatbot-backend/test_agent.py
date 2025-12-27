from agent import SoftAgent
import os
from dotenv import load_dotenv

load_dotenv()

def test_chat():
    agent = SoftAgent()
    try:
        response, sources = agent.chat("hi", history=[])
        print("Response:", response)
        print("Sources:", sources)
    except Exception as e:
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    test_chat()
