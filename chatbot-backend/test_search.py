from agent import textbook_search
import os
from dotenv import load_dotenv

load_dotenv()

def test_search():
    try:
        results = textbook_search("ROS 2 architecture")
        print("Search Results:", results)
    except Exception as e:
        import traceback
        traceback.print_exc()
        print(f"Search Failed: {e}")

if __name__ == "__main__":
    test_search()
