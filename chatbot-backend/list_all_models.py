import os
import google.generativeai as genai
from dotenv import load_dotenv

load_dotenv()
genai.configure(api_key=os.getenv("GOOGLE_API_KEY"))

try:
    for m in genai.list_models():
        print(f"Model: {m.name}, Methods: {m.supported_generation_methods}")
except Exception as e:
    print(e)
