import requests
import json

url = "http://localhost:8001/chat"
payload = {
    "query": "What is ROS 2?",
    "history": [],
    "selected_text": ""
}
headers = {
    "Content-Type": "application/json"
}

try:
    response = requests.post(url, json=payload, headers=headers)
    print(f"Status Code: {response.status_code}")
    print(f"Response: {response.text}")
except Exception as e:
    print(f"Error: {e}")
