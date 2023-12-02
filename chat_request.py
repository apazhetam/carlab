# chat_request.py
import requests
from credentials import openai_api_key

class ChatGPTSession:
    def __init__(self):
        self.session_prompt = ""  # Stores the initial context or model information

    def set_context(self, context):
        self.session_prompt = context

    def send_prompt(self, prompt):
        combined_prompt = self.session_prompt + "\n\n" + prompt
        headers = {
            'Authorization': f'Bearer {openai_api_key}'
        }
        data = {
            'model': 'gpt-3.5-turbo',  # You can change the model as needed
            'messages': [{"role": "user", "content" : combined_prompt}],
            'max_tokens': 150  # Adjust as necessary
        }
        response = requests.post('https://api.openai.com/v1/chat/completions', headers=headers, json=data)
        return response.json()

# Create a global session object
chat_session = ChatGPTSession()
