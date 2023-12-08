# main.py
import time
from chat_request import chat_session

def main():
   # Set initial context or model information
   chat_session.set_context("You need to categorize any following command into one of four categories: 1. go straight. 2. go left. 3. go right. 4. stop")

   #while True:
   # Example: Send a prompt within the context
   prompt = "The mall is directly in front of me. how do i get there"
   response = chat_session.send_prompt(prompt)
   print(response)  # Process the response as needed

   # time.sleep(1)  # Sleep to prevent overloading the CPU, adjust as needed

if __name__ == "__main__":
   main()
