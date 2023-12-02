# main.py
import time
from chat_request import chat_session
from speech_detector import detect_speech

def main():
    # Set initial context or model information
    chat_session.set_context("You need to categorize any following command into one of four categories: 1. go straight. 2. go left. 3. go right. 4. stop")

    while True:
        # Detect a person's speech
        try:
            speech = detect_speech()
        except KeyboardInterrupt:
            print("Program interrupted by user, exiting...")
            break
        except:
            print("Speech detection failed!")

        # Send the detected speech to ChatGPT
        try:
            response = chat_session.send_prompt(speech)
            print(response)  # Process the response as needed
            print()

        except Exception as e:
            print("An error occurred:", e)

        time.sleep(1)  # Sleep to prevent overloading the CPU, adjust as needed

if __name__ == "__main__":
    main()
