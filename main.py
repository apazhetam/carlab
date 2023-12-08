# main.py
import time
from chat_request import chat_session
from speech_detector import detect_speech
import serial

ser = serial.Serial("/dev/ttyAMA0", 115200)

context = "Categorize the following commands into one of these five buckets. \
1: Go forward (F). \
2: Go backwards (B). \
3: Turn left (L). \
4: Turn right (R). \
5: Stop, no movement (S). \
If unsure, default to Stop (S). \
Use either one or two commands in order to accomplish any task. \
Give me just a series of letters, for example 'FB'. Don't give any additional text."

def main():
    # Set initial context or model information
    chat_session.set_context(context)

    while True:
        # Detect a person's speech
        speech = ""

        try:
            speech = detect_speech()
        except KeyboardInterrupt:
            print("Program interrupted by user, exiting...")
            break
        except:
            print("Speech detection failed!")

        # Send the detected speech to ChatGPT
        if (speech != ""):
            try:
                response = chat_session.send_prompt(speech)
                print(response)  # Process the response as needed
                print()

                actions = response['choices'][0]['message']['content']
                print(f'{actions}\n')

                actions += "X"

                # ser.write(bytes(words,'utf-8'))
                for i in range(len(actions)):
                    ser.write(bytes(actions[i],'utf-8'))
                    time.sleep(0.1)

            except Exception as e:
                print("An error occurred:", e)
    
        time.sleep(5)  # Sleep while actions are being executed 

if __name__ == "__main__":
    # print(context)
    main()


# words = "FBRL"
# words += "X"
# print(words)
# ser.write(bytes(words,'utf-8'))
# for i in range(len(words)):
#     ser.write(bytes(words[i],'utf-8'))
#     time.sleep(0.1)