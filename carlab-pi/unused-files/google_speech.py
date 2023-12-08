import speech_recognition as sr

# Initialize the recognizer
r = sr.Recognizer()

# List available microphones and print them
mic_list = sr.Microphone.list_microphone_names()
print("Available microphones:")
for index, name in enumerate(mic_list):
    print(f"{index}: {name}")

# Select the USB PnP Sound Device, which is at index 2
mic_index = 1
mic = sr.Microphone(device_index=mic_index)

# Print the name of the selected microphone
selected_mic_name = mic_list[mic_index]
print(f"Selected microphone: {selected_mic_name}")

while True:
    # try:
        print("")
        print("START TALKING...")
        print("")
        with mic as source:
            audio = r.listen(source)
        words = r.recognize_google(audio)
        print(words)
    # except:
    #     print("ERROR: I didn't understand what you said....")
