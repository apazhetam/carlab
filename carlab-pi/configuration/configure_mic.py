# configure_mic.py
# Use this to check the available microphones
import speech_recognition as sr
mic_list = sr.Microphone.list_microphone_names()
print("Available microphones:")
for index, name in enumerate(mic_list):
    print(f"{index}: {name}")