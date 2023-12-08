import speech_recognition as sr

def main():
    r = sr.Recognizer()
    mic_list = sr.Microphone.list_microphone_names()
    print("Available microphones:")
    for index, name in enumerate(mic_list):
        print(f"{index}: {name}")

    # Select the USB PnP Sound Device, which is at index 2 (or another index as needed)
    mic_index = 2
    mic = sr.Microphone(device_index=mic_index)

    # Print the name of the selected microphone
    selected_mic_name = mic_list[mic_index]
    print(f"Selected microphone: {selected_mic_name}")

    print("\nStart talking!")

    while True:
        try:
            with mic as source:
                print("Listening...")
                r.adjust_for_ambient_noise(source)  # Adjust for ambient noise
                audio = r.listen(source)
                print("Done listening!")

            # Recognize speech using Google Web Speech API
            words = r.recognize_google(audio)
            print(f"You said: {words}")

        except sr.UnknownValueError:
            print("Google Speech Recognition could not understand audio")
        except sr.RequestError as e:
            print(f"Could not request results from Google Speech Recognition service; {e}")

if __name__ == "__main__":
    main()
