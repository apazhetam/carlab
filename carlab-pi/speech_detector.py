# speech_detector.py
import speech_recognition as sr

def detect_speech():
    r = sr.Recognizer()
    
    # Select desired mic (find list of mics on conifgure_mic.py)
    mic_index = 1
    mic = sr.Microphone(device_index=mic_index)

    # Print the name of the selected microphone
    mic_list = sr.Microphone.list_microphone_names()
    selected_mic_name = mic_list[mic_index]
    print(f"Selected microphone: {selected_mic_name}")

    try:
        with mic as source:
            print("\nListening...")
            r.adjust_for_ambient_noise(source)  # Adjust for ambient noise
            audio = r.listen(source, timeout=5, phrase_time_limit=10)
            print("Done listening!")

        # Recognize speech using Google Web Speech API
        words = r.recognize_google(audio)
        print(f"You said: {words}")
        return words

    except sr.UnknownValueError:
        print("Google Speech Recognition could not understand audio")
    except sr.RequestError as e:
        print(f"Could not request results from Google Speech Recognition service; {e}")
    except KeyboardInterrupt:
        print("Program interrupted by user")
        raise