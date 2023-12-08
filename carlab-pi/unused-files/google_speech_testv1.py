import speech_recognition as sr

# Define the recognizer
recognizer = sr.Recognizer()

# Load your WAV file
wav_file = "../mic-test/test1.wav"

# Use the recognizer to perform speech recognition on the file
with sr.AudioFile(wav_file) as source:
    # Listen for the data (load audio to memory)
    audio_data = recognizer.record(source)
    
    # Recognize (convert from speech to text)
    try:
        text = recognizer.recognize_google(audio_data)
        print("Text converted from speech:")
        print(text)
    except sr.UnknownValueError:
        print("Google Speech Recognition could not understand the audio")
    except sr.RequestError as e:
        print(f"Could not request results from Google Speech Recognition service; {e}")
