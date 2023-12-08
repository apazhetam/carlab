from openai import OpenAI
from credentials import openai_api_key

client = OpenAI(api_key=openai_api_key)

def transcribe_audio(file_path):

   with open(file_path, 'rb') as audio_file:
      transcript = client.audio.transcriptions.create(
         model="whisper-1", 
         file=audio_file
      )
   
   return transcript
