# pip install SpeechRecognition pyaudio
# brew install portaudio
import speech_recognition as sr
class SpeechToText:
    def __init__(self):
        self.text=''
    def listen_and_recognize(self):
        # Initialize recognizer class (for recognizing the speech)
        recognizer = sr.Recognizer()

        # Set up the microphone for listening
        with sr.Microphone() as source:
            print("Adjusting for ambient noise...")
            recognizer.adjust_for_ambient_noise(source, duration=1)  # Adjust for ambient noise
            print("Listening...")
            try:
            # Stream from the microphone and get audio data
                # Use phrase_time_limit to limit how long a phrase can be
                audio = recognizer.listen(source, phrase_time_limit=150)  # Adjust for phrase time
                print("Recognizing speech...")

                # Use Google Web Speech API to recognize speech
                text = recognizer.recognize_google(audio)

                # Check if the sentence starts with 'Here is the command' or 'Here is the guidance'
                print(f"Recognized text: {text}")
                return text

            except sr.UnknownValueError:
                print("Sorry, I could not understand the audio.")
            except sr.RequestError as e:
                print(f"Could not request results; {e}")
            return False
        
        

if __name__ == "__main__":
    listen_and_recognize()