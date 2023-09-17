import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import os
import speech_recognition as sr
from gtts import gTTS
import threading
from dexhand_llm_control.openai_session import OpenAISession

class LLMControlNode(Node):
    def __init__(self):
        super().__init__('llm_control')

        # Retreive the OpenAI key from the environment
        openai_key = os.environ.get('OPENAI_API_KEY')
        if openai_key is None:
            self.get_logger().error('OPENAI_API_KEY environment variable not set. Cannot continue.')
            exit(1)
        self.openai_session = OpenAISession(openai_key, self.get_logger())

        # Default publisher for hand 
        self.publisher_ = self.create_publisher(String, '/dexhand_command', 10)

        # Set up speech recognition
        self.recognizer = sr.Recognizer()
        self.recognizer.energy_threshold = 4000

        # Start audio processing loop in a thread
        self.audio_thread = threading.Thread(target=self.message_processing_loop)
        self.audio_thread.start()
        
 #   def read_from_serial(self):
 #       if self.serial_conn.in_waiting:
 #           response = self.serial_conn.readline().decode('utf-8').strip()
 #           self.get_logger().info('Received from Arduino: "%s"' % response)
 #           msg = String()
 #           msg.data = response
 #           self.publisher_.publish(msg)

    # Basic wrapper for Google Text to Speech. Takes a string, speaks it, and then deletes the file.
    def textToSpeech(self, text):
        tts = gTTS(text=text, lang='en')
        tts.save("speech.mp3")
        os.system("mpg123 speech.mp3")
        os.remove("speech.mp3")

    # This is our main message processor that records audio from the user and 
    # attempts to process it with OpenAI. It runs in a thread.
    def message_processing_loop(self):
        while rclpy.ok():
            with sr.Microphone() as source:
                self.get_logger().info('Ready for audio input')
                audio = self.recognizer.listen(source)

                try:
                    # for testing purposes, we're just using the default API key
                    # to use another API key, use `r.recognize_google(audio, key="GOOGLE_SPEECH_RECOGNITION_API_KEY")`
                    # instead of `r.recognize_google(audio)`
                    query = self.recognizer.recognize_google(audio)
                    print("Google Speech Recognition thinks you said " + query)

                    # Send the query to OpenAI
                    jsonmessage,textmessage = self.openai_session.processPrompt(query)

                    # If the we received a jsonmessage, we format that into a command and send it to the hand
                    if jsonmessage is not None:
                        self.get_logger().info('Sending to DexHand: "%s"' % jsonmessage)
                        #msg = String()
                        #msg.data = jsonmessage
                        #self.publisher_.publish(msg)
                        #self.textToSpeech(textmessage)

                    # If we received a text message, we just speak it
                    if textmessage is not None:
                        self.get_logger().info('Speaking to user: "%s"' % textmessage)
                        self.textToSpeech(textmessage)

                except sr.UnknownValueError:
                    print("Google Speech Recognition could not understand audio")
                    self.textToSpeech("I'm sorry, I didn't understand that. Try again.")

                except sr.RequestError as e:
                    print("Could not request results from Google Speech Recognition service; {0}".format(e))
                    self.textToSpeech("I'm sorry, there is an error with Google Speech Recognition.")

        

def main(args=None):
    rclpy.init(args=args)
    dexhand_node = LLMControlNode()
    rclpy.spin(dexhand_node)
    dexhand_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
