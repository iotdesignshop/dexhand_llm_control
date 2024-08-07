import openai
from openai import OpenAI


# This is our default prompt used whenever a session is started or reset
default_message_stack = [
    { 
        "role": "system",
        "content": 
            "Your name is now DexHand. We are going to use a humanoid robot hand as an  \
            output device which you will use to answer some basic questions about status, \
            mathematics and other things we may arrive upon in a conversation. \
            I will ask you some questions, and you figure out how to respond \
            using the hand as an output device if you can. \
            The hand has five digits in total. Four fingers and one thumb. \
            The API that you will have access to is pretty simple. For each digit on the hand, \
            you will be able to set the extension of the digit from 0.0 to 1.0. \
            0.0 means the digit is fully retracted and curled in toward the palm. \
            1.0 means the digit is fully extended and pointing up away from the palm of the hand. \
            So, a fist would be the result of a hand with all the fingers and thumb \
            set to 0.0 and an open hand would be all the fingers and the thumb set to 1.0. \
            It is valid to use the thumb to express a quantity if you need it to. \
            I would like you to issue responses to the questions by showing the extension of \
            each finger - they are named index, middle, ring, pinky, and thumb. \
            For numerical answers, please answer in digital where each finger represents \
            a count of 1. For example, if I asked you to show me the number 3, \
            you would extend your index, middle, and ring fingers. \
            Return the results in a valid JSON object. \
            For example: If I asked you to \"make a fist\" \
            Response should be { \"index\": 0.0, \"middle\": 0.0, \"ring\": 0.0, \"pinky\": 0.0, \"thumb:\"0.0 } \
            If I said \"show the peace sign\" \
            Response should be { \"index\": 1.0, \"middle\": 1.0, \"ring\": 0.0, \"pinky\": 0.0, \"thumb:\"0.0 } \
            The API also has a gesture command that will allow you to perform some simple \
            common animated hand gestures that are built into the hand firmware. \
            These gestures are: \
            \"wave\" - Waves the hand like you are saying hello or goodbye \
            \"shaka\" - Performs the Hawaiian shaka sign \
            If you feel like these are an appropriate response to a statement from the \
            user, you can respond with the gesture command in a JSON object as follows: \
            { \"gesture\": \"wave\" } \
            { \"gesture\": \"shaka\" } \
            If the answer or response can be calculated and returned, please return only the \
            JSON response and no commentary. If it cannot, or if I ask for clarification of \
            the answer, please return a text description explaining the answer"
    }           
]


# Class for managing a conversation session with ChatGPT
class OpenAISession:

    def __init__(self, key, logger):

        self.logger = logger

        # Set the OpenAI key
        self.client = OpenAI(api_key=key)

        # Start with a clean prompt
        self.resetPrompt()

    # Resets the message stack to the original prompt
    def resetPrompt(self):
        self.message_stack = default_message_stack

    # Processes a message from the user, returning a response, and adding the results
    # to the message stack (context)
    def processPrompt(self, prompt):

        # Add the user's message to the stack
        self.message_stack.append({ "role": "user", "content": prompt })

        # Send to GPT4
        try:
            response = self.client.chat.completions.create(model="gpt-4",
            messages=self.message_stack,
            temperature=0,
            max_tokens=1280,
            top_p=1,
            frequency_penalty=0,
            presence_penalty=0)
        # Catch rate limit
        except openai.RateLimitError as e:
            self.logger.info(str(e))
            return None, "I'm sorry. We're talking too fast and hit the rate limit. Try again in a few seconds."

        # Log to debug
        self.logger.debug(str(response))

        # Extract response message
        message = response.choices[0].message.content

        # Push the response to the message stack
        self.message_stack.append({ "role": "assistant", "content": message })

        # Search the message for a JSON object starting encapsulated in {}, as this
        # means the system detected a response that can be processed by the robot hand
        startbrace = message.find("{")
        endbrace = message.find("}")

        if (startbrace != -1 and endbrace != -1):
            jsonmessage = message[startbrace:endbrace+1]
        else:
            jsonmessage = None

        # If the response doesnt start with {, then it is a text response
        if startbrace != 0:
            # If the brace is not at the start of the message then ChatGPT has added some text to 
            # explain what happened. Replace the JSON with "this" so that the speech engine will
            # say "this" instead of the JSON. It sounds more natural.
            if (jsonmessage is not None):
                textmessage = message.replace(jsonmessage, "this")
            else:
                textmessage = message
        else:
            textmessage = None

        return jsonmessage, textmessage

