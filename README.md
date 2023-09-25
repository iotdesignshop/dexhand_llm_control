# dexhand_llm_control
A demonstration package for connecting the DexHand to GPT-4 via ROS 2. Check out our YouTube video below of the package in action!

[<img src="https://github.com/iotdesignshop/dexhand_llm_control/assets/2821763/834e62f0-065b-4caa-a0cd-f984665a38ad" width=300/>](https://youtu.be/GWHLRgOuJLU)

## What Does This Package Do?

This is an example package that connects ChatGPT (GPT-4) to the DexHand via a speech-based interface powered by Google Speech. Or, in other words, it lets you have a conversation with the DexHand! ChatGPT will try to use the hand to answer the questions you pose to it. This can be both interesting and perplexing/confusing, but you can always ask ChatGPT to clarify why it has made the decisions it has. This can be quite insightful.

### What Can It Do?

* Math operations - try asking the hand "What is 7-4?" or "What is 2+2" and it will show you a finger count
* Hand poses - Ask the hand to make the peace sign, or the devil horns, and see what happens
* Some animations - The firmware has a couple of basic hand animations - "Wave" and "Shaka". ChatGPT has been instructed to respond with these animations if it thinks they are appropriate.
* Other stuff - There's a lot of stuff we probably haven't considered. Try it and see how the system reacts!

## How Does It Work?

Internally, the components are as follows:

* **SpeechRecognition** - The Python SpeechRecogniton package uses Google Speech Recognition to record audio from the user and covert it to text.
* **OpenAI API** - This text prompt is forwarded to ChatGPT (GPT-4) via the OpenAI API, and the response is parsed.
* **DexHand Gesture Controller** - If a valid hand pose or gesture is returned by ChatGPT, it is forwarded to the DexHand Gesture Controller via ROS 2 messages
* **Google Text to Speech** - If any commentary, or additional context is returned by ChatGPT, it is converted back to speech via the gTTS API and played on the speaker for the user to hear.

## Speaker Input

By default, the Speech Recognizer will default to the microphone available on the host computer. We also scan for the [Seeed Studios Respeaker](https://www.seeedstudio.com/ReSpeaker-USB-Mic-Array-p-4247.html?queryID=49231497d3832aaa8264eac787e73027&objectID=4247&indexName=bazaar_retailer_products), which is a good alternate speaker array for busy or noisy environments. 

If it's detected on the most machine, the Respeaker will be used instead of the built-in mic. Otherwise, the system falls back to the default system audio input.


## ROS 2 Command Line

To run the node:

`ros2 run dexhand_llm_control llm_control`

