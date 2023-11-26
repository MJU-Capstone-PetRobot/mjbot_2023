import numpy as np
import sounddevice as sd
from openwakeword.model import Model

# Audio Configuration
CHANNELS = 1
RATE = 48000
CHUNK = 80
DEVICE = 5  # Assuming device index 5 is the correct microphone

# Initialize the wake word model
owwModel = Model(wakeword_models=["./src/mjbot_voice/models/hi.tflite"], inference_framework="tflite")

def audio_callback(indata, frames, time, status):
    if status:
        print(status)
    
    audio_data = np.frombuffer(indata, dtype=np.int16)
    prediction = owwModel.predict(audio_data)

    for mdl in owwModel.prediction_buffer.keys():
        scores = list(owwModel.prediction_buffer[mdl])
        if scores[-1] > 0.2:  # Wake word detected
            print(f"Wake word detected for model {mdl}!")

# Create an audio stream
with sd.InputStream(device=DEVICE, channels=CHANNELS, samplerate=RATE, dtype='int16', callback=audio_callback):
    print("Listening for wake word...")
    input("Press Enter to stop...")  # Keep the script running until Enter is pressed
