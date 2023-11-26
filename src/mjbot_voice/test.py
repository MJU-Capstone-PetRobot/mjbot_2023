import sounddevice as sd
import numpy as np
import soundfile as sf
from pydub import AudioSegment

def main():
    device_index = 5
    duration = 5  # seconds
    device_info = sd.query_devices(device_index, 'input')
    samplerate = int(device_info['default_samplerate'])
    channels = 2
    filename = "recording.wav"
    mp3_filename = "recording.mp3"

    # Prepare an array to store the recorded data
    recording = np.zeros((int(samplerate * duration), channels))

    def callback(indata, frames, time, status):
        if status:
            print("Status:", status)
        recording[:len(indata)] = indata  # Store recorded data in the array

    with sd.InputStream(device=device_index, channels=channels, samplerate=samplerate, callback=callback):
        print('#' * 80)
        print('Recording... Press Return to stop')
        print('#' * 80)
        input()

    # Save the recording as a WAV file
    sf.write(filename, recording, samplerate)

    # Convert to MP3
    
    audio = AudioSegment.from_wav(filename)
    audio.export(mp3_filename, format="mp3")
    print(f"Recording saved as {mp3_filename}")

    # Optional: Remove the WAV file
    # os.remove(filename)

if __name__ == "__main__":
    main()

