import sounddevice as sd
import numpy as np

def record_and_playback(device_index, duration=5):
    device_info = sd.query_devices(device_index, 'input')
    fs = int(device_info['default_samplerate'])  # Use the device's default sample rate

    print(f"Recording from device {device_index} for {duration} seconds with sample rate {fs}...")
    recording = sd.rec(int(duration * fs), samplerate=fs, channels=1, device=device_index)
    sd.wait()  # Wait for the recording to finish

    print("Playing back recording...")
    sd.play(recording, fs)
    sd.wait()  # Wait for the playback to finish

def main():
    mic_device_index = 5  # Replace with your microphone's device index
    record_and_playback(mic_device_index)

if __name__ == "__main__":
    main()
