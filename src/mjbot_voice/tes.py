from playsound import playsound

def play_test_sound():
    try:
        print("Playing 'test.wav'...")
        playsound("test.wav")
    except Exception as e:
        print(f"Error occurred while playing sound: {e}")

def main():
    # Call the function to play the sound
    play_test_sound()

if __name__ == "__main__":
    main()
