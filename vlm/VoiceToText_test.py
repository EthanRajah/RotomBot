import mlx_whisper as whisper
import pyaudio
import wave
import threading
from pynput import keyboard
import datetime
import time
import sys
import os

# Audio setup
CHUNK = 1024
FORMAT = pyaudio.paInt16
CHANNELS = 1
RATE = 16000
audio_file = "recorded_audio.wav"

p = pyaudio.PyAudio()
running = False
frames = []
exit_flag = threading.Event()  # Use an event for clean termination


def record_audio():
    global running, frames
    try:
        stream = p.open(format=FORMAT, channels=CHANNELS, rate=RATE, input=True, frames_per_buffer=CHUNK)
        frames = []

        print("Recording started... Press 'q' to stop.")

        while running and not exit_flag.is_set():
            data = stream.read(CHUNK)
            frames.append(data)

        print("Recording stopped. Saving file...")

        # Save recorded audio
        wf = wave.open(audio_file, 'wb')
        wf.setnchannels(CHANNELS)
        wf.setsampwidth(p.get_sample_size(FORMAT))
        wf.setframerate(RATE)
        wf.writeframes(b''.join(frames))
        wf.close()

        stream.stop_stream()
        stream.close()

        # Only transcribe if we're not exiting
        if not exit_flag.is_set():
            print("Audio saved. Now transcribing...")

            # Transcribe the audio
            result = whisper.transcribe(audio_file, path_or_hf_repo="mlx-community/whisper-large-v3-mlx")
            text_output = result["text"]

            print("Transcription:", text_output)

            # Save transcription to a file with a timestamp
            timestamp = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
            filename = f"transcription_{timestamp}.txt"

            with open(filename, "w") as f:
                f.write(text_output + "\n")

            print(f"Transcription saved to '{filename}'.")
    except Exception as e:
        print(f"Error in recording thread: {e}")


# Event handlers for key presses
def on_press(key):
    global running, record_thread

    try:
        if key.char == 's' and not running and not exit_flag.is_set():
            running = True
            record_thread = threading.Thread(target=record_audio, daemon=True)
            record_thread.start()

        elif key.char == 'q' and running and not exit_flag.is_set():
            running = False

    except AttributeError:
        if key == keyboard.Key.esc:
            running = False
            exit_flag.set()  # Signal all threads to terminate
            print("Exiting...")

            # Clean up PyAudio
            try:
                p.terminate()
            except:
                pass

            # Force program termination after a short delay
            def force_exit():
                time.sleep(0.5)  # Give threads time to clean up
                os._exit(0)  # Force exit

            threading.Thread(target=force_exit, daemon=True).start()


# Start listening for keyboard input in a separate thread
listener = keyboard.Listener(on_press=on_press)
listener.start()

print("Press 's' to start recording, 'q' to stop and transcribe, and 'esc' to exit.")

# Keep the main thread alive
try:
    while not exit_flag.is_set():
        time.sleep(0.1)
except KeyboardInterrupt:
    print("Program interrupted")
    exit_flag.set()
finally:
    # Make sure PyAudio is terminated
    try:
        p.terminate()
        print("PyAudio terminated")
    except:
        pass

    # Final forced exit to ensure all threads are terminated
    time.sleep(0.5)  # Give threads time to clean up
    os._exit(0)