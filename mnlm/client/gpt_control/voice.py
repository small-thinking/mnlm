import audioop
import os
import time
import wave

import pyaudio
import pygame
from dotenv import load_dotenv
from openai import OpenAI


def record_audio(
    filename: str = "temp_audio.mp3",
    silence_threshold: int = 300,
    silence_duration: int = 2,
    silence_proportion_threshold: float = 0.9,
    verbose: bool = False,
) -> str:
    """
    Records audio from the microphone and stops when silence is detected for a specified duration.
    If the recorded audio is mostly silence, it returns an empty string instead of saving the file.

    Parameters:
    filename (str): The name of the file to save the audio.
    silence_threshold (int): The volume level below which is considered silence.
    silence_duration (int): The duration of silence (in seconds) to stop recording.
    verbose (bool): If True, prints verbose output.
    silence_proportion_threshold (float): The proportion of total frames that are silent to consider the recording as predominantly silent.

    Returns:
    str: The filename of the recorded audio or an empty string if mostly silent.
    """
    FORMAT = pyaudio.paInt16
    CHANNELS = 1
    RATE = 44100
    CHUNK = 1024

    p = pyaudio.PyAudio()
    stream = p.open(
        format=FORMAT, channels=CHANNELS, rate=RATE, input=True, frames_per_buffer=CHUNK
    )

    if verbose:
        print("* Recording...")

    frames = []
    silence_start = None
    silent_frames = 0

    while True:
        data = stream.read(CHUNK)
        frames.append(data)

        volume = audioop.rms(data, 2)  # Get the RMS of the frame

        if volume < silence_threshold:
            silent_frames += 1
            if silence_start is None:
                silence_start = time.time()  # Mark the start of the silent period
            elif time.time() - silence_start > silence_duration:
                if verbose:
                    print("* Silence detected, stopping recording")
                break
        else:
            silence_start = None

    stream.stop_stream()
    stream.close()
    p.terminate()

    # Check if the recording is predominantly silent
    if silent_frames / len(frames) > silence_proportion_threshold:
        if verbose:
            print("Recording is mostly silent. No file saved.")
        return ""  # Return empty string for mostly silent recording

    wf = wave.open(filename, "wb")
    wf.setnchannels(CHANNELS)
    wf.setsampwidth(p.get_sample_size(FORMAT))
    wf.setframerate(RATE)
    wf.writeframes(b"".join(frames))
    wf.close()

    if verbose:
        print(f"Audio recorded to {filename}")

    return filename


def transcribe_audio(file_path: str, verbose: bool = False) -> str:
    """
    Transcribes the audio file using OpenAI's Whisper.

    Parameters:
    file_path (str): The path to the audio file to be transcribed.
    verbose (bool): If True, prints verbose output.

    Returns:
    str: The transcribed text.
    """
    client = OpenAI(api_key=os.environ.get("OPENAI_API_KEY"))
    with open(file_path, "rb") as audio_file:
        transcript = client.audio.transcriptions.create(
            model="whisper-1", file=audio_file
        )

    os.remove(file_path)  # Remove the temporary file

    if verbose:
        print("Transcription completed.")

    return transcript.text


def generate_transcription(verbose: bool = True) -> str:
    """
    Runs a test for audio transcription.

    Parameters:
    verbose (bool): If True, prints verbose output.
    """
    recorded_file = record_audio(verbose=verbose)
    if not os.path.exists(recorded_file):
        if verbose:
            print("No audio recorded.")
        return ""
    transcription = transcribe_audio(recorded_file, verbose=verbose)
    if verbose:
        print("Transcribed Text:", transcription)
    return transcription


def speak(text: str, client: OpenAI) -> None:
    # Generate the speech audio using TTS
    response = client.audio.speech.create(
        model="tts-1",
        voice="nova",
        input=text,
    )

    # Stream the audio to a file
    audio_filename = "output.mp3"
    response.stream_to_file(audio_filename)

    # Initialize pygame mixer
    pygame.mixer.init()
    pygame.mixer.music.load(audio_filename)
    pygame.mixer.music.play()

    # Wait for the audio to finish playing
    while pygame.mixer.music.get_busy():
        pygame.time.Clock().tick(10)

    # Delete the mp3 file
    os.remove(audio_filename)


if __name__ == "__main__":
    load_dotenv()
    generate_transcription()
