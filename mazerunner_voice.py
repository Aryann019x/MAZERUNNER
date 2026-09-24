#!/usr/bin/env python3
"""MazeRunner voice link: wait for BTN2 capture, transcribe, ask ollama, reply.

Flow per round:
  bot sends AUDHDR + len + PCM  ->  Vosk -> text
  text -> `ollama run qwen2.5:3b` -> short answer
  answer goes back as TXTHDR + len + "YOU: ...\\nAI: ..." -> bot shows it on OLED.
Runs forever. No WiFi needed for this part, just USB serial.
"""
import argparse
import json
import struct
import subprocess
import sys
import time
import wave

import serial
from vosk import Model, KaldiRecognizer, SetLogLevel

SAMPLE_RATE = 16000


def wait_and_capture(ser, timeout: float) -> bytes:
    print("\nWaiting for BTN2 hold...", flush=True)
    deadline = time.time() + timeout
    buf = b""
    while time.time() < deadline:
        chunk = ser.read(256)
        if not chunk:
            continue
        text = chunk.decode("utf-8", "replace")
        for line in text.replace("\r", "\n").split("\n"):
            line = line.strip()
            if line and any(k in line for k in ("voice:", "AUDEND", "wifi", "AP up", "mic ready")):
                print(f"  [{line}]", flush=True)
        buf += chunk
        idx = buf.find(b"AUDHDR")
        if idx >= 0:
            buf = buf[idx + 6:]
            print("Capture received...", flush=True)
            break
    else:
        raise RuntimeError("Timed out waiting for button")

    end = time.time() + 5
    while len(buf) < 4 and time.time() < end:
        buf += ser.read(4 - len(buf))
    if len(buf) < 4:
        raise RuntimeError("No length header")
    nbytes = struct.unpack("<I", buf[:4])[0]
    data = buf[4:]
    print(f"Reading {nbytes} bytes...", flush=True)

    end = time.time() + (nbytes / (ser.baudrate / 10.0)) + 10
    while len(data) < nbytes and time.time() < end:
        chunk = ser.read(min(4096, nbytes - len(data)))
        if chunk:
            data += chunk
    if len(data) < nbytes:
        raise RuntimeError(f"Short read: {len(data)}/{nbytes}")
    return data[:nbytes]


def save_wav(pcm: bytes, path: str) -> None:
    with wave.open(path, "wb") as w:
        w.setnchannels(1)
        w.setsampwidth(2)
        w.setframerate(SAMPLE_RATE)
        w.writeframes(pcm)


def ask_ollama(question: str, model: str) -> str:
    prompt = ("Answer briefly in at most 3 short sentences. " + question).encode("utf-8")
    try:
        r = subprocess.run(
            ["ollama", "run", model],
            input=prompt,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            timeout=60,
            check=False,
        )
        out = (r.stdout or b"").decode("utf-8", "replace").strip()
        return out if out else "(ollama gave nothing)"
    except Exception as e:
        return f"(ollama offline: {e})"


def send_reply(ser, heard: str, reply: str) -> None:
    payload = f"YOU: {heard}\nAI: {reply}".encode("utf-8")[:512]
    ser.write(b"TXTHDR" + struct.pack("<I", len(payload)) + payload)
    ser.flush()


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="COM3")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--model", default=r"C:\Users\gamar\vosk-model-small-en-us-0.15")
    ap.add_argument("--ollama-model", default="qwen2.5:3b")
    args = ap.parse_args()

    SetLogLevel(-1)
    print("Loading Vosk model...", flush=True)
    model = Model(args.model)
    print("Model loaded.", flush=True)

    ser = serial.Serial(args.port, args.baud, timeout=0.5)
    time.sleep(0.3)
    ser.reset_input_buffer()

    round_n = 0
    while True:
        try:
            pcm = wait_and_capture(ser, timeout=600)
        except Exception as e:
            print(f"ERROR: {e}", flush=True)
            time.sleep(0.5)
            ser.reset_input_buffer()
            continue

        round_n += 1
        wav_path = rf"D:\MAZERUNNER\voice_{round_n}.wav"
        save_wav(pcm, wav_path)

        rec = KaldiRecognizer(model, SAMPLE_RATE)
        rec.AcceptWaveform(pcm)
        text = json.loads(rec.FinalResult()).get("text", "").strip()
        print(f"\n=== TEXT #{round_n}: {text}", flush=True)
        print(f"    (saved {wav_path})", flush=True)

        if not text:
            send_reply(ser, "(nothing heard)", "say it again?")
            continue

        print("Asking ollama...", flush=True)
        answer = ask_ollama(text, args.ollama_model)
        print(f"=== AI #{round_n}: {answer}", flush=True)
        send_reply(ser, text, answer)


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        print("\nStopped.")
        sys.exit(0)
