"""Laptop side of the MazeRunner link.

Everything runs locally:
  /ask    - plain question -> short ollama answer
  /stt    - POST a wav -> vosk text + ollama answer
  /drive  - sensor state -> one drive action (json)
  /health - is the bridge up

    python server.py
    python server.py --model qwen2.5:3b --vosk C:\\path\\to\\vosk-model
"""

import argparse
import io
import json
import os
import re
import subprocess
import threading
import wave

from flask import Flask, jsonify, request

app = Flask(__name__)

OLLAMA_MODEL = os.environ.get("OLLAMA_MODEL", "qwen2.5:3b")
VOSK_MODEL = os.environ.get(
    "VOSK_MODEL", r"C:\Users\aryan\vosk-model-small-en-us-0.15"
)
_vosk = None

ACTIONS = ("forward", "backward", "left", "right", "stop")


def ask_ollama(prompt: str, timeout: int = 60) -> str:
    try:
        r = subprocess.run(
            ["ollama", "run", OLLAMA_MODEL],
            input=prompt.encode("utf-8"),
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            timeout=timeout,
            check=False,
        )
        return (r.stdout or b"").decode("utf-8", errors="replace").strip()
    except Exception as e:
        print(f"ollama error: {e}", flush=True)
        return ""


def clean(text: str, limit: int = 400) -> str:
    return text.replace('"', "'").replace("\n", " ").strip()[:limit]


def transcribe(wav_bytes: bytes) -> str:
    global _vosk
    if _vosk is None:
        from vosk import Model, SetLogLevel

        SetLogLevel(-1)
        _vosk = Model(VOSK_MODEL)

    from vosk import KaldiRecognizer

    with wave.open(io.BytesIO(wav_bytes), "rb") as w:
        frames = w.readframes(w.getnframes())
        rate = w.getframerate()

    rec = KaldiRecognizer(_vosk, rate)
    rec.AcceptWaveform(frames)
    return json.loads(rec.FinalResult()).get("text", "").strip()


def fallback_action(front, left, right):
    if front < 300:
        if left >= right:
            return "left", 110, "front blocked, going left"
        return "right", 110, "front blocked, going right"
    if left < 150:
        return "right", 120, "left wall close"
    if right < 150:
        return "left", 120, "right wall close"
    return "forward", 140, "path looks clear"


@app.route("/health")
def health():
    return jsonify({"ok": True, "model": OLLAMA_MODEL, "vosk": _vosk is not None})


@app.route("/ask", methods=["POST"])
def ask():
    data = request.get_json(silent=True) or {}
    question = data.get("question", "")
    if not question:
        return jsonify({"response": "No question received."})

    prompt = "Answer briefly in at most 3 short sentences. " + question
    reply = ask_ollama(prompt, timeout=45)
    if not reply:
        reply = "Error: ollama returned nothing."
    return jsonify({"response": clean(reply)})


@app.route("/stt", methods=["POST"])
def stt():
    try:
        text = transcribe(request.get_data())
    except Exception as e:
        return jsonify({"text": "", "reply": f"stt failed: {e}"})

    if not text:
        return jsonify({"text": "", "reply": "say it again?"})

    reply = ask_ollama("Answer briefly in at most 3 short sentences. " + text)
    if not reply:
        reply = "ollama is not answering"
    print(f"heard: {text}\nreply: {reply}", flush=True)
    return jsonify({"text": clean(text), "reply": clean(reply)})


@app.route("/drive", methods=["POST"])
def drive():
    d = request.get_json(silent=True) or {}
    front = float(d.get("front", 999))
    left = float(d.get("left", 999))
    right = float(d.get("right", 999))
    heading = float(d.get("heading", 0))

    prompt = (
        "You drive a small maze robot. Distances in mm, 999 means clear. "
        f"front={front:.0f} left={left:.0f} right={right:.0f} heading={heading:.0f}.\n"
        "Pick one: forward, backward, left, right, stop.\n"
        'Reply with json only: {"action":"forward","speed":140,"thought":"short"}'
    )
    raw = ask_ollama(prompt, timeout=30)

    action = ""
    speed = 140
    thought = ""
    m = re.search(r'"action"\s*:\s*"([a-z]+)"', raw)
    if m:
        action = m.group(1)
    m = re.search(r'"speed"\s*:\s*(\d+)', raw)
    if m:
        speed = int(m.group(1))
    m = re.search(r'"thought"\s*:\s*"([^"]*)"', raw)
    if m:
        thought = m.group(1)

    source = "llm"
    if action not in ACTIONS:
        action, speed, thought = fallback_action(front, left, right)
        source = "fallback"

    if action == "forward" and front < 250:
        action = "left" if left > right else "right"
        thought = "too close in front"
        source += "+safety"

    return jsonify(
        {
            "action": action,
            "speed": max(60, min(220, speed)),
            "thought": clean(thought, 80),
            "source": source,
        }
    )


def warmup():
    """Load vosk and the ollama model so the first robot request is fast."""
    global _vosk
    try:
        from vosk import Model, SetLogLevel

        SetLogLevel(-1)
        _vosk = Model(VOSK_MODEL)
        print("vosk model loaded", flush=True)
    except Exception as e:
        print(f"vosk warmup failed: {e}", flush=True)
    ask_ollama("hi", timeout=120)
    print("ollama warm", flush=True)


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=5000)
    ap.add_argument("--model", default=OLLAMA_MODEL)
    ap.add_argument("--vosk", default=VOSK_MODEL)
    args = ap.parse_args()

    OLLAMA_MODEL = args.model
    VOSK_MODEL = args.vosk
    print(f"bridge on http://0.0.0.0:{args.port}  model={OLLAMA_MODEL}", flush=True)
    threading.Thread(target=warmup, daemon=True).start()
    app.run(host="0.0.0.0", port=args.port)
