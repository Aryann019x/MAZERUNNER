from flask import Flask, request, jsonify
import subprocess

app = Flask(__name__)
@app.route("/ask", methods=["POST"])
def ask():
    data = request.get_json(silent=True) or {}
    question = data.get("question", "")
    if not question:
        return jsonify({"response": "No question received."})

    try:
        # Local Ollama model tuned for speed: qwen2.5:3b fits fully in
        # 4GB VRAM and answers in seconds. Prompt forces short replies.
        prompt = "Answer briefly in at most 3 short sentences. " + question
        result = subprocess.run(
            ["ollama", "run", "qwen2.5:3b"],
            input=prompt.encode("utf-8"),
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            timeout=45,
            check=False,
        )

        stdout = (result.stdout or b"").decode("utf-8", errors="replace").strip()
        stderr = (result.stderr or b"").decode("utf-8", errors="replace").strip()

        if stdout:
            reply = stdout
        else:
            reply = f"Error: Ollama returned no output. {stderr}" if stderr else "Error: Ollama returned no output."

    except Exception as e:
        reply = f"Error: {e}"

    return jsonify({"response": str(reply)})

if __name__ == "__main__":
    print("🚀 Server running on http://0.0.0.0:5000/ask")
    app.run(host="0.0.0.0", port=5000)
