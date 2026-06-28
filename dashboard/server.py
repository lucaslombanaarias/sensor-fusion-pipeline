#!/usr/bin/env python3
"""server.py — a tiny live dashboard for the fusion pipeline.

Standard library only (no Flask/Node), in keeping with the rest of the
project. It does two things:

  1. serves the static dashboard (index.html / app.js / style.css), and
  2. at GET /events, launches `sfp --stream` and relays each JSON tick it
     prints to the browser over Server-Sent Events (SSE).

So the data path is:

    sfp (C++) --JSON lines--> this server --SSE--> browser (live charts)

Run it, then open http://localhost:8000 :

    python3 dashboard/server.py                 # auto-finds the sfp binary
    python3 dashboard/server.py --sfp build/sfp --port 8000

Stopping the browser stream terminates the child `sfp` process.
"""

import argparse
import json
import os
import subprocess
import sys
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse, parse_qs

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)

# Files we serve, with content types.
STATIC = {
    "/": ("index.html", "text/html; charset=utf-8"),
    "/index.html": ("index.html", "text/html; charset=utf-8"),
    "/app.js": ("app.js", "application/javascript; charset=utf-8"),
    "/style.css": ("style.css", "text/css; charset=utf-8"),
}

SFP_BIN = None  # resolved in main()


def find_sfp():
    """Locate the sfp binary across the usual build locations."""
    env = os.environ.get("SFP_BIN")
    candidates = [env] if env else []
    candidates += [
        os.path.join(REPO, "build", "Release", "sfp.exe"),  # CMake/MSVC
        os.path.join(REPO, "build", "sfp.exe"),
        os.path.join(REPO, "build", "sfp"),                 # Makefile/Linux
    ]
    for c in candidates:
        if c and os.path.exists(c):
            return c
    return None


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *args):
        pass  # keep the console quiet

    def do_GET(self):
        path = urlparse(self.path).path
        if path == "/events":
            self.serve_events()
        elif path in STATIC:
            self.serve_static(*STATIC[path])
        else:
            self.send_error(404)

    def serve_static(self, filename, content_type):
        try:
            with open(os.path.join(HERE, filename), "rb") as f:
                body = f.read()
        except OSError:
            self.send_error(404)
            return
        self.send_response(200)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def serve_events(self):
        q = parse_qs(urlparse(self.path).query)
        config = q.get("config", ["robotics"])[0]
        filt = q.get("filter", ["kalman"])[0]
        duration = q.get("duration", ["86400"])[0]  # ~forever until closed

        args = [SFP_BIN, "--config", config, "--duration", duration, "--stream"]
        if filt == "kalman":
            args.append("--kalman")
        elif filt == "none":
            args.append("--no-filter")
        # "complementary" is the robotics default — no extra flag.

        self.send_response(200)
        self.send_header("Content-Type", "text/event-stream")
        self.send_header("Cache-Control", "no-cache")
        self.send_header("Connection", "keep-alive")
        self.send_header("Access-Control-Allow-Origin", "*")
        self.end_headers()

        proc = subprocess.Popen(
            args, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
            bufsize=1, text=True)
        try:
            # Tell the client the run parameters once, up front.
            meta = json.dumps({"config": config, "filter": filt})
            self.wfile.write(f"event: meta\ndata: {meta}\n\n".encode())
            self.wfile.flush()
            for line in proc.stdout:
                line = line.strip()
                if not line:
                    continue
                self.wfile.write(f"data: {line}\n\n".encode())
                self.wfile.flush()
        except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError):
            pass  # browser closed the stream (10053/10054 on Windows)
        finally:
            proc.terminate()
            try:
                proc.wait(timeout=2)
            except subprocess.TimeoutExpired:
                proc.kill()


def main():
    global SFP_BIN
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--sfp", help="path to the sfp binary")
    ap.add_argument("--port", type=int, default=8000)
    args = ap.parse_args()

    SFP_BIN = args.sfp or find_sfp()
    if not SFP_BIN or not os.path.exists(SFP_BIN):
        sys.exit("error: could not find the sfp binary. Build it first "
                 "(make / cmake) or pass --sfp <path>.")
    # Absolute path so subprocess spawning is independent of cwd (and so
    # Windows CreateProcess resolves it reliably).
    SFP_BIN = os.path.abspath(SFP_BIN)

    srv = ThreadingHTTPServer(("127.0.0.1", args.port), Handler)
    print(f"sfp dashboard: http://localhost:{args.port}  (sfp: {SFP_BIN})")
    print("Ctrl-C to stop.")
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        print("\nbye")


if __name__ == "__main__":
    main()
