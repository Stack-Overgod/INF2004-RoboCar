"""RoboCar dashboard: local web UI for the MQTT broker, commands and telemetry.

Stdlib only (no installs). Runs scripts/mqtt_broker.py as a child process,
reads its stdout (every PUBLISH and every connect/disconnect is printed
there), keeps the latest value of each topic and a ring of recent events,
and streams them to the browser with Server-Sent Events.

    python -I test\\dashboard\\dashboard.py            # then open http://127.0.0.1:8080

HTTP endpoints (127.0.0.1 only):
    GET  /                    the dashboard page
    GET  /api/events          SSE stream of parsed lines
    GET  /api/snapshot        JSON: broker, car link, latest per topic, last events
    GET  /api/log?n=200       last n raw broker lines (plain text)
    POST /api/broker/start    start the broker (log -> test/wifi/mqtt_run_<stamp>.log)
    POST /api/broker/stop     stop it
    POST /api/cmd?c=G         publish one character on car/01/cmd
"""
import argparse
import collections
import datetime
import json
import os
import queue
import re
import socket
import subprocess
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, "..", ".."))
BROKER_PY = os.path.join(REPO, "scripts", "mqtt_broker.py")
LOG_DIR = os.path.join(REPO, "test", "wifi")
PAGE = os.path.join(HERE, "index.html")

MQTT_PORT = 1883
CMD_TOPIC = "car/01/cmd"
CAR_CLIENT_ID = "robocar-01"
CMD_LETTERS = "GXPLRSUABCDZ"

LINE_RE = re.compile(r"^(\d\d:\d\d:\d\d\.\d{3}) (.*)$")
CONN_RE = re.compile(r"^\[broker\] connection from ([\d.]+):(\d+)")
DISC_RE = re.compile(r"^\[broker\] ([\d.]+) disconnected")
CID_RE = re.compile(r"^\[broker\] CONNECT client_id=(\S+)")


class Hub:
    """Shared state between the broker reader, the HTTP handlers and SSE clients."""

    def __init__(self):
        self.lock = threading.Lock()
        self.proc = None
        self.log_path = None
        self.started_at = None
        self.raw = collections.deque(maxlen=2000)       # raw broker lines
        self.events = collections.deque(maxlen=500)     # parsed non-state events
        self.latest = {}                                # topic leaf -> {t, data}
        self.states = collections.deque(maxlen=240)     # last ~60 s of car/01/state, with epoch
        self.car = {"connected": False, "ip": None, "since": None, "last_rx": None}
        # Open broker connections, oldest first: {"ip", "port", "cid"}. The broker
        # names disconnects by IP only, so a disconnect closes the one connection
        # from that IP that is NOT the car if there is one (command senders are
        # short-lived), else the car's.
        self.conns = []
        self.ack_cv = threading.Condition()
        self.acks = collections.deque(maxlen=20)        # (time, rx) from car/01/ack
        self.subscribers = []                           # SSE queues

    # -- broker process ----------------------------------------------------
    def start_broker(self):
        with self.lock:
            if self.proc and self.proc.poll() is None:
                return False, "broker already running"
            if port_busy(MQTT_PORT):
                return False, f"port {MQTT_PORT} is in use (another broker running?)"
            os.makedirs(LOG_DIR, exist_ok=True)
            stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M")
            self.log_path = os.path.join(LOG_DIR, f"mqtt_run_{stamp}.log")
            self.proc = subprocess.Popen(
                [sys.executable, "-I", "-u", BROKER_PY, "--log", self.log_path],
                stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                text=True, encoding="utf-8", errors="replace", bufsize=1)
            self.started_at = time.time()
            self.car.update(connected=False, ip=None, since=None)
            self.conns = []
            threading.Thread(target=self._read, args=(self.proc,), daemon=True).start()
        self.publish({"kind": "broker", "running": True, "log": self.log_path})
        return True, self.log_path

    def stop_broker(self):
        with self.lock:
            p = self.proc
            self.proc = None
        if not p or p.poll() is not None:
            return False, "broker not running"
        p.terminate()
        try:
            p.wait(timeout=3)
        except subprocess.TimeoutExpired:
            p.kill()
        self.car.update(connected=False, since=None)
        self.publish({"kind": "broker", "running": False, "log": self.log_path})
        return True, "stopped"

    def broker_running(self):
        return bool(self.proc and self.proc.poll() is None)

    # -- parsing -----------------------------------------------------------
    def _read(self, proc):
        for line in proc.stdout:
            self.ingest(line.rstrip("\r\n"))
        self.publish({"kind": "broker", "running": False, "log": self.log_path,
                      "note": f"broker exited (code {proc.poll()})"})

    def ingest(self, line):
        self.raw.append(line)
        m = LINE_RE.match(line)
        if not m:
            return
        stamp, rest = m.groups()
        if rest.startswith("[broker]"):
            self._broker_line(stamp, rest)
            return
        topic, _, payload = rest.partition(" ")
        leaf = topic.split("/", 2)[-1] if topic.startswith("car/") else topic
        try:
            data = json.loads(payload)
        except ValueError:
            data = payload
        if leaf != "cmd":
            self.car["last_rx"] = time.time()
        if leaf == "ack" and isinstance(data, dict):
            with self.ack_cv:
                self.acks.append((time.time(), data.get("rx"), data.get("cmd")))
                self.ack_cv.notify_all()
        msg = {"kind": "msg", "t": stamp, "leaf": leaf, "data": data}
        self.latest[leaf] = {"t": stamp, "data": data}
        if leaf == "state":
            self.states.append({"epoch": time.time(), "t": stamp, "data": data})
        else:
            self.events.append(msg)
        self.publish(msg)

    def _broker_line(self, stamp, rest):
        m = CONN_RE.match(rest)
        if m:
            self.conns.append({"ip": m.group(1), "port": m.group(2), "cid": None})
        m = CID_RE.match(rest)
        if m:
            for c in reversed(self.conns):              # the newest unnamed connection
                if c["cid"] is None:
                    c["cid"] = m.group(1)
                    break
        m = DISC_RE.match(rest)
        if m:
            same_ip = [c for c in self.conns if c["ip"] == m.group(1)]
            others = [c for c in same_ip if c["cid"] != CAR_CLIENT_ID]
            gone = (others or same_ip or [None])[0]
            if gone:
                self.conns.remove(gone)
        car_conn = next((c for c in self.conns if c["cid"] == CAR_CLIENT_ID), None)
        if car_conn and not self.car["connected"]:
            self.car.update(connected=True, ip=car_conn["ip"], since=time.time())
        elif not car_conn and self.car["connected"]:
            self.car.update(connected=False, since=None)
        ev = {"kind": "link", "t": stamp, "text": rest[9:], "car": dict(self.car)}
        self.events.append(ev)
        self.publish(ev)

    def wait_ack(self, letter, since, timeout=2.0):
        """True if car/01/ack echoed `letter` after `since` within `timeout` s."""
        deadline = time.time() + timeout
        with self.ack_cv:
            while True:
                for t, rx, cmd in self.acks:
                    if t >= since and rx == letter:
                        return True, cmd
                left = deadline - time.time()
                if left <= 0:
                    return False, None
                self.ack_cv.wait(left)

    # -- SSE fan-out ---------------------------------------------------------
    def publish(self, ev):
        for q in list(self.subscribers):
            try:
                q.put_nowait(ev)
            except queue.Full:
                pass

    def snapshot(self, n_events=100):
        return {
            "now": datetime.datetime.now().strftime("%H:%M:%S"),
            "broker": {"running": self.broker_running(), "log": self.log_path,
                       "started_at": self.started_at},
            "conns": list(self.conns),
            "car": dict(self.car,
                        last_rx_age_s=(round(time.time() - self.car["last_rx"], 1)
                                       if self.car["last_rx"] else None)),
            "latest": self.latest,
            "events": list(self.events)[-n_events:],
            "states": list(self.states),
        }


def port_busy(port):
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.settimeout(0.3)
        return s.connect_ex(("127.0.0.1", port)) == 0


def send_cmd(letter):
    """Same wire format as scripts/send_cmd.py: CONNECT, PUBLISH QoS 0, DISCONNECT."""
    def utf8(s):
        b = s.encode()
        return len(b).to_bytes(2, "big") + b
    with socket.create_connection(("127.0.0.1", MQTT_PORT), timeout=3) as s:
        body = utf8("MQTT") + b"\x04\x02\x00\x3c" + utf8("dashboard")
        s.sendall(b"\x10" + bytes([len(body)]) + body)
        if s.recv(4)[:2] != b"\x20\x02":
            raise OSError("broker refused the connection")
        body = utf8(CMD_TOPIC) + letter.encode()
        s.sendall(b"\x30" + bytes([len(body)]) + body)
        s.sendall(b"\xe0\x00")


hub = Hub()


class Handler(BaseHTTPRequestHandler):
    def log_message(self, fmt, *args):   # keep the console quiet
        pass

    def _send(self, code, body, ctype="application/json"):
        data = body.encode("utf-8") if isinstance(body, str) else body
        self.send_response(code)
        self.send_header("Content-Type", ctype + "; charset=utf-8")
        self.send_header("Content-Length", str(len(data)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(data)

    def _json(self, code, obj):
        self._send(code, json.dumps(obj))

    def do_GET(self):
        url = urlparse(self.path)
        q = parse_qs(url.query)
        if url.path in ("/", "/index.html"):
            with open(PAGE, "rb") as f:
                self._send(200, f.read(), "text/html")
        elif url.path == "/api/snapshot":
            n = int(q.get("n", ["100"])[0])
            self._json(200, hub.snapshot(n))
        elif url.path == "/api/log":
            n = int(q.get("n", ["200"])[0])
            self._send(200, "\n".join(list(hub.raw)[-n:]) + "\n", "text/plain")
        elif url.path == "/api/events":
            self._sse()
        else:
            self._send(404, '{"error":"not found"}')

    def do_POST(self):
        url = urlparse(self.path)
        q = parse_qs(url.query)
        if url.path == "/api/broker/start":
            ok, info = hub.start_broker()
            self._json(200 if ok else 409, {"ok": ok, "info": info})
        elif url.path == "/api/broker/stop":
            ok, info = hub.stop_broker()
            self._json(200 if ok else 409, {"ok": ok, "info": info})
        elif url.path == "/api/cmd":
            c = q.get("c", [""])[0][:1].upper()
            if not c or c not in CMD_LETTERS:
                self._json(400, {"ok": False, "info": f"unknown command '{c}'"})
                return
            if not hub.broker_running():
                self._json(409, {"ok": False, "info": "broker not running: start it first"})
                return
            t0 = time.time()
            try:
                send_cmd(c)
            except OSError as e:
                self._json(503, {"ok": False, "info": f"send failed: {e}"})
                return
            acked, cmd = hub.wait_ack(c, t0)
            if acked:
                info = f"{c} acknowledged by the car" + ("" if cmd else " (ignored: not a command now)")
            elif not hub.car["connected"]:
                info = f"{c} sent, but the car is not connected"
            else:
                info = f"{c} sent, no ack from the car within 2 s"
            self._json(200, {"ok": True, "acked": acked, "info": info})
        else:
            self._send(404, '{"error":"not found"}')

    def _sse(self):
        self.send_response(200)
        self.send_header("Content-Type", "text/event-stream")
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        q = queue.Queue(maxsize=1000)
        hub.subscribers.append(q)
        try:
            self.wfile.write(f"data: {json.dumps({'kind': 'hello', **hub.snapshot(200)})}\n\n".encode())
            self.wfile.flush()
            while True:
                try:
                    ev = q.get(timeout=15)
                    self.wfile.write(f"data: {json.dumps(ev)}\n\n".encode())
                except queue.Empty:
                    self.wfile.write(b": keepalive\n\n")
                self.wfile.flush()
        except (ConnectionError, OSError):
            pass
        finally:
            hub.subscribers.remove(q)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=8080)
    ap.add_argument("--start-broker", action="store_true", help="start the broker at launch")
    a = ap.parse_args()
    if a.start_broker:
        print(hub.start_broker()[1])
    # No SO_REUSEADDR: on Windows it lets a second dashboard bind the same
    # port and requests then land on either one (2026-10-29).
    ThreadingHTTPServer.allow_reuse_address = False
    try:
        srv = ThreadingHTTPServer(("127.0.0.1", a.port), Handler)
    except OSError:
        hub.stop_broker()
        raise SystemExit(f"port {a.port} is in use: a dashboard is already running "
                         f"(open http://127.0.0.1:{a.port}) or pass --port")
    srv.daemon_threads = True
    print(f"dashboard on http://127.0.0.1:{a.port}  (Ctrl+C to quit; stops the broker too)")
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        hub.stop_broker()


if __name__ == "__main__":
    main()
