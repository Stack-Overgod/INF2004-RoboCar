"""Send one RoboCar command over MQTT (no installs needed).

    python -I send_cmd.py X                 # STOP, broker on this laptop
    python -I send_cmd.py L --host 172.20.10.3
"""
import argparse
import socket


def utf8(s):
    b = s.encode()
    return len(b).to_bytes(2, "big") + b


ap = argparse.ArgumentParser()
ap.add_argument("cmd", help="L/A left, R/B right, S/C straight, U/D u-turn, X/P stop")
ap.add_argument("--host", default="127.0.0.1")
ap.add_argument("--port", type=int, default=1883)
ap.add_argument("--topic", default="car/01/cmd")
a = ap.parse_args()

with socket.create_connection((a.host, a.port), timeout=5) as s:
    body = utf8("MQTT") + b"\x04\x02\x00\x3c" + utf8("cmd-sender")   # CONNECT, clean session
    s.sendall(b"\x10" + bytes([len(body)]) + body)
    if s.recv(4)[:2] != b"\x20\x02":
        raise SystemExit("broker did not accept the connection")
    body = utf8(a.topic) + a.cmd[:1].encode()                          # PUBLISH, QoS 0
    s.sendall(b"\x30" + bytes([len(body)]) + body)
    s.sendall(b"\xe0\x00")                                             # DISCONNECT
print(f"sent '{a.cmd[:1]}' to {a.topic} via {a.host}:{a.port}")