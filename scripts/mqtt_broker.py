"""Minimal MQTT 3.1.1 broker + logger for the RoboCar (no installs needed).

Handles CONNECT, PUBLISH (QoS 0/1), SUBSCRIBE, UNSUBSCRIBE, PINGREQ and
DISCONNECT. Every PUBLISH is logged (time, topic, payload) to stdout and to
--log, and forwarded at QoS 0 to clients whose filter matches (exact, '+'
and '#'), so a second client can send car/01/cmd to the car.

    python -I mqtt_broker.py --log D:\\...\\test\\wifi\\mqtt.log [--secs 600]
"""
import argparse
import asyncio
import datetime
import sys

clients = {}  # writer -> list of topic filters
log_file = None


def log(line):
    stamp = datetime.datetime.now().strftime("%H:%M:%S.%f")[:-3]
    text = f"{stamp} {line}"
    print(text, flush=True)
    if log_file:
        log_file.write(text + "\n")
        log_file.flush()


def matches(filt, topic):
    f, t = filt.split("/"), topic.split("/")
    for i, part in enumerate(f):
        if part == "#":
            return True
        if i >= len(t) or (part != "+" and part != t[i]):
            return False
    return len(f) == len(t)


def encode_len(n):
    out = bytearray()
    while True:
        b, n = n % 128, n // 128
        out.append(b | (0x80 if n else 0))
        if not n:
            return bytes(out)


def utf8(b, i):
    n = int.from_bytes(b[i:i + 2], "big")
    return b[i + 2:i + 2 + n].decode("utf-8", "replace"), i + 2 + n


async def read_packet(reader):
    head = await reader.readexactly(1)
    mult, length = 1, 0
    while True:
        b = (await reader.readexactly(1))[0]
        length += (b & 0x7F) * mult
        if not b & 0x80:
            break
        mult *= 128
    body = await reader.readexactly(length) if length else b""
    return head[0], body


async def forward(topic, payload, sender):
    tb = topic.encode()
    pkt_body = len(tb).to_bytes(2, "big") + tb + payload
    pkt = bytes([0x30]) + encode_len(len(pkt_body)) + pkt_body
    for w, filters in list(clients.items()):
        if w is not sender and any(matches(f, topic) for f in filters):
            try:
                w.write(pkt)
                await w.drain()
            except (ConnectionError, OSError):
                pass


async def handle(reader, writer):
    peer = writer.get_extra_info("peername")
    clients[writer] = []
    log(f"[broker] connection from {peer[0]}:{peer[1]}")
    try:
        while True:
            first, body = await read_packet(reader)
            ptype = first >> 4
            if ptype == 1:                                   # CONNECT
                cid = "?"
                try:
                    _, i = utf8(body, 0)                     # "MQTT"
                    cid, _ = utf8(body, i + 4)               # level, flags, keepalive
                except Exception:
                    pass
                log(f"[broker] CONNECT client_id={cid}")
                writer.write(b"\x20\x02\x00\x00")
            elif ptype == 3:                                 # PUBLISH
                qos = (first >> 1) & 3
                topic, i = utf8(body, 0)
                if qos:
                    pid = body[i:i + 2]
                    i += 2
                    writer.write(b"\x40\x02" + pid)          # PUBACK
                payload = body[i:]
                log(f"{topic} {payload.decode('utf-8', 'replace')}")
                await forward(topic, payload, writer)
            elif ptype == 8:                                 # SUBSCRIBE
                pid, i, granted = body[0:2], 2, bytearray()
                while i < len(body):
                    filt, i = utf8(body, i)
                    i += 1
                    clients[writer].append(filt)
                    granted.append(0)
                    log(f"[broker] SUBSCRIBE {filt}")
                writer.write(bytes([0x90]) + encode_len(2 + len(granted)) + pid + granted)
            elif ptype == 10:                                # UNSUBSCRIBE
                writer.write(b"\xb0\x02" + body[0:2])
            elif ptype == 12:                                # PINGREQ
                writer.write(b"\xd0\x00")
            elif ptype == 14:                                # DISCONNECT
                break
            await writer.drain()
    except (asyncio.IncompleteReadError, ConnectionError, OSError):
        pass
    finally:
        clients.pop(writer, None)
        log(f"[broker] {peer[0]} disconnected")
        writer.close()


async def main(args):
    server = await asyncio.start_server(handle, "0.0.0.0", args.port)
    log(f"[broker] listening on 0.0.0.0:{args.port}")
    async with server:
        if args.secs > 0:
            await asyncio.sleep(args.secs)
            log("[broker] time limit reached, stopping")
        else:
            await server.serve_forever()


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=1883)
    ap.add_argument("--log", default=None)
    ap.add_argument("--secs", type=int, default=0, help="stop after N s (0 = run forever)")
    a = ap.parse_args()
    if a.log:
        log_file = open(a.log, "a", encoding="utf-8")
    try:
        asyncio.run(main(a))
    except KeyboardInterrupt:
        sys.exit(0)
