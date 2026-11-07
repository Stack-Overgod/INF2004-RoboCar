"""Decoder for the RoboCar's binary MQTT telemetry.

The car (app_program/subsystem1/tm_frame.c) sends every message as

    byte 0   [ver:3 | type:5]
    byte 1   seq (8 bits)
    byte 2.. fields packed back to back, least significant bit first,
             each in exactly the bits its range needs (signed = two's
             complement in that width)

FRAMES below is the same schema as the car's table; keep the two in step
and bump VERSION with TM_VER. decode() turns a frame into a JSON dict with
readable values (letters, "10" line bits, ...), which the broker logs and
the dashboard reads.

Stdlib only. Used by scripts/mqtt_broker.py (log lines) and
dashboard/fake_car.py (encode).

    python -I scripts/tm_frames.py 2101e8030000...   # decode one hex frame
"""
import json
import sys

VERSION = 2                             # = TM_VER in tm_frame.h

# (key, bits, kind). kinds: u, s (signed), line2, sym, sym3, char, flags4, sink
_STATE = [
    ("t", 32, "u"), ("spd_l", 11, "s"), ("spd_r", 11, "s"), ("dist", 16, "u"),
    ("m_l", 11, "s"), ("m_r", 11, "s"), ("line", 2, "line2"), ("cls", 3, "u"),
    ("peak", 7, "u"), ("rng", 12, "u"), ("bc", 5, "sym"), ("nav", 3, "u"),
    ("flt", 8, "u"), ("h", 10, "s"),
]

FRAMES = {
    1: ("state", True, _STATE),
    2: ("state", True, _STATE + [("enc_l", 32, "u"), ("enc_r", 32, "u")]),
    3: ("status", False, [
        ("up", 32, "u"), ("tx", 24, "u"), ("fail", 16, "u"),
        ("drop_fast", 16, "u"), ("drop_slow", 16, "u"), ("sink", 2, "sink"),
        ("wifi", 1, "u"), ("mqtt", 1, "u"), ("cmds", 16, "u"),
        ("retries", 16, "u"), ("live", 4, "flags4")]),
    4: ("barcode", False, [("sym", 5, "sym"), ("cmd", 3, "u"), ("rev", 1, "u")]),
    5: ("bcscan", False, [
        ("txt", 15, "sym3"), ("ok", 8, "u"), ("edges", 8, "u"),
        ("f_ratio", 8, "u"), ("f_wide", 8, "u"), ("f_nomatch", 8, "u"),
        ("f_glitch", 8, "u"), ("pat", 9, "u"), ("min_10us", 16, "u"),
        ("max_10us", 16, "u")]),
    6: ("bcbits", False, [("n", 7, "u"), ("first", 1, "u")]
        + [("b%d" % i, 8, "u") for i in range(8)]
        + [("nar_10us", 16, "u"), ("wid_10us", 16, "u"), ("more", 8, "u")]),
    7: ("spin", False, [("face", 12, "u"), ("n", 8, "u"), ("found", 4, "u")]
        + sum([[("d%d" % i, 12, "s"), ("r%d" % i, 8, "u")] for i in range(4)], [])),
    8: ("hump", False, [("peak_mm", 7, "u"), ("ms", 13, "u"), ("pitch", 8, "s")]),
    9: ("obstacle", False, [
        ("n", 6, "u"), ("near_mm", 12, "u"), ("at_deg", 8, "u"),
        ("w_mm", 12, "u"), ("cl_l", 10, "u"), ("cl_r", 10, "u")]),
    10: ("avoid", False, [("act", 3, "u"), ("lat_mm", 12, "u"), ("fwd_mm", 12, "u")]),
    11: ("impact", False, [("impact", 1, "u")]),
    12: ("motion", False, [("cls", 3, "u")]),
    13: ("line", False, [("lost", 1, "u")]),
    14: ("move", False, [("id", 15, "u"), ("ok", 1, "u"), ("mm", 16, "u")]),
    15: ("ack", False, [("rx", 8, "char"), ("cmd", 3, "u")]),
    16: ("ramp", False, [("rise_mm", 9, "u"), ("net_mm", 10, "s"),
                         ("deg", 6, "u"), ("len_mm", 12, "u")]),
}

NAV_STATES = ["IDLE", "FOLLOWING", "READING_BARCODE", "EXECUTING_CMD",
              "AVOIDING", "RECOVERING", "STOPPED"]
FAULTS = ["MOTOR_STALL", "ENCODER", "IMU", "NO_ECHO", "WIFI_LOSS", "IMPACT",
          "BARCODE_ADC"]
SINKS = ["console", "mqtt", "other", "other"]


def frame_len(type_):
    """Bytes on the wire for a frame type, or None."""
    f = FRAMES.get(type_)
    return None if f is None else 2 + (sum(b for _, b, _ in f[2]) + 7) // 8


def sym_char(code):
    if code == 1:
        return "*"
    if 2 <= code <= 27:
        return chr(ord("A") + code - 2)
    return ""


def unpack(buf):
    """(type, seq, {key: int}) or None if it is not a valid frame."""
    if len(buf) < 2 or (buf[0] >> 5) != VERSION:
        return None
    t = buf[0] & 0x1F
    if t not in FRAMES or len(buf) != frame_len(t):
        return None
    bits = int.from_bytes(bytes(buf[2:]), "little")
    pos = 0
    out = {}
    for key, n, kind in FRAMES[t][2]:
        v = (bits >> pos) & ((1 << n) - 1)
        pos += n
        if kind == "s" and v & (1 << (n - 1)):
            v -= 1 << n
        out[key] = v
    return t, buf[1], out


def decode(buf):
    """(leaf, dict) with readable values, or None."""
    r = unpack(buf)
    if r is None:
        return None
    t, seq, v = r
    leaf, show_seq, _ = FRAMES[t]
    d = {"seq": seq} if show_seq else {}

    if leaf == "state":
        d.update(t=v["t"], spd_l=v["spd_l"], spd_r=v["spd_r"], dist=v["dist"])
        if "enc_l" in v:
            d.update(enc_l=v["enc_l"], enc_r=v["enc_r"])
        d.update(m_l=v["m_l"], m_r=v["m_r"],
                 line="%d%d" % (v["line"] & 1, (v["line"] >> 1) & 1),
                 cls=v["cls"], peak=v["peak"], rng=v["rng"],
                 bc=sym_char(v["bc"]) or "-", nav=v["nav"], flt=v["flt"],
                 h=v["h"])
    elif leaf == "status":
        d.update(up=v["up"], tx=v["tx"], fail=v["fail"],
                 drop_fast=v["drop_fast"], drop_slow=v["drop_slow"],
                 sink=SINKS[v["sink"]], wifi=v["wifi"], mqtt=v["mqtt"],
                 cmds=v["cmds"], retries=v["retries"],
                 live="".join("1" if v["live"] >> i & 1 else "0" for i in range(4)))
    elif leaf == "barcode":
        d.update(sym=sym_char(v["sym"]) or "-", cmd=v["cmd"], rev=v["rev"])
    elif leaf == "bcscan":
        txt = ""
        for i in range(3):
            c = sym_char((v["txt"] >> (5 * i)) & 0x1F)
            if not c:
                break
            txt += c
        pat = "".join("w" if v["pat"] & (1 << (8 - b)) else "n" for b in range(9)) \
            if v["f_nomatch"] else "-"
        d.update(res="ok" if v["ok"] else "fail", txt=txt, ok=v["ok"],
                 edges=v["edges"], f_ratio=v["f_ratio"], f_wide=v["f_wide"],
                 f_nomatch=v["f_nomatch"], f_glitch=v["f_glitch"], pat=pat,
                 min_us=v["min_10us"] * 10, max_us=v["max_10us"] * 10)
    elif leaf == "bcbits":
        raw = [v["b%d" % i] for i in range(8)]
        s = ""
        n = min(v["n"], 64)                 # the car never sends more
        for i in range(n):
            if i % 10 == 9:
                s += " "
            s += "1" if raw[i // 8] >> (i % 8) & 1 else "0"
            if i % 10 == 9 and i + 1 < n:
                s += " "
        d.update(n=v["n"], first="bar" if v["first"] else "space", bits=s,
                 nar_us=v["nar_10us"] * 10, wid_us=v["wid_10us"] * 10,
                 more=v["more"])
    elif leaf == "spin":
        d.update(face=v["face"], n=v["n"])
        for i, k in enumerate(("l0", "l1", "r1", "r2")):
            d[k] = "%d/%d" % (v["d%d" % i], v["r%d" % i] * 2) \
                if v["found"] >> i & 1 else "-"
    elif leaf == "ack":
        c = v["rx"]
        d.update(rx=chr(c) if 32 < c < 127 and c not in (34, 92) else "?",
                 cmd=v["cmd"])
    else:
        d.update(v)                     # names and values are already right
    return leaf, d


def sym_code(c):
    if c == "*":
        return 1
    if c and "A" <= c <= "Z":
        return 2 + ord(c) - ord("A")
    return 0


def pack(type_, seq, vals):
    """Frame bytes from {key: int}, clamping like the car (tm_pack)."""
    _, _, fields = FRAMES[type_]
    bits = pos = 0
    for key, n, kind in fields:
        v = int(vals.get(key, 0))
        if kind == "s":
            lo, hi = -(1 << (n - 1)), (1 << (n - 1)) - 1
            v = max(lo, min(hi, v)) & ((1 << n) - 1)
        else:
            v = max(0, min((1 << n) - 1, v))
        bits |= v << pos
        pos += n
    body = bits.to_bytes((pos + 7) // 8, "little")
    return bytes([(VERSION << 5) | type_, seq & 0xFF]) + body


def encode(leaf, d, seq=0):
    """The inverse of decode(): a decoded dict for `leaf` -> frame.
    For the fake car and self-tests; the real car packs in C."""
    v = dict(d)
    if leaf == "state":
        t = 2 if "enc_l" in d else 1
        line = d.get("line", "00")
        v.update(line=(line[0] == "1") | ((line[1] == "1") << 1),
                 bc=sym_code(d.get("bc", "-")))
        return pack(t, d.get("seq", seq), v)
    if leaf == "status":
        v.update(sink=SINKS.index(d.get("sink", "other")),
                 live=sum(1 << i for i, c in enumerate(d.get("live", "0000")) if c == "1"))
        return pack(3, seq, v)
    if leaf == "barcode":
        v.update(sym=sym_code(d.get("sym")))
        return pack(4, seq, v)
    if leaf == "bcscan":
        txt = d.get("txt", "")[:3]
        pat = d.get("pat", "-")
        v.update(txt=sum(sym_code(c) << (5 * i) for i, c in enumerate(txt)),
                 pat=0 if pat == "-" else sum(1 << (8 - i) for i, c in enumerate(pat) if c == "w"),
                 min_10us=d.get("min_us", 0) // 10, max_10us=d.get("max_us", 0) // 10)
        return pack(5, seq, v)
    if leaf == "bcbits":
        flat = d.get("bits", "").replace(" ", "")
        # drop the inter-character gap bits back in: the string is
        # 9 bits, space, gap bit, space, ...; removing spaces keeps order
        raw = [0] * 8
        for i, c in enumerate(flat[:64]):
            if c == "1":
                raw[i // 8] |= 1 << (i % 8)
        v.update(first=1 if d.get("first") == "bar" else 0,
                 nar_10us=d.get("nar_us", 0) // 10, wid_10us=d.get("wid_us", 0) // 10,
                 **{"b%d" % i: raw[i] for i in range(8)})
        return pack(6, seq, v)
    if leaf == "spin":
        found = 0
        for i, k in enumerate(("l0", "l1", "r1", "r2")):
            s = d.get(k, "-")
            if s != "-":
                deg, mm = s.split("/")
                v["d%d" % i], v["r%d" % i] = int(deg), int(mm) // 2
                found |= 1 << i
        v["found"] = found
        return pack(7, seq, v)
    if leaf == "ack":
        v.update(rx=ord(d.get("rx", "?")[:1] or "?"))
        return pack(15, seq, v)
    for t, (lf, _, _) in FRAMES.items():
        if lf == leaf:
            return pack(t, seq, v)
    raise KeyError(leaf)


def log_text(payload):
    """The broker's log form of one payload: decoded JSON, a tab, the raw
    hex. Plain text (commands) is logged as it is; anything else as hex."""
    r = decode(payload)
    if r is not None:
        return json.dumps(r[1], separators=(",", ":")) + "\t" + payload.hex()
    try:
        text = payload.decode("ascii")
        if text.isprintable():
            return text
    except UnicodeDecodeError:
        pass
    return "hex:" + payload.hex()


if __name__ == "__main__":
    for h in sys.argv[1:]:
        print(decode(bytes.fromhex(h)))
