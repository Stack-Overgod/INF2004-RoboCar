"""
gen_hcsr04_divider.py - draw the HC-SR04 Echo voltage divider two ways:
a schematic (what it is) and a breadboard sketch (how to build it).

Left:  HC-SR04 -> Grove 2, Echo through 1 kOhm to GP3, 2 kOhm from GP3 to GND.
Right: the same on a mini breadboard, wire by wire, using the Grove cable's
       colour code (black GND, red 3V3, white GP2, yellow GP3).

Run:  python docs/img/hw/gen_hcsr04_divider.py
Out:  docs/img/hw/hcsr04_divider.png
"""

import os
from PIL import Image, ImageDraw, ImageFont

# ----------------------------------------------------------------- palette
BG     = "#ffffff"
INK    = "#1a1a1a"
GREY   = "#6b6b6b"
LIGHT  = "#f2f2f2"
PCB    = "#5c2d91"
PCB_E  = "#3b1a5e"
SENSOR = "#2f6fb5"
SENS_E = "#1d4a7a"
BOARD  = "#f7f1dc"
BOARD_E= "#c9bd93"
RES_B  = "#e9d6a8"     # resistor body
RES_E  = "#8a7a4a"
W_BLK  = "#222222"
W_RED  = "#d62828"
W_WHT  = "#e8e8e8"
W_YEL  = "#f2c500"
W_GRN  = "#2e9e4f"
W_BLU  = "#3b78d8"
VOLT5  = "#d62828"
VOLT33 = "#2e7d4f"

FONTDIR = r"C:\Windows\Fonts"


def font(name, size):
    for cand in (name, "arial.ttf"):
        try:
            return ImageFont.truetype(os.path.join(FONTDIR, cand), size)
        except OSError:
            continue
    return ImageFont.load_default()


F_H1 = font("arialbd.ttf", 30)
F_H2 = font("arialbd.ttf", 22)
F_B  = font("arialbd.ttf", 18)
F_T  = font("arial.ttf", 17)
F_S  = font("arial.ttf", 14)
F_XS = font("arial.ttf", 12)

W, H = 2600, 1040
img = Image.new("RGB", (W, H), BG)
d = ImageDraw.Draw(img)


def tw(t, f):
    return d.textlength(t, font=f)


def ctext(x, y, t, f, fill=INK):
    d.text((x - tw(t, f) / 2, y), t, font=f, fill=fill)


def rtext(x, y, t, f, fill=INK):
    d.text((x - tw(t, f), y), t, font=f, fill=fill)


def wire(pts, col, w=6):
    d.line(pts, fill=col, width=w, joint="curve")


def dot(x, y, r=7, col=INK):
    d.ellipse([x - r, y - r, x + r, y + r], fill=col)


def resistor_h(x0, y, length, label, sub, bands):
    """Horizontal resistor centred on y from x0 to x0+length, with colour bands."""
    bw, bh = 92, 34
    cx = x0 + length / 2
    wire([(x0, y), (cx - bw / 2, y)], INK, 4)
    wire([(cx + bw / 2, y), (x0 + length, y)], INK, 4)
    d.rounded_rectangle([cx - bw / 2, y - bh / 2, cx + bw / 2, y + bh / 2], radius=10, fill=RES_B, outline=RES_E, width=2)
    for i, col in enumerate(bands):
        bx = cx - bw / 2 + 16 + i * 14
        d.rectangle([bx, y - bh / 2 + 2, bx + 7, y + bh / 2 - 2], fill=col)
    d.rectangle([cx + bw / 2 - 16, y - bh / 2 + 2, cx + bw / 2 - 9, y + bh / 2 - 2], fill="#c8a94a")  # gold tolerance
    ctext(cx, y - bh / 2 - 26, label, F_B)
    ctext(cx, y + bh / 2 + 8, sub, F_S, GREY)


def resistor_v(x, y0, length, label, sub, bands, label_left=False, label_x=None):
    bw, bh = 34, 92
    cy = y0 + length / 2
    wire([(x, y0), (x, cy - bh / 2)], INK, 4)
    wire([(x, cy + bh / 2), (x, y0 + length)], INK, 4)
    d.rounded_rectangle([x - bw / 2, cy - bh / 2, x + bw / 2, cy + bh / 2], radius=10, fill=RES_B, outline=RES_E, width=2)
    for i, col in enumerate(bands):
        by = cy - bh / 2 + 16 + i * 14
        d.rectangle([x - bw / 2 + 2, by, x + bw / 2 - 2, by + 7], fill=col)
    d.rectangle([x - bw / 2 + 2, cy + bh / 2 - 16, x + bw / 2 - 2, cy + bh / 2 - 9], fill="#c8a94a")
    if label_left:
        lx = label_x if label_x is not None else x - bw / 2 - 12
        rtext(lx, cy - 22, label, F_B)
        rtext(lx, cy + 2, sub, F_S, GREY)
    else:
        d.text((x + bw / 2 + 12, cy - 22), label, font=F_B, fill=INK)
        d.text((x + bw / 2 + 12, cy + 2), sub, font=F_S, fill=GREY)


BROWN, BLACK, RED = "#6b3e1e", "#111111", "#d62828"
BANDS_1K = (BROWN, BLACK, RED)          # brown black red = 1 kOhm
BANDS_2K = (RED, BLACK, RED)            # red black red   = 2 kOhm

# ================================================================= LEFT: schematic
d.text((60, 40), "1.  What the circuit is  \u2014  a voltage divider on Echo", font=F_H1, fill=INK)
d.text((60, 82), "The HC-SR04 answers with a 5 V pulse on Echo. The Pico's pins must never see more than 3.3 V.", font=F_T, fill=GREY)
d.text((60, 106), "Two resistors in series split the 5 V: the junction between them sits at 3.33 V.", font=F_T, fill=GREY)

# HC-SR04 body
sx0, sy0, sw, sh = 90, 230, 300, 210
d.rounded_rectangle([sx0, sy0, sx0 + sw, sy0 + sh], radius=12, fill=SENSOR, outline=SENS_E, width=3)
for ex in (sx0 + 80, sx0 + sw - 80):
    d.ellipse([ex - 55, sy0 + 25, ex + 55, sy0 + 135], fill="#c9ced6", outline="#7d858f", width=3)
    d.ellipse([ex - 40, sy0 + 40, ex + 40, sy0 + 120], fill="#8d949c", outline="#5c636b", width=2)
ctext(sx0 + 80, sy0 + 140, "T", F_S, "#ffffff")
ctext(sx0 + sw - 80, sy0 + 140, "R", F_S, "#ffffff")
ctext(sx0 + sw / 2, sy0 + 160, "HC-SR04", F_H2, "#ffffff")
# its four pins along the right edge
pin_names = ["VCC", "Trig", "Echo", "GND"]
pin_y = {}
for i, n in enumerate(pin_names):
    py = sy0 + 45 + i * 42
    pin_y[n] = py
    d.rectangle([sx0 + sw - 2, py - 7, sx0 + sw + 26, py + 7], fill="#d8b64a", outline="#8a7a2a")
    rtext(sx0 + sw - 12, py - 10, n, F_B, "#ffffff")

# Grove 2 socket (right side)
gx0, gy0, gw, gh = 1010, 220, 150, 240
d.rounded_rectangle([gx0 - 30, gy0 - 40, gx0 + gw + 30, gy0 + gh + 40], radius=16, fill=PCB, outline=PCB_E, width=3)
ctext(gx0 + gw / 2, gy0 - 32, "Robo Pico", F_B, "#f4f1ea")
d.rectangle([gx0, gy0, gx0 + gw, gy0 + gh], fill="#f4f1ea", outline="#b8b3a8", width=2)
ctext(gx0 + gw / 2, gy0 + gh - 26, "GROVE 2", F_S, GREY)
grove = ["GND", "3V3", "GP2", "GP3"]
grove_y = {}
for i, n in enumerate(grove):
    gy = gy0 + 40 + i * 44
    grove_y[n] = gy
    d.rectangle([gx0 - 2, gy - 7, gx0 + 18, gy + 7], fill="#d8b64a", outline="#8a7a2a")
    d.text((gx0 + 28, gy - 10), n, font=F_B, fill=INK)

# wires: VCC (red) and Trig (white) go straight across
xa = sx0 + sw + 26          # sensor pin tip
xb = gx0 - 2                # grove pin tip
wire([(xa, pin_y["VCC"]), (700, pin_y["VCC"]), (700, grove_y["3V3"]), (xb, grove_y["3V3"])], W_RED)
wire([(xa, pin_y["Trig"]), (640, pin_y["Trig"]), (640, grove_y["GP2"]), (xb, grove_y["GP2"])], "#9a9a9a")
d.text((470, pin_y["VCC"] - 26), "VCC \u2192 3V3  (red wire, direct)", font=F_S, fill=W_RED)
d.text((470, pin_y["Trig"] - 26), "Trig \u2192 GP2  (white wire, direct)", font=F_S, fill=GREY)

# Echo path: pin -> 1k -> junction -> GP3 ; junction -> 2k down -> GND rail
ey = pin_y["Echo"]
jx = 760                    # junction x
wire([(xa, ey), (xa + 40, ey)], INK, 4)
resistor_h(xa + 40, ey, jx - (xa + 40), "1 k\u03a9", "brown \u00b7 black \u00b7 red", BANDS_1K)
dot(jx, ey)
wire([(jx, ey), (jx + 60, ey), (jx + 60, grove_y["GP3"]), (xb, grove_y["GP3"])], W_YEL)
d.text((jx + 74, grove_y["GP3"] + 18), "yellow wire \u2192 GP3", font=F_S, fill="#8a7000")
# 2k down to ground
gnd_y = 600
resistor_v(jx, ey, gnd_y - ey, "2 k\u03a9", "red \u00b7 black \u00b7 red", BANDS_2K)
# GND rail: sensor GND -> rail -> grove GND
wire([(xa, pin_y["GND"]), (xa + 40, pin_y["GND"]), (xa + 40, gnd_y), (jx, gnd_y), (940, gnd_y), (940, grove_y["GND"]), (xb, grove_y["GND"])], W_BLK)
dot(jx, gnd_y, col=W_BLK)
d.text((xa + 60, gnd_y + 10), "GND  (black wire) \u2014 sensor and Pico share this", font=F_S, fill=INK)

# voltage callouts
d.rounded_rectangle([xa + 50, ey + 44, xa + 130, ey + 68], radius=6, fill="#fde8e8", outline=VOLT5)
ctext(xa + 90, ey + 47, "5 V", F_S, VOLT5)
d.rounded_rectangle([jx - 104, ey + 46, jx - 18, ey + 70], radius=6, fill="#e6f4ea", outline=VOLT33)
ctext(jx - 61, ey + 49, "3.33 V", F_S, VOLT33)

d.text((60, 680), "How it works:  5 V  \u00d7  2 k\u03a9 / (1 k\u03a9 + 2 k\u03a9)  =  3.33 V  at the junction.  2.2 k\u03a9 instead of 2 k\u03a9 gives 3.44 V \u2014 also fine.", font=F_T, fill=INK)
d.text((60, 710), "Resistors have no polarity: either leg can go either way. Any \u00bc W through-hole resistor is fine.", font=F_T, fill=GREY)

# ================================================================= RIGHT: breadboard
ox = 1320
d.text((ox, 40), "2.  How to build it  \u2014  on a mini breadboard", font=F_H1, fill=INK)
d.text((ox, 82), "Rows are numbered; the five holes in a row (a\u2013e) are joined together underneath.", font=F_T, fill=GREY)

bx0, by0 = ox + 40, 170
ROWS, COLS = 17, 5
pitch = 34
bw, bh = COLS * pitch + 60, ROWS * pitch + 40
d.rounded_rectangle([bx0, by0, bx0 + bw, by0 + bh], radius=10, fill=BOARD, outline=BOARD_E, width=3)


def hole(r, c):
    return (bx0 + 50 + c * pitch, by0 + 30 + r * pitch)


for r in range(ROWS):
    d.text((bx0 + 8, hole(r, 0)[1] - 8), f"{r + 1:>2}", font=F_XS, fill=GREY)
    for c in range(COLS):
        x, y = hole(r, c)
        d.rectangle([x - 5, y - 5, x + 5, y + 5], fill="#3a3a3a")
for c in range(COLS):
    ctext(hole(0, c)[0], by0 + 6, "abcde"[c], F_XS, GREY)

# highlight the three rows we use
for r, col in ((4, "#fde8e8"), (9, "#e6f4ea"), (14, "#e8e8e8")):
    x0, y0 = hole(r, 0)
    x1, _ = hole(r, COLS - 1)
    d.rounded_rectangle([x0 - 14, y0 - 12, x1 + 14, y0 + 12], radius=8, outline=col, width=3)

# component legs
R5, R10, R15 = 4, 9, 14   # zero-based rows 5, 10, 15


def leg_v(r0, r1, c, label, sub, bands):
    x, y0 = hole(r0, c)
    _, y1 = hole(r1, c)
    resistor_v(x, y0, y1 - y0, label, sub, bands, label_left=True, label_x=bx0 - 16)
    dot(x, y0, 5, "#555555")
    dot(x, y1, 5, "#555555")


leg_v(R5, R10, 0, "1 k\u03a9", "row 5 \u2192 row 10", BANDS_1K)
leg_v(R10, R15, 2, "2 k\u03a9", "row 10 \u2192 row 15", BANDS_2K)

# wires into rows (from the right side of the board)
right = bx0 + bw
x5 = hole(R5, 4)[0]
y5 = hole(R5, 4)[1]
x10, y10 = hole(R10, 4)
x15, y15 = hole(R15, 4)
x15b, _ = hole(R15, 0)

wx = right + 80
# Echo -> row 5 (blue jumper from sensor)
wire([(x5, y5), (wx, y5)], W_BLU)
d.text((wx + 12, y5 - 10), "HC-SR04  Echo   (jumper wire)", font=F_B, fill=W_BLU)
# GP3 yellow -> row 10
wire([(x10, y10), (wx, y10)], W_YEL)
d.text((wx + 12, y10 - 10), "Grove 2  GP3   (yellow)", font=F_B, fill="#8a7000")
d.text((wx + 12, y10 + 12), "the junction \u2014 3.33 V lives here", font=F_S, fill=VOLT33)
# GND black -> row 15 ; sensor GND jumper -> row 15 too
wire([(x15, y15), (wx, y15)], W_BLK)
d.text((wx + 12, y15 - 10), "Grove 2  GND   (black)", font=F_B, fill=INK)
wire([(x15, y15), (right + 30, y15), (right + 30, by0 + bh + 20)], W_BLK)
d.text((right + 42, by0 + bh + 10), "HC-SR04  GND  (jumper, same row 15)", font=F_S, fill=INK)

# direct wires listed beside
lx, ly = ox + 40, by0 + bh + 70
d.text((lx, ly), "Not on the breadboard \u2014 direct wires:", font=F_H2, fill=INK)
rows = [("red", W_RED, "Grove 2  3V3  \u2192  HC-SR04  VCC"),
        ("white", "#9a9a9a", "Grove 2  GP2  \u2192  HC-SR04  Trig")]
for i, (nm, col, txt) in enumerate(rows):
    yy = ly + 44 + i * 34
    wire([(lx, yy + 10), (lx + 60, yy + 10)], col)
    d.text((lx + 76, yy), txt, font=F_T, fill=INK)

# steps
sx = ox + 760
d.text((sx, by0), "Steps", font=F_H2, fill=INK)
steps = [
    "Plug the Grove cable into GROVE 2.",
    "   Four loose ends: black = GND, red = 3V3,",
    "   white = GP2, yellow = GP3.",
    "1 k\u03a9: one leg row 5, other leg row 10.",
    "2 k\u03a9: one leg row 10, other leg row 15.",
    "Jumper: HC-SR04 Echo \u2192 row 5.",
    "Yellow (GP3) \u2192 row 10  (the junction).",
    "Black (GND) \u2192 row 15,",
    "   plus a jumper HC-SR04 GND \u2192 row 15.",
    "Red (3V3) \u2192 HC-SR04 VCC.",
    "White (GP2) \u2192 HC-SR04 Trig.",
    "Multimeter check, DC volts:",
    "   black probe row 15, red probe row 10.",
    "   At rest ~0 V; while ranging never above 3.4 V.",
    "   Reads 5 V \u2192 the 2 k\u03a9 isn't reaching row 15.",
    "   Never reads anything \u2192 the 1 k\u03a9 isn't",
    "   between row 5 and row 10.",
]
n = 1
yy = by0 + 40
for s in steps:
    if s.startswith("   "):
        d.text((sx + 30, yy), s.strip(), font=F_S, fill=GREY)
        yy += 24
    else:
        d.text((sx, yy), f"{n}.", font=F_B, fill=INK)
        d.text((sx + 30, yy), s, font=F_T, fill=INK)
        n += 1
        yy += 30

d.text((60, H - 50), "No breadboard? Make the same three joints by twisting and taping/soldering: Echo\u21941 k\u03a9,  1 k\u03a9\u2194yellow\u21942 k\u03a9,  2 k\u03a9\u2194black\u2194sensor GND.  "
                     "Never plug Echo straight into GP3.", font=F_T, fill=INK)

root = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
out = os.path.join(root, "docs", "img", "hw", "hcsr04_divider.png")
img.save(out)
print("wrote", out)
