#!/usr/bin/env python3
"""Generate esp32-wiring.svg: wiring of the ESP32 node (30-pin DevKit V1, USB at the bottom)."""
LEFT = ["EN", "VP", "VN", "D34", "D35", "D32", "D33", "D25", "D26", "D27", "D14", "D12", "D13", "GND", "VIN"]
RIGHT = ["D23", "D22", "TX0", "RX0", "D21", "D19", "D18", "D5", "D17", "D16", "D4", "D2", "D15", "GND", "3V3"]
RED, BLK, BLU, ORG, GRN = "#d32f2f", "#222", "#1e88e5", "#e08a00", "#2e7d32"
LEDC = {"G": "#2e9d3a", "Y": "#e6b800", "R": "#d32f2f"}
Y0, DY = 140, 34
XL, XR = 330, 690
def ry(i): return Y0 + i * DY
o = []
a = o.append
a('<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 1180 810" font-family="Helvetica,Arial,sans-serif">')
a('<rect width="1180" height="810" fill="#fff"/>')
a('<text x="30" y="34" font-size="22" font-weight="bold">ESP32 node - wiring (room node: BME280 + SCD30 + DHT22 + 3 LEDs)</text>')
a('<text x="30" y="56" font-size="12.5" fill="#444">30-pin DevKit V1, USB end at the bottom. All parts run from 3V3. LEDs: on-board (v1b) or on the 4-way terminal (v1) - same circuit.</text>')
a('<text x="30" y="74" font-size="12.5" fill="#444">Backyard shed: BME280 only. Wires that cross without a dot are not connected.</text>')

def wire(pts, c, w=3):
    d = "M" + " L".join("%d,%d" % p for p in pts)
    a('<path d="%s" fill="none" stroke="%s" stroke-width="%d" stroke-linejoin="round"/>' % (d, c, w))
def dot(x, y, c): a('<circle cx="%d" cy="%d" r="4.5" fill="%s"/>' % (x, y, c))

# vertical buses on the right: x position, net colour, y extent filled in below
BUS = {"SCL": 740, "SDA": 770, "3V3": 800, "GND": 830}
COL = {"SCL": ORG, "SDA": BLU, "3V3": RED, "GND": BLK}
XB = 900                                         # device boxes start here
pins = {"SCL": ry(1), "SDA": ry(4), "DHT": ry(10), "GND": ry(13), "3V3": ry(14)}
dev = []                                         # (net, y) of every device pin
bme = [("3V3", 128), ("GND", 152), ("SCL", 176), ("SDA", 200), ("3V3", 224), ("GND", 248)]
scd = [("3V3", 300), ("GND", 324), ("SCL", 348), ("SDA", 372)]
dht = [("3V3", pins["DHT"] - 24), ("DHT", pins["DHT"]), ("GND", pins["DHT"] + 24)]
for grp in (bme, scd, dht):
    dev += grp
ext = {}
for net in BUS:
    ys = [pins[net]] + [y for n, y in dev if n == net]
    ext[net] = (min(ys), max(ys))

def hops_for(y, net, x1, x2):
    return [BUS[n] for n in BUS if n != net and x1 < BUS[n] < x2 and ext[n][0] < y < ext[n][1]]

def hwire(x1, x2, y, c, net, w=3):
    d = "M%d,%d" % (x1, y)
    for h in sorted(hops_for(y, net, x1, x2)):
        d += " L%d,%d A5,5 0 0 1 %d,%d" % (h - 5, y, h + 5, y)
    d += " L%d,%d" % (x2, y)
    a('<path d="%s" fill="none" stroke="%s" stroke-width="%d"/>' % (d, c, w))

# buses
for net, x in BUS.items():
    wire([(x, ext[net][0]), (x, ext[net][1])], COL[net])
    a('<text x="%d" y="%d" font-size="11" text-anchor="middle" fill="%s" font-weight="bold">%s</text>' % (x, 96, COL[net], net))
# DevKit pin -> bus
for net in BUS:
    hwire(XR, BUS[net], pins[net], COL[net], net); dot(BUS[net], pins[net], COL[net])
# device stubs
for net, y in dev:
    if net == "DHT":
        hwire(XR, XB, y, GRN, "DHT")        # DHT data pin goes straight to its terminal
        continue
    hwire(BUS[net], XB, y, COL[net], net); dot(BUS[net], y, COL[net])
# DHT pin y differs from the device pin y: bring the pin row to the device with a jog
# (pins['DHT'] = ry(10) = 450; device DAT row = 448 -> aligned by construction below)

# device boxes
def box(y1, y2, title, sub, rows):
    a('<rect x="%d" y="%d" width="230" height="%d" rx="6" fill="#f5f5f5" stroke="#666"/>' % (XB, y1, y2 - y1))
    a('<text x="%d" y="%d" font-size="13" font-weight="bold">%s</text>' % (XB + 100, y1 + 18, title))
    a('<text x="%d" y="%d" font-size="11" fill="#555">%s</text>' % (XB + 100, y1 + 33, sub))
    for (net, y), lab in rows:
        c = GRN if net == "DHT" else COL[net]
        a('<circle cx="%d" cy="%d" r="5" fill="%s"/>' % (XB, y, c))
        a('<text x="%d" y="%d" font-size="12">%s</text>' % (XB + 12, y + 4, lab))
box(104, 262, "BME280", "6-pin socket, I2C 0x76", list(zip(bme, ["VCC", "GND", "SCL", "SDA", "CSB (to 3V3 on board)", "SDO (to GND on board)"])))
box(276, 386, "SCD30", "terminal, 4 wires", list(zip(scd, ["VIN", "GND", "SCL (TX)", "SDA (RX)"])))
box(pins["DHT"] - 58, pins["DHT"] + 42, "DHT22 module", "3-pin, has its own pull-up", list(zip(dht, ["+ (VCC)", "OUT / DATA", "- (GND)"])))

# DevKit pins
a('<rect x="%d" y="%d" width="%d" height="%d" rx="10" fill="#eef2f7" stroke="#789" stroke-dasharray="5 4"/>' % (XL - 22, Y0 - 38, XR - XL + 44, 14 * DY + 76))
a('<text x="%d" y="%d" font-size="14" font-weight="bold" text-anchor="middle" fill="#456">ESP32 DevKit V1 (30-pin)</text>' % ((XL + XR) // 2, Y0 - 16))
a('<text x="%d" y="%d" font-size="12" text-anchor="middle" fill="#666">&#9660; USB-C end</text>' % ((XL + XR) // 2, ry(14) + 38))
used_l = {"D25": LEDC["G"], "D26": LEDC["Y"], "D27": LEDC["R"], "GND": BLK}
used_r = {"D22": ORG, "D21": BLU, "D4": GRN, "GND": BLK, "3V3": RED}
for i in range(15):
    for x, name, used, side in ((XL, LEFT[i], used_l, "l"), (XR, RIGHT[i], used_r, "r")):
        y = ry(i)
        if name in used:
            a('<circle cx="%d" cy="%d" r="11" fill="none" stroke="%s" stroke-width="3"/>' % (x, y, used[name]))
        fillc = "#e53935" if name in ("VIN", "3V3") else ("#222" if name == "GND" else "#cfd8dc")
        a('<circle cx="%d" cy="%d" r="7" fill="%s" stroke="#000" stroke-width="0.8"/>' % (x, y, fillc))
        if side == "l":
            a('<text x="%d" y="%d" font-size="12"%s>%s</text>' % (x + 16, y + 4, ' font-weight="bold"' if name in used else "", name))
        else:
            a('<text x="%d" y="%d" font-size="12" text-anchor="end"%s>%s</text>' % (x - 16, y + 4, ' font-weight="bold"' if name in used else "", name))

# LEDs on the left: pin -> 220R -> LED -> GND bus
GX = 80
chains = (("G", ry(7), ry(7) - 48), ("Y", ry(8), ry(8)), ("R", ry(9), ry(9) + 48))
for lab, ypin, ych in chains:
    c = LEDC[lab]
    wire([(XL, ypin), (292, ypin), (292, ych), (250, ych)], c)
    a('<rect x="170" y="%d" width="80" height="16" fill="#fff" stroke="#555" stroke-width="2"/>' % (ych - 8))
    a('<text x="210" y="%d" font-size="11" text-anchor="middle" fill="#333">220 &#937;</text>' % (ych - 13))
    wire([(170, ych), (140, ych)], c)
    a('<circle cx="125" cy="%d" r="14" fill="%s" stroke="#222" stroke-width="2"/>' % (ych, c))
    a('<text x="125" y="%d" font-size="11" text-anchor="middle" font-weight="bold" fill="#fff">%s</text>' % (ych + 4, lab))
    a('<text x="140" y="%d" font-size="11" fill="#222">+</text>' % (ych - 18))
    wire([(111, ych), (GX, ych)], BLK); dot(GX, ych, BLK)
wire([(GX, ry(7) - 48), (GX, ry(13)), (XL, ry(13))], BLK)
a('<text x="30" y="%d" font-size="12" font-weight="bold">CO2 LEDs</text>' % (ry(7) - 100))
a('<text x="30" y="%d" font-size="11" fill="#555">long leg (+) toward the resistor</text>' % (ry(7) - 85))
a('<text x="100" y="%d" font-size="11" fill="#555">cathodes (-) to GND</text>' % (ry(13) + 16))

# notes
ny = 715
notes = ["I2C bus: BME280 (0x76) and SCD30 (0x61) share SDA D21 / SCL D22.",
         "DHT22 data on D4. The 3-pin module has a built-in pull-up; a bare 4-pin DHT22 needs 10k from DATA to 3V3.",
         "LED pins D25 / D26 / D27 go high to light the LED (series 220 ohm, about 6 mA at 3.3 V).",
         "On the PCB: LEDs go in the 4-way terminal as G Y R K (v1) or are soldered on the board (v1b)."]
for i, t in enumerate(notes):
    a('<text x="30" y="%d" font-size="12" fill="#333">&#8226; %s</text>' % (ny + i * 20, t))
a('</svg>')
open("esp32-wiring.svg", "w").write("\n".join(o))
