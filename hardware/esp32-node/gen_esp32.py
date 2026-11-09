#!/usr/bin/env python3
"""Build the ESP32 node carrier (86x50 mm, 2-layer) with KiCad's pcbnew. Run with KiCad's bundled python:
   /Applications/KiCad/KiCad.app/Contents/Frameworks/Python.framework/Versions/Current/bin/python3 gen_esp32.py
Takes a 30-pin ESP32 DevKit V1 (two 1x15 headers, 25.4 mm apart, USB end at the bottom) on female sockets.
Plan view: x right, y down, mm from the board's top-left. Pad numbers of the 1x15 sockets run top to bottom.

Left header (pad 1..15):  EN VP VN D34 D35 D32 D33 D25 D26 D27 D14 D12 D13 GND VIN
Right header (pad 1..15): D23 D22 TX0 RX0 D21 D19 D18 D5 D17 D16 D4 D2 D15 GND 3V3
"""
import math, sys
K = "/Applications/KiCad/KiCad.app/Contents"
sys.path.insert(0, K + "/Frameworks/Python.framework/Versions/Current/lib/python3.9/site-packages")
import pcbnew
from pcbnew import (BOARD, NETINFO_ITEM, PCB_TRACK, PCB_SHAPE, PCB_TEXT, ZONE, ZONE_FILLER, VECTOR2I, FromMM,
                    EDA_ANGLE, DEGREES_T, FootprintLoad, F_Cu, B_Cu, F_SilkS, Edge_Cuts, SHAPE_T_SEGMENT, SHAPE_T_ARC)

FP = K + "/SharedSupport/footprints/"
W, H, R = 86.0, 54.0, 3.0
def mm(v): return FromMM(v)
def P(x, y): return VECTOR2I(mm(x), mm(y))

b = BOARD()
nets = {}
for n in ("3V3", "GND", "SDA", "SCL", "DHT", "LED_G", "LED_Y", "LED_R", "G_A", "Y_A", "R_A"):
    ni = NETINFO_ITEM(b, n); b.Add(ni); nets[n] = ni

def fp(lib, name, ref, val, x, y, rot=0, hide_ref=False):
    f = FootprintLoad(FP + lib + ".pretty", name)
    f.SetReference(ref); f.SetValue(val)
    b.Add(f)
    f.SetPosition(P(x, y)); f.SetOrientation(EDA_ANGLE(rot, DEGREES_T))
    f.Value().SetVisible(False)
    if hide_ref:
        f.Reference().SetVisible(False)
    return f

def setnet(f, mapping):
    for pad in f.Pads():
        n = mapping.get(pad.GetNumber())
        if n: pad.SetNet(nets[n])

def track(pts, layer, net, w):
    for (x1, y1), (x2, y2) in zip(pts, pts[1:]):
        t = PCB_TRACK(b); t.SetStart(P(x1, y1)); t.SetEnd(P(x2, y2)); t.SetWidth(mm(w))
        t.SetLayer(layer); t.SetNet(nets[net]); b.Add(t)

def seg(x1, y1, x2, y2):
    s = PCB_SHAPE(b); s.SetShape(SHAPE_T_SEGMENT); s.SetLayer(Edge_Cuts); s.SetWidth(mm(0.1))
    s.SetStart(P(x1, y1)); s.SetEnd(P(x2, y2)); b.Add(s)
def arc(cx, cy, a0):
    s = PCB_SHAPE(b); s.SetShape(SHAPE_T_ARC); s.SetLayer(Edge_Cuts); s.SetWidth(mm(0.1))
    a1, am, a2 = [math.radians(a) for a in (a0, a0 + 45, a0 + 90)]
    s.SetArcGeometry(P(cx + R*math.cos(a1), cy + R*math.sin(a1)),
                     P(cx + R*math.cos(am), cy + R*math.sin(am)), P(cx + R*math.cos(a2), cy + R*math.sin(a2)))
    b.Add(s)

def text(s, x, y, size=1.0, layer=F_SilkS):
    t = PCB_TEXT(b); t.SetText(s); t.SetPosition(P(x, y)); t.SetLayer(layer)
    t.SetTextSize(VECTOR2I(mm(size), mm(size))); t.SetTextThickness(mm(0.15)); b.Add(t)

# outline
seg(R, 0, W-R, 0); seg(W, R, W, H-R); seg(W-R, H, R, H); seg(0, H-R, 0, R)
arc(W-R, R, 270); arc(W-R, H-R, 0); arc(R, H-R, 90); arc(R, R, 180)
for i, (x, y) in enumerate(((3.5, 3.5), (W-3.5, 3.5), (3.5, H-3.5), (W-3.5, H-3.5))):
    fp("MountingHole", "MountingHole_2.7mm_M2.5", "H%d" % (i + 1), "M2.5", x, y, hide_ref=True)

# DevKit sockets
XL, XR, Y0, PITCH = 30.0, 55.4, 8.0, 2.54
def ry(row): return Y0 + row * PITCH            # row = pad number - 1
JL = fp("Connector_PinSocket_2.54mm", "PinSocket_1x15_P2.54mm_Vertical", "J1", "ESP32 left", XL, Y0)
JR = fp("Connector_PinSocket_2.54mm", "PinSocket_1x15_P2.54mm_Vertical", "J2", "ESP32 right", XR, Y0)
setnet(JL, {"8": "LED_G", "9": "LED_Y", "10": "LED_R", "14": "GND"})
setnet(JR, {"2": "SCL", "5": "SDA", "11": "DHT", "14": "GND", "15": "3V3"})
for r in (JL, JR):
    r.Reference().SetVisible(False)

# LED section: 220 R in series per LED, cathodes common. Terminal pins (top to bottom): G Y R K
PW, SW = 0.5, 0.3
JK = fp("TerminalBlock_Phoenix", "TerminalBlock_Phoenix_MKDS-1,5-4-5.08_1x04_P5.08mm_Horizontal", "J3", "LEDs", 6.0, 20.7, 270, hide_ref=True)
setnet(JK, {"1": "G_A", "2": "Y_A", "3": "R_A", "4": "GND"})
RV = "R_Axial_DIN0207_L6.3mm_D2.5mm_P2.54mm_Vertical"
y25, y26, y27 = ry(7), ry(8), ry(9)             # D25, D26, D27 rows on the left socket
r3 = fp("Resistor_THT", RV, "R3", "220", 24.0, y25, 90)   # pad 2 goes up
r4 = fp("Resistor_THT", RV, "R4", "220", 20.0, y26, 90)
r5 = fp("Resistor_THT", RV, "R5", "220", 16.0, y27, 90)
setnet(r3, {"1": "LED_G", "2": "G_A"}); setnet(r4, {"1": "LED_Y", "2": "Y_A"}); setnet(r5, {"1": "LED_R", "2": "R_A"})
track([(XL, y25), (24.0, y25)], F_Cu, "LED_G", SW)
track([(XL, y26), (20.0, y26)], F_Cu, "LED_Y", SW)
track([(XL, y27), (16.0, y27)], F_Cu, "LED_R", SW)
track([(24.0, y25 - PITCH), (12.0, y25 - PITCH), (12.0, 20.7), (6.0, 20.7)], F_Cu, "G_A", SW)
track([(20.0, y26 - PITCH), (6.0, y26 - PITCH)], F_Cu, "Y_A", SW)
track([(16.0, y27 - PITCH), (12.0, y27 - PITCH), (12.0, 30.86), (6.0, 30.86)], F_Cu, "R_A", SW)
text("LED top to bottom: G Y R K(-)", 18.0, 17.4, 0.8)

# sensors on the right
BX = 62.0
BME = fp("Connector_PinSocket_2.54mm", "PinSocket_1x06_P2.54mm_Vertical", "J4", "BME280", BX, 9.0, hide_ref=True)
setnet(BME, {"1": "3V3", "2": "GND", "3": "SCL", "4": "SDA", "5": "3V3", "6": "GND"})
TX = 80.0
SCD = fp("TerminalBlock_Phoenix", "TerminalBlock_Phoenix_MKDS-1,5-4-5.08_1x04_P5.08mm_Horizontal", "J5", "SCD30", TX, 25.74, 90, hide_ref=True)
setnet(SCD, {"1": "SDA", "2": "SCL", "3": "GND", "4": "3V3"})             # pad 1 is the bottom pin
DHT = fp("TerminalBlock_Phoenix", "TerminalBlock_Phoenix_MKDS-1,5-3-5.08_1x03_P5.08mm_Horizontal", "J6", "DHT22", TX, 43.16, 90, hide_ref=True)
setnet(DHT, {"1": "GND", "2": "DHT", "3": "3V3"})
C1 = fp("Capacitor_THT", "C_Disc_D5.0mm_W2.5mm_P2.50mm", "C1", "100n", 66.0, 35.5, hide_ref=True)
setnet(C1, {"1": "3V3", "2": "GND"})

# 3V3 on the back copper
track([(XR, ry(14)), (59.5, ry(14)), (59.5, 9.0), (70.0, 9.0), (70.0, 10.5), (TX, 10.5)], B_Cu, "3V3", PW)
track([(59.5, 19.16), (BX, 19.16)], B_Cu, "3V3", PW)
track([(59.5, 33.0), (TX, 33.0)], B_Cu, "3V3", PW)
track([(66.0, 33.0), (66.0, 35.5)], B_Cu, "3V3", PW)
# I2C: BME on the front, continuation to the SCD30 terminal on the back
track([(XR, ry(1)), (58.5, ry(1)), (58.5, 14.08), (BX, 14.08)], F_Cu, "SCL", SW)
track([(XR, ry(4)), (57.5, ry(4)), (57.5, 16.62), (BX, 16.62)], F_Cu, "SDA", SW)
track([(BX, 14.08), (68.0, 14.08), (68.0, 20.66), (TX, 20.66)], B_Cu, "SCL", SW)
track([(BX, 16.62), (66.0, 16.62), (66.0, 25.74), (TX, 25.74)], B_Cu, "SDA", SW)
# DHT data on the front
track([(XR, ry(10)), (57.0, ry(10)), (57.0, 38.08), (TX, 38.08)], F_Cu, "DHT", SW)

# GND pours
for layer in (F_Cu, B_Cu):
    z = ZONE(b); z.SetLayer(layer); z.SetNet(nets["GND"])
    z.SetLocalClearance(mm(0.3)); z.SetMinThickness(mm(0.25))
    o = z.Outline(); o.NewOutline()
    for x, y in ((0.3, 0.3), (W-0.3, 0.3), (W-0.3, H-0.3), (0.3, H-0.3)):
        o.Append(mm(x), mm(y))
    b.Add(z)
ZONE_FILLER(b).Fill(b.Zones())

text("esp32 node v1 - DevKit V1 30-pin, USB down", 38.0, 3.2, 1.0)
text("BME280", BX, 5.8, 0.8)
text("SCD30  3V3 GND SCL SDA", 71.0, 1.9, 0.8)
text("DHT22  3V3 DAT GND", 70.0, 29.6, 0.8)
pcbnew.SaveBoard("esp32-node.kicad_pcb", b)
