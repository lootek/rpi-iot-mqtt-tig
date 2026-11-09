#!/usr/bin/env python3
"""Build the moria Pi Zero hat (65x30 mm, 2-layer) with KiCad's pcbnew. Run with KiCad's bundled python:
   /Applications/KiCad/KiCad.app/Contents/Frameworks/Python.framework/Versions/Current/bin/python3 gen_hat.py
Plan view = Pi Zero as in the user's photos' orientation convention: SD end left, header along the top edge,
pin 1 (3V3) bottom-left of the header, even pins on the outer (top) row. All coordinates in mm from the board's top-left.
"""
import math, sys
K = "/Applications/KiCad/KiCad.app/Contents"
sys.path.insert(0, K + "/Frameworks/Python.framework/Versions/Current/lib/python3.9/site-packages")
import pcbnew
from pcbnew import (BOARD, NETINFO_ITEM, PCB_TRACK, PCB_SHAPE, PCB_TEXT, ZONE, ZONE_FILLER, VECTOR2I, FromMM,
                    EDA_ANGLE, DEGREES_T, FootprintLoad, F_Cu, B_Cu, F_SilkS, Edge_Cuts, SHAPE_T_SEGMENT, SHAPE_T_ARC)

FP = K + "/SharedSupport/footprints/"
W, H, R = 65.0, 30.0, 3.0
def mm(v): return FromMM(v)
def P(x, y): return VECTOR2I(mm(x), mm(y))

b = BOARD()
nets = {}
for n in ("3V3", "GND", "ONEWIRE", "DHT"):
    ni = NETINFO_ITEM(b, n); b.Add(ni); nets[n] = ni

def fp(lib, name, ref, val, x, y, rot=0, underside=False):
    f = FootprintLoad(FP + lib + ".pretty", name)
    f.SetReference(ref); f.SetValue(val)
    b.Add(f)
    if underside:
        f.Flip(VECTOR2I(0, 0), pcbnew.FLIP_DIRECTION_LEFT_RIGHT)
    f.SetPosition(P(x, y)); f.SetOrientation(EDA_ANGLE(rot, DEGREES_T))
    f.Value().SetVisible(False)
    if ref.startswith("H"):
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

# outline (rounded rectangle, r=3 like the pHAT spec)
seg(R, 0, W-R, 0); seg(W, R, W, H-R); seg(W-R, H, R, H); seg(0, H-R, 0, R)
arc(W-R, R, 270); arc(W-R, H-R, 0); arc(R, H-R, 90); arc(R, R, 180)

# mounting holes (Pi Zero: 58 x 23 mm, 3.5 mm in from the edges)
for i, (x, y) in enumerate(((3.5, 3.5), (61.5, 3.5), (3.5, 26.5), (61.5, 26.5))):
    fp("MountingHole", "MountingHole_2.7mm_M2.5", "H%d" % (i + 1), "M2.5", x, y)

# 2x20 socket (mounts on the underside, plugs onto the Pi's header). Rotated so pin 1 is bottom-left, odd pins inner row.
J1 = fp("Connector_PinSocket_2.54mm", "PinSocket_2x20_P2.54mm_Vertical", "J1", "Pi GPIO", 8.37, 4.77, 270, underside=True)
GND_PINS = ("6", "9", "14", "20", "25", "30", "34", "39")
m = {"1": "3V3", "7": "ONEWIRE", "11": "DHT"}
m.update({p: "GND" for p in GND_PINS})
setnet(J1, m)

def text(s, x, y, size=1.0, layer=F_SilkS):
    t = PCB_TEXT(b); t.SetText(s); t.SetPosition(P(x, y)); t.SetLayer(layer)
    t.SetTextSize(VECTOR2I(mm(size), mm(size))); t.SetTextThickness(mm(0.15)); b.Add(t)

# screw terminals, 5.08 mm pitch (Phoenix MKDS 1,5/3-5.08 footprint = generic ARK/KF128 3-pole blocks): pad 1 = 3V3, 2 = data, 3 = GND
TY = 22.0
def term(ref, val, x, data):
    f = fp("TerminalBlock_Phoenix", "TerminalBlock_Phoenix_MKDS-1,5-3-5.08_1x03_P5.08mm_Horizontal", ref, val, x, TY)
    setnet(f, {"1": "3V3", "2": data, "3": "GND"}); f.Reference().SetVisible(False)
    for i, lab in enumerate(("3V3", "DAT", "GND")):
        text(lab, x + i * 5.08, 15.6, 0.8)
    return f
J2 = term("J2", "DS18B20 #1", 10.5, "ONEWIRE")
J3 = term("J3", "DS18B20 #2", 27.0, "ONEWIRE")
J4 = term("J4", "DHT22", 43.5, "DHT")

# pull-up and decoupling
R1 = fp("Resistor_THT", "R_Axial_DIN0207_L6.3mm_D2.5mm_P2.54mm_Vertical", "R1", "4.7k", 24.0, 14.2, 90)
setnet(R1, {"1": "3V3", "2": "ONEWIRE"})
C1 = fp("Capacitor_THT", "C_Disc_D5.0mm_W2.5mm_P2.50mm", "C1", "100n", 56.0, 12.5)
setnet(C1, {"1": "3V3", "2": "GND"})

# routing. 3V3 on the back copper, data on the front, GND as a pour on both
PW, SW = 0.5, 0.3
track([(8.37, 4.77), (8.37, 14.2), (56.0, 14.2), (56.0, 12.5)], B_Cu, "3V3", PW)
for x in (10.5, 27.0, 43.5):
    track([(x, 14.2), (x, TY)], B_Cu, "3V3", PW)
# 1-Wire data: header pin 7 -> bus -> both terminals + pull-up
track([(15.99, 4.77), (15.99, 18.5)], F_Cu, "ONEWIRE", SW)
track([(15.58, TY), (15.58, 18.5), (32.08, 18.5), (32.08, TY)], F_Cu, "ONEWIRE", SW)
track([(15.99, 11.66), (24.0, 11.66)], F_Cu, "ONEWIRE", SW)
# DHT data: header pin 11 -> J4
track([(21.07, 4.77), (21.07, 9.0), (48.58, 9.0), (48.58, TY)], F_Cu, "DHT", SW)

# GND pours
for layer in (F_Cu, B_Cu):
    z = ZONE(b); z.SetLayer(layer); z.SetNet(nets["GND"])
    z.SetLocalClearance(mm(0.3)); z.SetMinThickness(mm(0.25))
    o = z.Outline(); o.NewOutline()
    for x, y in ((0.3, 0.3), (W-0.3, 0.3), (W-0.3, H-0.3), (0.3, H-0.3)):
        o.Append(mm(x), mm(y))
    b.Add(z)
ZONE_FILLER(b).Fill(b.Zones())

text("moria hat v1 - Pi Zero W", 40.0, 12.5)
for x, lab in ((10.5, "DS18B20 #1"), (27.0, "DS18B20 #2"), (43.5, "DHT22")):
    text(lab, x + 5.08, 28.6, 0.9)
text("pin1", 8.37, 0.9, 0.8)

pcbnew.SaveBoard("moria-hat.kicad_pcb", b)
for p in J1.Pads():
    if p.GetNumber() in ("1", "2", "7", "11"):
        print("pad", p.GetNumber(), round(pcbnew.ToMM(p.GetPosition().x), 2), round(pcbnew.ToMM(p.GetPosition().y), 2))
