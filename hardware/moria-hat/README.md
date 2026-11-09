# moria hat v1 - Pi Zero W sensor hat

65 x 30 mm, 2-layer FR-4, standard Pi Zero pHAT outline (M2.5 holes 58 x 23 mm).
Screw terminals for 2x DS18B20 (1-Wire, GPIO4) and 1x DHT22 (GPIO17), 4.7 k pull-up and a 100 nF cap on board.
DRC clean (KiCad 10, 0 violations, 0 unconnected). Files: `moria-hat.kicad_pcb`, `moria-hat-gerbers.zip` (order this), `gen_hat.py` (regenerates everything).

## Nets
| Pi pin | Signal | Goes to |
|---|---|---|
| 1 | 3V3 | terminal pin 1 (square pad) of J2/J3/J4, R1, C1 |
| 7 | GPIO4 (1-Wire) | terminal pin 2 of J2/J3, R1 other end |
| 11 | GPIO17 | terminal pin 2 of J4 (DHT22) |
| 6, 9, 14, 20, 25, 30, 34, 39 | GND | terminal pin 3, copper pours on both layers |

All three terminals: pin 1 = 3V3 (square pad, left), pin 2 = data, pin 3 = GND (silkscreen: 3V3 / DAT / GND).
DS18B20 probe wires: red = 3V3, yellow (or white) = DAT, black = GND. Wire entry faces the bottom long edge, away from the GPIO header.

## BOM
| Ref | Part |
|---|---|
| J1 | 2x20 female header, 2.54 mm, straight; soldered on the **underside**, plugs onto the Pi's male header |
| J2-J4 | 3-pole screw terminal, 5.08 mm pitch (ARK / KF128-5.08-3P), 3 pcs |
| R1 | 4.7 kOhm, 1/4 W axial |
| C1 | 100 nF ceramic disc, 2.5 mm lead spacing |

The Pi Zero W needs a male 2x20 header soldered to it. Without the socket you can also solder the hat straight onto the Pi's header pins (all 40 holes are plated).

## Order settings (JLCPCB or similar)
2 layers, 65 x 30 mm, qty 5+, FR-4 1.6 mm, HASL (lead-free), any solder mask colour, no assembly.
Upload `moria-hat-gerbers.zip` as is (Gerber + Excellon drill, PTH and NPTH separate).
