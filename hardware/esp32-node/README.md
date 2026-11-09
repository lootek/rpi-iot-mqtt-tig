# esp32 node v1 - carrier for the 30-pin ESP32 DevKit V1

86 x 54 mm, 2-layer FR-4, DRC clean (KiCad 10: 0 violations, 0 unconnected). Used by the shed, office and dining_room nodes
(see ~/ai/esp32-weather-v2 spec). Populate only what a node needs; the shed needs just the BME280 socket.
Files: `esp32-node.kicad_pcb` + `esp32-node-gerbers.zip` (v1: LEDs external on a 4-way terminal), `esp32-node-leds.kicad_pcb` + `esp32-node-leds-gerbers.zip` (v1b: 3 LEDs soldered on the board), `gen_esp32.py` (regenerates both; `--leds-onboard` for v1b), `esp32-wiring.svg/png` (wiring diagram, `gen-esp32-wiring.py`).
Order one of the two Gerber zips. v1b has 5 mm LEDs (green, yellow, red) at the left edge, 7 mm apart, long leg (anode) toward the resistors.

## DevKit and orientation
30-pin DOIT-style DevKit V1 (USB-C, CH340), two 1x15 headers **25.4 mm apart** (10 pitches).
Plan view: USB end of the DevKit at the bottom edge of the PCB (it overhangs a little).
Left header top to bottom: EN VP VN D34 D35 D32 D33 D25 D26 D27 D14 D12 D13 GND VIN.
Right header top to bottom: D23 D22 TX0 RX0 D21 D19 D18 D5 D17 D16 D4 D2 D15 GND 3V3.

## Nets
| ESP32 pin | Signal | Goes to |
|---|---|---|
| 3V3 | 3V3 | BME280 VCC + CSB, SCD30 VIN, DHT22 +, C1 |
| GND | GND | ground pour on both layers, BME280 GND + SDO, SCD30 GND, DHT22 -, LED cathodes |
| D21 | SDA | BME280 SDA, SCD30 SDA |
| D22 | SCL | BME280 SCL, SCD30 SCL |
| D4 | DHT data | DHT22 DAT |
| D25 / D26 / D27 | LED green / yellow / red | 220 ohm each, then the LED terminal |

BME280 address pins are tied on the board (CSB to 3V3 = I2C mode, SDO to GND = address 0x76).
The DHT22 terminal expects a 3-pin module with its own pull-up; a bare 4-pin DHT22 needs a 10k from DAT to 3V3 added by hand.

## Connectors (5.08 mm screw terminals, wire entry faces the board edge)
- LED terminal, left edge, top to bottom: **G Y R K**. Each LED anode to its terminal, all cathodes to K. 220 ohm resistors R3-R5 are on the board.
- SCD30 terminal, right edge, top to bottom: **3V3 GND SCL SDA**.
- DHT22 terminal, right edge, top to bottom: **3V3 DAT GND**.
- BME280: 1x6 female socket, pins VCC GND SCL SDA CSB SDO (GY-BME280 order). Plug the module so it lies to the **right** of the socket (away from the DevKit).

## BOM
| Ref | Part |
|---|---|
| J1, J2 | 1x15 female header 2.54 mm, 2 pcs (DevKit sockets) |
| J3 | 4-pole screw terminal 5.08 mm (LEDs) |
| J4 | 1x6 female header 2.54 mm (BME280) |
| J5 | 4-pole screw terminal 5.08 mm (SCD30) |
| J6 | 3-pole screw terminal 5.08 mm (DHT22) |
| R3-R5 | 220 ohm 1/4 W axial |
| C1 | 100 nF ceramic disc, 2.5 mm lead spacing |

## Order settings
2 layers, 86 x 54 mm, qty 5+, FR-4 1.6 mm, HASL, no assembly. Upload `esp32-node-gerbers.zip` as is.

## Check before ordering
- Row spacing 25.4 mm was derived from the DevKit photos (11 holes counted inclusively on the perfboard = 10 pitches); measure your DevKit's pin rows once with a ruler.
- Terminal wire-entry side was confirmed from the KiCad 3D model, not a datasheet.
- Not test-fitted on real hardware. The BME280 module must not extend left over the DevKit.
