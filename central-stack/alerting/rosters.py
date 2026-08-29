"""Single source of truth for which series the alert rules are allowed to see.

Every roster here is pinned deliberately. InfluxDB keeps tag values forever, so an
unpinned query happily alerts on hardware that was decommissioned years ago.

Verified against live InfluxDB on lothlorien, 2026-08-29.
"""

# --- DHT22 temperature/humidity sensors -------------------------------------
# The 14 that are actually publishing. Confirmed live: every one of these had a
# reading less than 40s old at the time of writing.
DHT22_ALL = [
    "attic", "backyard_shed", "basement", "bathroom_hex", "bathroom_tetris",
    "bedroom", "dining_room", "garage", "guests", "kids_room_1", "kitchen",
    "living_room", "office", "patio",
]

# Heated interior rooms. Comfort/mould/plausibility rules apply here only --
# an unheated shed legitimately sits at 99.9% humidity and near 0C in winter.
DHT22_INDOOR = [
    "basement", "bathroom_hex", "bathroom_tetris", "bedroom", "dining_room",
    "guests", "kids_room_1", "kitchen", "living_room", "office",
]

# Unheated / outdoor-ish. Excluded from comfort rules.
DHT22_UNHEATED = ["attic", "backyard_shed", "garage", "patio"]

# Where a freeze actually costs money (pipes, stored liquids).
DHT22_FREEZE_RISK = ["attic", "backyard_shed", "basement", "garage", "patio"]

# --- other MQTT sensors -----------------------------------------------------
SCD30_LOCATIONS = ["dining_room", "office"]     # CO2
SPS30_LOCATIONS = ["patio"]                     # particulates
WIFI_LOCATIONS = ["bathroom_tetris", "dining_room", "office"]

# --- telegraf hosts ---------------------------------------------------------
# rohan is deliberately absent: last metric 2026-06-23, so including it would
# mean a permanently-firing rule. Re-add it here once it is back on the network.
HOSTS = ["ithilien", "lothlorien", "shire"]

# Containers expected to be running, per host. Pinned because telegraf has
# reported 18 stale one-off container names (cool_yalow, elated_clarke, ...)
# that would otherwise each become a phantom "container down" alert.
EXPECTED_CONTAINERS = {
    "lothlorien": ["grafana", "influxdb", "mosquitto", "mqtt-to-influxdb",
                   "telegraf", "iot-dht22"],
    "ithilien": ["telegraf", "yt-rpi-player", "iot-dht22"],
    "shire": ["telegraf"],
}

# --- known-bad series, documented so nobody "helpfully" re-adds them ---------
# arbor           dht22, silent 423 days
# backyard-shed   hyphen typo of backyard_shed, silent 413 days
# soil_1, soil_2  analog/dht11 soil probes, silent 426 days
# "bathroom_tetris"  <- WITH LITERAL QUOTE CHARACTERS. A second physical device
#                    is still publishing under a malformed tag (last seen ~1h
#                    before writing) with humidity 91.6-96.5% vs the real
#                    sensor's 70.1-92.3%. Fix belongs in the device firmware;
#                    until then the ^(...)$ anchoring below keeps it out.
# rivendell, rivendell.local  retired Mac, telegraf host tag
EXCLUDED = ["arbor", "backyard-shed", "soil_1", "soil_2", '"bathroom_tetris"',
            "rohan", "rivendell", "rivendell.local"]


def rx(values):
    """InfluxQL anchored alternation, e.g. ^(office|patio)$

    Anchoring matters: an unanchored /bathroom_tetris/ also matches the
    malformed '"bathroom_tetris"' tag described above.
    """
    return "^(" + "|".join(values) + ")$"
