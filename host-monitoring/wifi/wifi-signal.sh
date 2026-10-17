#!/bin/bash
# Writes the signal strength of every *known* wifi network (saved NetworkManager
# wifi profile) as InfluxDB line protocol, for telegraf's inputs.file to pick up
# (the telegraf container sees it at /hostfs/var/lib/wifi-metrics/).
# Run as root every minute (see wifi-signal.timer). A rescan is triggered at most
# every RESCAN_EVERY seconds, otherwise NetworkManager's cached scan is used.
set -u
DIR=/var/lib/wifi-metrics
OUT=$DIR/wifi_scan.influx
STAMP=$DIR/.last-rescan
RESCAN_EVERY=300

command -v nmcli >/dev/null || exit 0
mkdir -p "$DIR"

now=$(date +%s)
last=$(stat -c %Y "$STAMP" 2>/dev/null || echo 0)
if [ $((now - last)) -ge $RESCAN_EVERY ]; then
  nmcli dev wifi rescan >/dev/null 2>&1
  touch "$STAMP"
  sleep 8
fi

# SSIDs of saved wifi profiles
known=$(nmcli -t -f NAME,TYPE con show | while IFS= read -r l; do
  [ "${l##*:}" = "802-11-wireless" ] && nmcli -g 802-11-wireless.ssid con show "${l%:*}"
done)

esc() { sed 's/[ ,=]/\\&/g'; }

tmp=$DIR/.wifi_scan.tmp
nmcli -t -e no -f IN-USE,SSID,CHAN,SIGNAL,BSSID dev wifi list | while IFS=: read -r inuse ssid chan sig bssid; do
  [ -n "$ssid" ] && grep -qxF -- "$ssid" <<<"$known" || continue
  conn=0; [ "$inuse" = "*" ] && conn=1
  printf 'wifi_scan,ssid=%s,bssid=%s,chan=%s connected=%di,signal_pct=%si\n' \
    "$(esc <<<"$ssid")" "$bssid" "$chan" "$conn" "$sig"
done >"$tmp"

# Real link quality/level (dBm) of the connected interface. Read here because the
# telegraf container has its own network namespace and can't see wlan0 in /proc/net.
awk 'NR>2 && $1 ~ /:$/ {i=$1; sub(/:$/,"",i); q=$3; l=$4; sub(/\./,"",q); sub(/\./,"",l); \
  printf "wifi_link,interface=%s quality=%si,level_dbm=%si\n", i, q, l}' /proc/net/wireless >>"$tmp"

mv "$tmp" "$OUT"
