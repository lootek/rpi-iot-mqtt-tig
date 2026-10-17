# WiFi signal metrics

`wifi-signal.sh` runs on the host (root, every minute via `wifi-signal.timer`) and
writes `/var/lib/wifi-metrics/wifi_scan.influx`, which telegraf reads through
`/hostfs` (see `[[inputs.file]]` in `volumes/telegraf/telegraf.conf`):

- `wifi_scan` — one series per *known* network (saved NetworkManager wifi profile):
  tags `ssid`, `bssid`, `chan`; fields `signal_pct`, `connected`.
- `wifi_link` — the connected interface: `quality`, `level_dbm` (real dBm).

A rescan is triggered at most every 5 minutes, otherwise NetworkManager's cached
scan is used. Hosts without wifi/NetworkManager simply write nothing.

Install on a host with wifi:

    sudo install -m 755 wifi-signal.sh /usr/local/sbin/
    sudo cp wifi-signal.service wifi-signal.timer /etc/systemd/system/
    sudo systemctl daemon-reload && sudo systemctl enable --now wifi-signal.timer
