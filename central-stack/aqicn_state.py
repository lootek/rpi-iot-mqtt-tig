"""File-backed handshake between the MQTT listener and the aqicn-ui container.

Two files live in a directory bind-mounted into both containers:

    pause.json   written by aqicn-ui,  read here
    status.json  written here,         read by aqicn-ui

One writer per file, so no locking is needed. Everything here degrades to a
no-op rather than raising: this sits in the MQTT message path, and a broken
bind mount must never stop readings reaching InfluxDB.

Reads of pause.json fail open. If the file is missing, truncated or nonsense,
uploads continue. Uploads carrying on when they should have stopped is a
recoverable annoyance; uploads stopping because a file got mangled would take
the station offline without anyone noticing.

Deliberately stdlib-only, so the tests run anywhere without installing paho,
influxdb or requests.
"""

import json
import logging
import os
import tempfile
import time

PAUSE_FILE = "pause.json"
STATUS_FILE = "status.json"

# status.json is rewritten as readings arrive, which is several times a second.
# Publishing at most this often keeps the page live without hammering the SD
# card.
STATUS_MIN_WRITE_INTERVAL = 10.0


class AqicnState:
    def __init__(
        self,
        state_dir,
        station_id=None,
        min_upload_interval=None,
        status_min_write_interval=STATUS_MIN_WRITE_INTERVAL,
        clock=time.time,
    ):
        self.state_dir = state_dir
        self.station_id = station_id
        self.min_upload_interval = min_upload_interval
        self.status_min_write_interval = status_min_write_interval
        self.clock = clock

        self._latest = {}
        self._last_upload = None
        self._status_written_at = 0.0
        self._paused = None  # last value logged, so transitions log once

    # -- pause -----------------------------------------------------------

    def paused(self):
        """Whether uploads to AQICN are currently suppressed."""
        paused = self._read_paused()
        if paused != self._paused:
            logging.info("AQICN uploads %s", "paused" if paused else "resumed")
            self._paused = paused
        return paused

    def _read_paused(self):
        path = os.path.join(self.state_dir, PAUSE_FILE)
        try:
            with open(path) as handle:
                pause = json.load(handle)
        except FileNotFoundError:
            return False
        except (OSError, ValueError) as error:
            logging.warning("Ignoring unreadable %s: %s", path, error)
            return False

        try:
            paused_until = float(pause["paused_until"])
        except (KeyError, TypeError, ValueError):
            logging.warning("Ignoring %s: no usable paused_until", path)
            return False

        return self.clock() < paused_until

    # -- status ----------------------------------------------------------

    def record_reading(self, specie, value):
        """Note the newest value for one specie, whether or not it is uploaded.

        Called on every patio measurement including while paused, so the page
        keeps showing the smoke while nothing is being published.
        """
        self._latest[specie] = {"value": value, "at": int(self.clock())}
        self._write_status()

    def record_upload(self, readings, ok=True, error=None):
        """Note the outcome of an upload attempt and publish it right away."""
        self._last_upload = {
            "at": int(self.clock()),
            "ok": ok,
            "readings": readings,
            "error": error,
        }
        self._write_status(force=True)

    def _write_status(self, force=False):
        now = self.clock()
        if not force and now - self._status_written_at < self.status_min_write_interval:
            return

        payload = {
            "updated_at": int(now),
            "latest": self._latest,
            "last_upload": self._last_upload,
            "min_upload_interval": self.min_upload_interval,
            "station_id": self.station_id,
        }

        if self._write_atomic(STATUS_FILE, payload):
            self._status_written_at = now

    def _write_atomic(self, name, payload):
        """Write via a temp file plus rename, so no reader sees a partial file."""
        tmp_path = None
        try:
            os.makedirs(self.state_dir, exist_ok=True)
            handle_fd, tmp_path = tempfile.mkstemp(
                dir=self.state_dir, prefix=name + ".tmp"
            )
            with os.fdopen(handle_fd, "w") as handle:
                json.dump(payload, handle)
            os.chmod(tmp_path, 0o644)
            os.replace(tmp_path, os.path.join(self.state_dir, name))
            return True
        except OSError as error:
            logging.warning("Failed to write %s: %s", name, error)
            if tmp_path is not None:
                try:
                    os.unlink(tmp_path)
                except OSError:
                    pass
            return False
