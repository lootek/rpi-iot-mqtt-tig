"""Tests for aqicn_state, the file-backed handshake with aqicn-ui.

Stdlib only, so this runs on the Pi as well as on a laptop:
    python3 -m unittest discover -s central-stack -p 'test_*.py'
"""

import json
import os
import tempfile
import unittest

from aqicn_state import AqicnState


class FakeClock:
    """A clock the tests move by hand, so throttling and expiry are exact."""

    def __init__(self, now=1757012100.0):
        self.now = now

    def __call__(self):
        return self.now

    def advance(self, seconds):
        self.now += seconds


class AqicnStateTestCase(unittest.TestCase):
    def setUp(self):
        self._dir = tempfile.TemporaryDirectory()
        self.addCleanup(self._dir.cleanup)
        self.dir = self._dir.name
        self.clock = FakeClock()
        self.state = AqicnState(
            self.dir,
            station_id="brzo-gorska-001",
            min_upload_interval=300.0,
            clock=self.clock,
        )

    def write_pause(self, payload):
        with open(os.path.join(self.dir, "pause.json"), "w") as handle:
            handle.write(payload)

    def read_status(self):
        with open(os.path.join(self.dir, "status.json")) as handle:
            return json.load(handle)


class TestPaused(AqicnStateTestCase):
    def test_not_paused_when_no_file(self):
        self.assertFalse(self.state.paused())

    def test_paused_while_paused_until_is_in_the_future(self):
        self.write_pause(json.dumps({"paused_until": self.clock.now + 60}))

        self.assertTrue(self.state.paused())

    def test_not_paused_once_paused_until_has_passed(self):
        self.write_pause(json.dumps({"paused_until": self.clock.now + 60}))
        self.assertTrue(self.state.paused())

        self.clock.advance(61)

        self.assertFalse(self.state.paused())

    def test_corrupt_pause_file_fails_open(self):
        """Uploads silently stopping is worse than a campfire slipping through."""
        self.write_pause("{ this is not json")

        self.assertFalse(self.state.paused())

    def test_pause_file_without_paused_until_fails_open(self):
        self.write_pause(json.dumps({"reason": "campfire"}))

        self.assertFalse(self.state.paused())

    def test_non_numeric_paused_until_fails_open(self):
        self.write_pause(json.dumps({"paused_until": "tonight"}))

        self.assertFalse(self.state.paused())

    def test_pause_file_read_is_not_cached_across_changes(self):
        """The UI writes this file while the listener is running."""
        self.assertFalse(self.state.paused())

        self.write_pause(json.dumps({"paused_until": self.clock.now + 60}))

        self.assertTrue(self.state.paused())


class TestRecordReading(AqicnStateTestCase):
    def test_reading_is_published_under_its_aqicn_specie(self):
        self.state.record_reading("pm25", 96.4)

        status = self.read_status()
        self.assertEqual(status["latest"]["pm25"]["value"], 96.4)
        self.assertEqual(status["latest"]["pm25"]["at"], int(self.clock.now))

    def test_status_carries_station_and_interval(self):
        self.state.record_reading("pm25", 2.3)

        status = self.read_status()
        self.assertEqual(status["station_id"], "brzo-gorska-001")
        self.assertEqual(status["min_upload_interval"], 300.0)

    def test_token_is_never_written_to_the_status_file(self):
        """status.json sits in a bind mount; the repo is public."""
        self.state.record_reading("pm25", 2.3)

        with open(os.path.join(self.dir, "status.json")) as handle:
            raw = handle.read()
        self.assertNotIn("token", raw.lower())

    def test_writes_are_throttled(self):
        self.state.record_reading("pm25", 1.0)
        self.clock.advance(3)

        self.state.record_reading("pm25", 2.0)

        self.assertEqual(self.read_status()["latest"]["pm25"]["value"], 1.0)

    def test_write_resumes_after_the_throttle_window(self):
        self.state.record_reading("pm25", 1.0)
        self.clock.advance(11)

        self.state.record_reading("pm25", 2.0)

        self.assertEqual(self.read_status()["latest"]["pm25"]["value"], 2.0)

    def test_readings_are_kept_even_when_the_write_is_throttled(self):
        """A throttled write must not lose the value, only defer publishing it."""
        self.state.record_reading("pm25", 1.0)
        self.state.record_reading("pm10", 5.0)
        self.clock.advance(11)

        self.state.record_reading("temp", 20.0)

        latest = self.read_status()["latest"]
        self.assertEqual(latest["pm10"]["value"], 5.0)
        self.assertEqual(latest["temp"]["value"], 20.0)

    def test_no_temp_files_are_left_behind(self):
        """The Go side polls this file; a partial write must never be visible."""
        self.state.record_reading("pm25", 2.3)

        self.assertEqual(sorted(os.listdir(self.dir)), ["status.json"])


class TestRecordUpload(AqicnStateTestCase):
    def test_successful_upload_is_published_immediately(self):
        """Bypasses the throttle: the receipt is the thing you came to check."""
        readings = [{"specie": "pm25", "value": 2.3}]

        self.state.record_upload(readings, ok=True)

        upload = self.read_status()["last_upload"]
        self.assertTrue(upload["ok"])
        self.assertEqual(upload["at"], int(self.clock.now))
        self.assertEqual(upload["readings"], readings)

    def test_failed_upload_records_the_error(self):
        self.state.record_upload([], ok=False, error="503 Service Unavailable")

        upload = self.read_status()["last_upload"]
        self.assertFalse(upload["ok"])
        self.assertEqual(upload["error"], "503 Service Unavailable")

    def test_last_upload_is_null_before_anything_is_sent(self):
        self.state.record_reading("pm25", 2.3)

        self.assertIsNone(self.read_status()["last_upload"])

    def test_readings_keep_updating_while_paused(self):
        """The whole point: the page must show the smoke spike during a pause."""
        self.write_pause(json.dumps({"paused_until": self.clock.now + 3600}))
        self.state.record_upload([{"specie": "pm25", "value": 2.3}], ok=True)

        self.clock.advance(11)
        self.state.record_reading("pm25", 96.4)

        status = self.read_status()
        self.assertTrue(self.state.paused())
        self.assertEqual(status["latest"]["pm25"]["value"], 96.4)
        self.assertEqual(status["last_upload"]["readings"][0]["value"], 2.3)


class TestResilience(AqicnStateTestCase):
    def test_missing_state_directory_is_created(self):
        nested = os.path.join(self.dir, "does", "not", "exist")
        state = AqicnState(nested, clock=self.clock)

        state.record_reading("pm25", 2.3)

        self.assertTrue(os.path.exists(os.path.join(nested, "status.json")))

    def test_unwritable_state_directory_does_not_raise(self):
        """A broken bind mount must not take down the Influx write path."""
        state = AqicnState(os.path.join(self.dir, "nope"), clock=self.clock)
        os.chmod(self.dir, 0o500)
        self.addCleanup(os.chmod, self.dir, 0o700)

        state.record_reading("pm25", 2.3)  # must not raise

    def test_unreadable_pause_file_fails_open(self):
        self.write_pause(json.dumps({"paused_until": self.clock.now + 60}))
        os.chmod(os.path.join(self.dir, "pause.json"), 0o000)
        self.addCleanup(os.chmod, os.path.join(self.dir, "pause.json"), 0o600)

        self.assertFalse(self.state.paused())


if __name__ == "__main__":
    unittest.main()
