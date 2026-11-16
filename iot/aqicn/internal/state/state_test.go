package state

import (
	"encoding/json"
	"os"
	"path/filepath"
	"testing"
	"time"
)

func newStore(t *testing.T) Store {
	t.Helper()
	return Store{Dir: t.TempDir()}
}

// A missing pause.json is the normal steady state: the station has never been
// paused. It must read as "not paused" rather than as an error the caller might
// mistake for a pause.
func TestReadPauseMissingFileIsNotPaused(t *testing.T) {
	s := newStore(t)

	p, err := s.ReadPause()
	if err != nil {
		t.Fatalf("ReadPause on empty dir: unexpected error %v", err)
	}
	if p.Active(time.Now()) {
		t.Errorf("missing pause.json reported as active: %+v", p)
	}
}

func TestSetPauseThenReadRoundTrip(t *testing.T) {
	s := newStore(t)
	now := time.Unix(1757012100, 0)

	written, err := s.SetPause(3600, "campfire", now)
	if err != nil {
		t.Fatalf("SetPause: %v", err)
	}
	if want := now.Unix() + 3600; written.PausedUntil != want {
		t.Errorf("PausedUntil = %d, want %d", written.PausedUntil, want)
	}

	read, err := s.ReadPause()
	if err != nil {
		t.Fatalf("ReadPause: %v", err)
	}
	if read != written {
		t.Errorf("round trip mismatch: read %+v, wrote %+v", read, written)
	}
	if read.Reason != "campfire" {
		t.Errorf("Reason = %q, want %q", read.Reason, "campfire")
	}
	if !read.Active(now) {
		t.Error("freshly set pause is not active at set time")
	}
}

// Expiry is evaluated from the absolute PausedUntil instant, so it works across
// restarts of either container without a timer.
func TestPauseActiveOnlyBeforePausedUntil(t *testing.T) {
	until := time.Unix(1757015700, 0)
	p := Pause{PausedUntil: until.Unix(), SetAt: 1757012100}

	if !p.Active(until.Add(-time.Second)) {
		t.Error("pause should be active one second before expiry")
	}
	if p.Active(until) {
		t.Error("pause should not be active at the expiry instant")
	}
	if p.Active(until.Add(time.Hour)) {
		t.Error("pause should not be active after expiry")
	}
}

func TestRemainingIsZeroOnceExpired(t *testing.T) {
	until := time.Unix(1757015700, 0)
	p := Pause{PausedUntil: until.Unix()}

	if got := p.Remaining(until.Add(-90 * time.Second)); got != 90*time.Second {
		t.Errorf("Remaining = %v, want 90s", got)
	}
	if got := p.Remaining(until.Add(time.Hour)); got != 0 {
		t.Errorf("Remaining after expiry = %v, want 0", got)
	}
}

// A corrupt pause file must fail open. Uploads stopping silently because the UI
// wrote garbage is far worse than a campfire slipping through.
func TestReadPauseCorruptFileFailsOpen(t *testing.T) {
	s := newStore(t)
	if err := os.WriteFile(filepath.Join(s.Dir, pauseFile), []byte("{not json"), 0o644); err != nil {
		t.Fatal(err)
	}

	p, err := s.ReadPause()
	if err == nil {
		t.Error("expected an error describing the corrupt file")
	}
	if p.Active(time.Now()) {
		t.Errorf("corrupt pause.json reported as active: %+v", p)
	}
}

func TestSetPauseCapsAtMaxPause(t *testing.T) {
	s := newStore(t)
	now := time.Unix(1757012100, 0)

	p, err := s.SetPause(int64((72 * time.Hour).Seconds()), "oops", now)
	if err != nil {
		t.Fatalf("SetPause: %v", err)
	}
	if want := now.Unix() + int64(MaxPause.Seconds()); p.PausedUntil != want {
		t.Errorf("PausedUntil = %d, want capped %d", p.PausedUntil, want)
	}
}

func TestSetPauseRejectsNonPositiveDuration(t *testing.T) {
	s := newStore(t)
	now := time.Unix(1757012100, 0)

	for _, seconds := range []int64{0, -1} {
		if _, err := s.SetPause(seconds, "", now); err == nil {
			t.Errorf("SetPause(%d) succeeded, want error", seconds)
		}
	}
}

func TestClearPauseMakesItInactive(t *testing.T) {
	s := newStore(t)
	now := time.Unix(1757012100, 0)
	if _, err := s.SetPause(3600, "campfire", now); err != nil {
		t.Fatal(err)
	}

	if err := s.ClearPause(); err != nil {
		t.Fatalf("ClearPause: %v", err)
	}

	p, err := s.ReadPause()
	if err != nil {
		t.Fatalf("ReadPause after clear: %v", err)
	}
	if p.Active(now) {
		t.Errorf("pause still active after ClearPause: %+v", p)
	}
}

func TestClearPauseOnMissingFileSucceeds(t *testing.T) {
	s := newStore(t)
	if err := s.ClearPause(); err != nil {
		t.Errorf("ClearPause with no pause file: %v", err)
	}
}

func TestReadStatusMissingFileIsZero(t *testing.T) {
	s := newStore(t)

	st, err := s.ReadStatus()
	if err != nil {
		t.Fatalf("ReadStatus on empty dir: unexpected error %v", err)
	}
	if st.UpdatedAt != 0 || len(st.Latest) != 0 {
		t.Errorf("expected zero status, got %+v", st)
	}
}

// The listener writes this file; the contract is the JSON shape, so pin it.
func TestReadStatusParsesListenerShape(t *testing.T) {
	s := newStore(t)
	raw := `{
	  "updated_at": 1757012400,
	  "latest": {
	    "pm25": {"value": 2.3, "at": 1757012395},
	    "temp": {"value": 23.6, "at": 1757012390}
	  },
	  "last_upload": {
	    "at": 1757012100,
	    "ok": true,
	    "readings": [{"specie": "pm25", "value": 2.3}],
	    "error": null
	  },
	  "min_upload_interval": 300,
	  "station_id": "brzo-gorska-001"
	}`
	if err := os.WriteFile(filepath.Join(s.Dir, statusFile), []byte(raw), 0o644); err != nil {
		t.Fatal(err)
	}

	st, err := s.ReadStatus()
	if err != nil {
		t.Fatalf("ReadStatus: %v", err)
	}
	if st.StationID != "brzo-gorska-001" {
		t.Errorf("StationID = %q", st.StationID)
	}
	if st.MinUploadInterval != 300 {
		t.Errorf("MinUploadInterval = %v, want 300", st.MinUploadInterval)
	}
	if got := st.Latest["pm25"]; got.Value != 2.3 || got.At != 1757012395 {
		t.Errorf("latest pm25 = %+v", got)
	}
	if st.LastUpload == nil {
		t.Fatal("LastUpload is nil")
	}
	if !st.LastUpload.OK || st.LastUpload.At != 1757012100 {
		t.Errorf("LastUpload = %+v", st.LastUpload)
	}
	if len(st.LastUpload.Readings) != 1 || st.LastUpload.Readings[0].Specie != "pm25" {
		t.Errorf("LastUpload.Readings = %+v", st.LastUpload.Readings)
	}
}

func TestReadStatusCorruptFileReturnsZeroAndError(t *testing.T) {
	s := newStore(t)
	if err := os.WriteFile(filepath.Join(s.Dir, statusFile), []byte("half a fil"), 0o644); err != nil {
		t.Fatal(err)
	}

	st, err := s.ReadStatus()
	if err == nil {
		t.Error("expected an error for corrupt status.json")
	}
	if st.UpdatedAt != 0 || len(st.Latest) != 0 {
		t.Errorf("expected zero status on corrupt read, got %+v", st)
	}
}

// The Python listener polls pause.json continuously. A half-written file must
// never be observable, so writes go via a temp file plus rename, and no temp
// files may be left lying around.
func TestSetPauseLeavesNoTempFiles(t *testing.T) {
	s := newStore(t)
	if _, err := s.SetPause(3600, "campfire", time.Unix(1757012100, 0)); err != nil {
		t.Fatal(err)
	}

	entries, err := os.ReadDir(s.Dir)
	if err != nil {
		t.Fatal(err)
	}
	if len(entries) != 1 || entries[0].Name() != pauseFile {
		names := make([]string, len(entries))
		for i, e := range entries {
			names[i] = e.Name()
		}
		t.Errorf("dir contains %v, want only %s", names, pauseFile)
	}
}

// The Python side reads this file, so keep the wire format stable.
func TestPauseFileUsesSnakeCaseKeys(t *testing.T) {
	s := newStore(t)
	if _, err := s.SetPause(3600, "campfire", time.Unix(1757012100, 0)); err != nil {
		t.Fatal(err)
	}

	raw, err := os.ReadFile(filepath.Join(s.Dir, pauseFile))
	if err != nil {
		t.Fatal(err)
	}
	var decoded map[string]any
	if err := json.Unmarshal(raw, &decoded); err != nil {
		t.Fatalf("pause.json is not valid JSON: %v", err)
	}
	for _, key := range []string{"paused_until", "set_at", "reason"} {
		if _, ok := decoded[key]; !ok {
			t.Errorf("pause.json missing key %q; got %v", key, decoded)
		}
	}
}
