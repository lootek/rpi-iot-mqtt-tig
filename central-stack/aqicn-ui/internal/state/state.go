// Package state reads and writes the two small JSON files that aqicn-ui and
// the MQTT listener use to talk to each other across container boundaries.
//
// The listener owns status.json; aqicn-ui owns pause.json. Neither ever writes
// the other's file, so there is one writer per file and no locking is needed.
//
// Reads fail open on purpose. If pause.json is missing, truncated or garbage,
// ReadPause returns the zero Pause alongside the error: uploads continuing when
// they should have stopped is a recoverable annoyance, whereas uploads stopping
// because a file got mangled would take the station offline unnoticed.
package state

import (
	"encoding/json"
	"errors"
	"fmt"
	"os"
	"path/filepath"
	"time"
)

const (
	pauseFile  = "pause.json"
	statusFile = "status.json"
)

// MaxPause caps how long a single pause can last, so a forgotten campfire
// cannot silently take the station offline for days.
const MaxPause = 24 * time.Hour

// Pause is the contract with mqtt-listener.py. PausedUntil is an absolute Unix
// timestamp rather than a duration so that expiry needs no timer on either side
// and survives a restart of either container.
type Pause struct {
	PausedUntil int64  `json:"paused_until"`
	SetAt       int64  `json:"set_at"`
	Reason      string `json:"reason"`
}

// Active reports whether uploads should be suppressed at now.
func (p Pause) Active(now time.Time) bool { return now.Unix() < p.PausedUntil }

// Remaining is how much of the pause is left, or zero once it has expired.
func (p Pause) Remaining(now time.Time) time.Duration {
	if !p.Active(now) {
		return 0
	}
	return time.Duration(p.PausedUntil-now.Unix()) * time.Second
}

// Reading is the most recent value seen for one specie, whether or not it was
// ever uploaded.
type Reading struct {
	Value float64 `json:"value"`
	At    int64   `json:"at"`
}

// Species is one entry of an AQICN upload payload.
type Species struct {
	Specie string  `json:"specie"`
	Value  float64 `json:"value"`
}

// Upload records the outcome of the last attempt to push to AQICN.
type Upload struct {
	At       int64     `json:"at"`
	OK       bool      `json:"ok"`
	Readings []Species `json:"readings"`
	Error    string    `json:"error,omitempty"`
}

// Status is written by the listener. Latest updates on every patio measurement,
// including while paused, so the page keeps showing the smoke spike even when
// nothing is being uploaded.
type Status struct {
	UpdatedAt         int64              `json:"updated_at"`
	Latest            map[string]Reading `json:"latest"`
	LastUpload        *Upload            `json:"last_upload"`
	MinUploadInterval float64            `json:"min_upload_interval"`
	StationID         string             `json:"station_id"`
}

// Store is the shared state directory, bind-mounted into both containers.
type Store struct{ Dir string }

// ReadPause returns the current pause. The returned Pause is never active when
// err is non-nil, so a caller that logs the error and uses the value still fails
// open.
func (s Store) ReadPause() (Pause, error) {
	var p Pause
	if err := s.readJSON(pauseFile, &p); err != nil {
		return Pause{}, err
	}
	return p, nil
}

// SetPause pauses uploads for seconds from now, clamped to MaxPause.
func (s Store) SetPause(seconds int64, reason string, now time.Time) (Pause, error) {
	if seconds <= 0 {
		return Pause{}, fmt.Errorf("pause duration must be positive, got %ds", seconds)
	}
	if max := int64(MaxPause.Seconds()); seconds > max {
		seconds = max
	}

	p := Pause{
		PausedUntil: now.Unix() + seconds,
		SetAt:       now.Unix(),
		Reason:      reason,
	}
	if err := s.writeJSONAtomic(pauseFile, p); err != nil {
		return Pause{}, err
	}
	return p, nil
}

// ClearPause resumes uploads immediately. Clearing an already-absent pause is
// not an error, so the resume button is idempotent.
func (s Store) ClearPause() error {
	err := os.Remove(filepath.Join(s.Dir, pauseFile))
	if err != nil && !errors.Is(err, os.ErrNotExist) {
		return err
	}
	return nil
}

// ReadStatus returns what the listener last published. A zero Status means the
// listener has not written yet; the caller should render that as "no data".
func (s Store) ReadStatus() (Status, error) {
	var st Status
	if err := s.readJSON(statusFile, &st); err != nil {
		return Status{}, err
	}
	return st, nil
}

// readJSON leaves dst untouched and returns nil when the file does not exist,
// which is the expected state before either side has written anything.
func (s Store) readJSON(name string, dst any) error {
	raw, err := os.ReadFile(filepath.Join(s.Dir, name))
	if errors.Is(err, os.ErrNotExist) {
		return nil
	}
	if err != nil {
		return err
	}
	if err := json.Unmarshal(raw, dst); err != nil {
		return fmt.Errorf("parsing %s: %w", name, err)
	}
	return nil
}

// writeJSONAtomic writes via a temp file in the same directory plus a rename,
// so the listener polling this file never observes a partial write.
func (s Store) writeJSONAtomic(name string, v any) error {
	raw, err := json.Marshal(v)
	if err != nil {
		return err
	}

	tmp, err := os.CreateTemp(s.Dir, name+".tmp*")
	if err != nil {
		return err
	}
	defer os.Remove(tmp.Name()) // no-op once the rename below succeeds

	if _, err := tmp.Write(raw); err != nil {
		tmp.Close()
		return err
	}
	if err := tmp.Close(); err != nil {
		return err
	}
	// The listener runs as a different user than this process may; keep the
	// file world-readable rather than inheriting CreateTemp's 0600.
	if err := os.Chmod(tmp.Name(), 0o644); err != nil {
		return err
	}
	return os.Rename(tmp.Name(), filepath.Join(s.Dir, name))
}
