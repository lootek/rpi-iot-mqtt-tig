package main

import (
	"encoding/json"
	"net/http"
	"net/http/httptest"
	"os"
	"path/filepath"
	"strings"
	"testing"
	"time"

	"github.com/lootek/rpi-iot-mqtt-tig/central-stack/aqicn-ui/internal/state"
)

var testNow = time.Unix(1757012100, 0)

func newServer(t *testing.T) *server {
	t.Helper()
	return &server{
		store: state.Store{Dir: t.TempDir()},
		now:   func() time.Time { return testNow },
	}
}

func do(t *testing.T, s *server, method, path, body string) *httptest.ResponseRecorder {
	t.Helper()
	req := httptest.NewRequest(method, path, strings.NewReader(body))
	rec := httptest.NewRecorder()
	s.routes().ServeHTTP(rec, req)
	return rec
}

func decodeView(t *testing.T, rec *httptest.ResponseRecorder) stateView {
	t.Helper()
	var v stateView
	if err := json.Unmarshal(rec.Body.Bytes(), &v); err != nil {
		t.Fatalf("decoding %s: %v", rec.Body.String(), err)
	}
	return v
}

func TestStateEndpointReportsNotPausedWhenNothingWritten(t *testing.T) {
	s := newServer(t)

	rec := do(t, s, http.MethodGet, "/api/state", "")

	if rec.Code != http.StatusOK {
		t.Fatalf("status = %d, want 200", rec.Code)
	}
	v := decodeView(t, rec)
	if v.Paused {
		t.Error("reported paused with no pause file")
	}
	if v.RemainingSeconds != 0 {
		t.Errorf("RemainingSeconds = %d, want 0", v.RemainingSeconds)
	}
	if v.Now != testNow.Unix() {
		t.Errorf("Now = %d, want %d", v.Now, testNow.Unix())
	}
}

func TestPauseThenStateReportsRemaining(t *testing.T) {
	s := newServer(t)

	if rec := do(t, s, http.MethodPost, "/api/pause", `{"seconds":3600}`); rec.Code != http.StatusOK {
		t.Fatalf("pause status = %d, body %s", rec.Code, rec.Body)
	}

	v := decodeView(t, do(t, s, http.MethodGet, "/api/state", ""))
	if !v.Paused {
		t.Fatal("not paused after POST /api/pause")
	}
	if v.RemainingSeconds != 3600 {
		t.Errorf("RemainingSeconds = %d, want 3600", v.RemainingSeconds)
	}
	if v.PausedUntil != testNow.Unix()+3600 {
		t.Errorf("PausedUntil = %d, want %d", v.PausedUntil, testNow.Unix()+3600)
	}
}

func TestPauseRejectsNonPositiveSeconds(t *testing.T) {
	s := newServer(t)

	rec := do(t, s, http.MethodPost, "/api/pause", `{"seconds":0}`)

	if rec.Code != http.StatusBadRequest {
		t.Errorf("status = %d, want 400", rec.Code)
	}
}

func TestPauseRejectsMalformedBody(t *testing.T) {
	s := newServer(t)

	rec := do(t, s, http.MethodPost, "/api/pause", `{"seconds":`)

	if rec.Code != http.StatusBadRequest {
		t.Errorf("status = %d, want 400", rec.Code)
	}
}

func TestResumeClearsAnActivePause(t *testing.T) {
	s := newServer(t)
	if rec := do(t, s, http.MethodPost, "/api/pause", `{"seconds":3600}`); rec.Code != http.StatusOK {
		t.Fatalf("pause status = %d", rec.Code)
	}

	if rec := do(t, s, http.MethodPost, "/api/resume", ""); rec.Code != http.StatusOK {
		t.Fatalf("resume status = %d, body %s", rec.Code, rec.Body)
	}

	if v := decodeView(t, do(t, s, http.MethodGet, "/api/state", "")); v.Paused {
		t.Error("still paused after resume")
	}
}

// Pressing resume twice happens; the second press must not error.
func TestResumeIsIdempotent(t *testing.T) {
	s := newServer(t)

	do(t, s, http.MethodPost, "/api/resume", "")
	rec := do(t, s, http.MethodPost, "/api/resume", "")

	if rec.Code != http.StatusOK {
		t.Errorf("second resume status = %d, want 200", rec.Code)
	}
}

func TestPauseRejectsGet(t *testing.T) {
	s := newServer(t)

	rec := do(t, s, http.MethodGet, "/api/pause", "")

	if rec.Code != http.StatusMethodNotAllowed {
		t.Errorf("status = %d, want 405", rec.Code)
	}
}

// The listener's status.json must surface through the API untouched, since the
// page renders live readings straight from it.
func TestStateEndpointSurfacesListenerStatus(t *testing.T) {
	s := newServer(t)
	raw := `{"updated_at":1757012400,
	         "latest":{"pm25":{"value":42.5,"at":1757012395}},
	         "last_upload":{"at":1757012100,"ok":true,
	                        "readings":[{"specie":"pm25","value":2.3}]},
	         "min_upload_interval":300,
	         "station_id":"brzo-gorska-001"}`
	if err := os.WriteFile(filepath.Join(s.store.Dir, "status.json"), []byte(raw), 0o644); err != nil {
		t.Fatal(err)
	}

	v := decodeView(t, do(t, s, http.MethodGet, "/api/state", ""))

	if v.Status.StationID != "brzo-gorska-001" {
		t.Errorf("StationID = %q", v.Status.StationID)
	}
	if got := v.Status.Latest["pm25"]; got.Value != 42.5 {
		t.Errorf("pm25 = %+v, want 42.5", got)
	}
	if v.Status.LastUpload == nil || !v.Status.LastUpload.OK {
		t.Errorf("LastUpload = %+v", v.Status.LastUpload)
	}
}

// A corrupt status.json must not take the pause controls down with it — that is
// exactly when you need the button.
func TestStateEndpointStillServesWhenStatusIsCorrupt(t *testing.T) {
	s := newServer(t)
	if err := os.WriteFile(filepath.Join(s.store.Dir, "status.json"), []byte("{oops"), 0o644); err != nil {
		t.Fatal(err)
	}
	if rec := do(t, s, http.MethodPost, "/api/pause", `{"seconds":3600}`); rec.Code != http.StatusOK {
		t.Fatalf("pause status = %d", rec.Code)
	}

	rec := do(t, s, http.MethodGet, "/api/state", "")

	if rec.Code != http.StatusOK {
		t.Fatalf("status = %d, want 200", rec.Code)
	}
	if v := decodeView(t, rec); !v.Paused {
		t.Error("pause state lost because status.json was corrupt")
	}
}

func TestIndexServesHTML(t *testing.T) {
	s := newServer(t)

	rec := do(t, s, http.MethodGet, "/", "")

	if rec.Code != http.StatusOK {
		t.Fatalf("status = %d, want 200", rec.Code)
	}
	if ct := rec.Header().Get("Content-Type"); !strings.HasPrefix(ct, "text/html") {
		t.Errorf("Content-Type = %q, want text/html", ct)
	}
	if !strings.Contains(rec.Body.String(), "<html") {
		t.Error("body does not look like an HTML document")
	}
}

func TestUnknownPathIs404(t *testing.T) {
	s := newServer(t)

	if rec := do(t, s, http.MethodGet, "/nope", ""); rec.Code != http.StatusNotFound {
		t.Errorf("status = %d, want 404", rec.Code)
	}
}
