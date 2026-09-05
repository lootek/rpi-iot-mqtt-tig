package main

import (
	"encoding/json"
	"log"
	"net/http"
	"time"

	"github.com/lootek/rpi-iot-mqtt-tig/central-stack/aqicn-ui/internal/state"
)

type server struct {
	store state.Store
	now   func() time.Time
}

// stateView is the single payload the page polls: pause state flattened
// alongside whatever the listener last published.
type stateView struct {
	Now              int64        `json:"now"`
	Paused           bool         `json:"paused"`
	PausedUntil      int64        `json:"paused_until"`
	RemainingSeconds int64        `json:"remaining_seconds"`
	Reason           string       `json:"reason,omitempty"`
	Status           state.Status `json:"status"`
}

type pauseRequest struct {
	Seconds int64  `json:"seconds"`
	Reason  string `json:"reason"`
}

func (s *server) routes() http.Handler {
	mux := http.NewServeMux()
	mux.HandleFunc("GET /{$}", s.handleIndex)
	mux.HandleFunc("GET /api/state", s.handleState)
	mux.HandleFunc("POST /api/pause", s.handlePause)
	mux.HandleFunc("POST /api/resume", s.handleResume)
	return mux
}

func (s *server) handleIndex(w http.ResponseWriter, r *http.Request) {
	w.Header().Set("Content-Type", "text/html; charset=utf-8")
	// The page is a handful of KB and changes only on redeploy, but a stale
	// copy would show a stale pause button, so never cache it.
	w.Header().Set("Cache-Control", "no-store")
	if _, err := w.Write(indexHTML); err != nil {
		log.Printf("writing index: %v", err)
	}
}

func (s *server) handleState(w http.ResponseWriter, r *http.Request) {
	now := s.now()

	// Both reads fail open: a bad file degrades one section of the page rather
	// than taking the whole thing down.
	pause, err := s.store.ReadPause()
	if err != nil {
		log.Printf("reading pause: %v", err)
	}
	status, err := s.store.ReadStatus()
	if err != nil {
		log.Printf("reading status: %v", err)
	}

	writeJSON(w, http.StatusOK, stateView{
		Now:              now.Unix(),
		Paused:           pause.Active(now),
		PausedUntil:      pause.PausedUntil,
		RemainingSeconds: int64(pause.Remaining(now).Seconds()),
		Reason:           pause.Reason,
		Status:           status,
	})
}

func (s *server) handlePause(w http.ResponseWriter, r *http.Request) {
	var req pauseRequest
	if err := json.NewDecoder(r.Body).Decode(&req); err != nil {
		http.Error(w, "malformed request body", http.StatusBadRequest)
		return
	}
	if req.Reason == "" {
		req.Reason = "campfire"
	}

	p, err := s.store.SetPause(req.Seconds, req.Reason, s.now())
	if err != nil {
		// SetPause only rejects a non-positive duration; a write failure would
		// be a 500, but on this box a failed write means the bind mount is gone
		// and there is nothing better to say than the error itself.
		http.Error(w, err.Error(), http.StatusBadRequest)
		return
	}

	log.Printf("uploads paused until %s (reason: %s)",
		time.Unix(p.PausedUntil, 0).Format(time.RFC3339), p.Reason)
	s.handleState(w, r)
}

func (s *server) handleResume(w http.ResponseWriter, r *http.Request) {
	if err := s.store.ClearPause(); err != nil {
		http.Error(w, err.Error(), http.StatusInternalServerError)
		return
	}

	log.Print("uploads resumed")
	s.handleState(w, r)
}

func writeJSON(w http.ResponseWriter, code int, v any) {
	w.Header().Set("Content-Type", "application/json")
	w.WriteHeader(code)
	if err := json.NewEncoder(w).Encode(v); err != nil {
		log.Printf("encoding response: %v", err)
	}
}
