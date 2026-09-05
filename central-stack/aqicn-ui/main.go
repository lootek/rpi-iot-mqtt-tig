// Command aqicn-ui serves a small LAN page for pausing AQICN uploads from the
// patio sensor, for when a backyard campfire would otherwise publish smoke as
// if it were ambient air quality.
//
// It holds no credentials and talks to nothing over the network. All it does is
// read the status file the MQTT listener publishes and write the pause file the
// listener honours, both in a shared bind-mounted directory.
package main

import (
	_ "embed"
	"log"
	"net/http"
	"os"
	"time"

	"github.com/lootek/rpi-iot-mqtt-tig/central-stack/aqicn-ui/internal/state"
)

//go:embed web/index.html
var indexHTML []byte

func main() {
	log.SetFlags(log.LstdFlags | log.LUTC)

	dir := env("AQICN_STATE_DIR", "/state")
	addr := env("AQICN_UI_ADDR", ":8080")

	if err := os.MkdirAll(dir, 0o755); err != nil {
		log.Fatalf("state directory %s: %v", dir, err)
	}

	srv := &server{
		store: state.Store{Dir: dir},
		now:   time.Now,
	}

	// Timeouts are modest on purpose: every handler is a small local file read,
	// so anything slow is a stuck client rather than real work.
	httpSrv := &http.Server{
		Addr:              addr,
		Handler:           srv.routes(),
		ReadHeaderTimeout: 5 * time.Second,
		ReadTimeout:       10 * time.Second,
		WriteTimeout:      10 * time.Second,
		IdleTimeout:       60 * time.Second,
	}

	log.Printf("aqicn-ui listening on %s, state in %s", addr, dir)

	log.Fatal(httpSrv.ListenAndServe())
}

func env(key, fallback string) string {
	if v := os.Getenv(key); v != "" {
		return v
	}
	return fallback
}
