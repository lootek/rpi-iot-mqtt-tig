# msze-collector

Holy Mass schedule API for the Grafana Infinity datasource. A nightly job reads
the church pages and bulletins (HTML and PDF), has an LLM (OpenRouter) extract
the regular schedule and one-off changes into a JSON cache, and the HTTP API
computes the upcoming Masses from that cache per request — no network and no
LLM at view time.

## Endpoints

```sh
curl '127.0.0.1:8091/upcoming?hours=3'   # Masses starting within the window
curl 127.0.0.1:8091/health               # heartbeat, cache ages, config gaps
```

`/upcoming` → `{"generated_at", "window_hours", "rows": [...]}`. `hours` is
clamped to 1..12, default 3 (garbage → 3). Rows are sorted by start time (ties
by `eta_max`) with `no_data` rows ("brak danych") for churches without a usable
schedule at the end. An empty window is `rows: []` with HTTP 200. Row keys:
`start, start_hm, church, mode, eta, eta_min, eta_max, leave_in_min, status
(ok|late|no_data), badges, data_age_days, changed, note`.

`/health` → `{"heartbeat", "heartbeat_age_h", "sources": {id: {"regular_age_days",
"exceptions_age_days", "error"}}, "unconfirmed_eta": [church ids]}`. Always 200,
also without a cache (`heartbeat: null`). `unconfirmed_eta` lists churches whose
ETA in `churches.yaml` is still a placeholder (`eta_confirmed: false`).

## Config

`churches.yaml` (baked into the image, edit and redeploy): per church name,
`mode` (car/walk), ETA range, `eta_confirmed`, URLs, optional `location` for
multi-location pages, `schedule_from` for a filia, per-church HTTP headers.

| env | default | |
|---|---|---|
| `PORT` | `8091` | |
| `DATA_DIR` | `/data` | holds `cache.json` (named volume `msze-data`) |
| `CHURCHES` | `/app/churches.yaml` | |
| `OPENROUTER_API_KEY` | – | only needed by the scheduler / dry run |
| `MSZE_SCHEDULER` | `0` | `1` starts the nightly job (~03:00 Europe/Warsaw) in a daemon thread |

With `MSZE_SCHEDULER=1` but no key the container logs an error and keeps
serving without the scheduler.

## Secret

The OpenRouter key goes only in `env/msze.secret` on the host, mode 600, never
committed (`env/*.secret` is gitignored and blocked by `hooks/pre-commit`):

```sh
cp env/msze.secret.example env/msze.secret && chmod 600 env/msze.secret
$EDITOR env/msze.secret          # OPENROUTER_API_KEY=...
```

The compose file loads it as an optional `env_file` (compose >= 2.24); the
container starts without it.

## Deploy (lothlorien)

```sh
git pull
docker compose -f central-stack/docker-compose.msze.yml up -d --build
curl -s localhost:8091/health
```

Redeploy after editing `churches.yaml` or code is the same command. The cache
survives in the `msze-data` volume.

## Dry run, then enable the scheduler

```sh
docker exec msze-collector python job.py --once --force-schedule --dry-run
```

Prints what would be extracted, writes nothing. Review it, then run without
`--dry-run` once, and finally set `MSZE_SCHEDULER=1` in the compose file and
`docker compose ... up -d` to recreate the container.

## Tests

```sh
python -m venv .venv && .venv/bin/pip install -r requirements-dev.txt
.venv/bin/python -m pytest -q
```

`requirements.txt` is the runtime set (used by the Dockerfile); pytest lives in
`requirements-dev.txt`.
