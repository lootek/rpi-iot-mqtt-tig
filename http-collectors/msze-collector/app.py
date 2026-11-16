#!/usr/bin/env python3
"""msze-collector HTTP API for the Grafana Infinity datasource.

GET /upcoming?hours=3  rows for Masses starting within the window (1..12 h, default 3)
GET /health            heartbeat, per-source cache ages/errors, churches with unconfirmed ETA

Every request reloads the JSON cache from disk (the nightly job replaces it atomically)
and computes the rows on the spot: no network, no LLM at view time. Entrypoint also runs
the nightly scheduler in a daemon thread when MSZE_SCHEDULER=1.
"""
from __future__ import annotations

import json
import logging
import os
import threading
from datetime import datetime, timezone
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlsplit
from zoneinfo import ZoneInfo

from config import Church, load_churches
from schedule import _age, build_rows
from store import Store

log = logging.getLogger("msze.app")

WARSAW = ZoneInfo("Europe/Warsaw")
DEFAULT_HOURS, MIN_HOURS, MAX_HOURS = 3, 1, 12
JSON_TYPE = "application/json; charset=utf-8"


def parse_hours(query: str) -> int:
    raw = (parse_qs(query).get("hours") or [""])[0].strip()
    try:
        hours = int(raw)
    except ValueError:
        return DEFAULT_HOURS
    return max(MIN_HOURS, min(MAX_HOURS, hours))


def _days(fetched_at: str | None, now: datetime) -> float | None:
    age = _age(fetched_at, now)
    return None if age is None else round(age.total_seconds() / 86400, 2)


def health(now: datetime, churches: list[Church], cache: dict) -> dict:
    hb = cache.get("heartbeat")
    hb_age = _age(hb, now)
    sources = {}
    for sid in dict.fromkeys(c.source_id for c in churches):
        src = (cache.get("sources") or {}).get(sid) or {}
        reg, exc = src.get("regular") or {}, src.get("exceptions") or {}
        sources[sid] = {
            "regular_age_days": _days(reg.get("fetched_at"), now),
            "exceptions_age_days": _days(exc.get("fetched_at"), now),
            "error": reg.get("error") or exc.get("error"),
        }
    return {
        "heartbeat": hb,
        "heartbeat_age_h": None if hb_age is None else round(hb_age.total_seconds() / 3600, 2),
        "sources": sources,
        "unconfirmed_eta": [c.id for c in churches if not c.eta_confirmed],
    }


def make_server(port: int, store: Store, churches: list[Church], now_fn=None,
                host: str = "") -> ThreadingHTTPServer:
    now_fn = now_fn or (lambda: datetime.now(WARSAW))

    class Handler(BaseHTTPRequestHandler):
        def _send(self, status: int, payload) -> None:
            body = json.dumps(payload, ensure_ascii=False).encode("utf-8")
            self.send_response(status)
            self.send_header("Content-Type", JSON_TYPE)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):  # noqa: N802 (http.server API)
            url = urlsplit(self.path)
            try:
                if url.path == "/upcoming":
                    now, hours = now_fn(), parse_hours(url.query)
                    rows = build_rows(now, churches, store.load(), hours)
                    self._send(200, {"generated_at": now.isoformat(), "window_hours": hours,
                                     "rows": rows})
                elif url.path == "/health":
                    self._send(200, health(now_fn(), churches, store.load()))
                else:
                    self._send(404, {"error": "not found"})
            except Exception:
                log.exception("request %s failed", self.path)
                self._send(500, {"error": "internal error"})

        def log_message(self, fmt, *args):
            log.info("%s %s", self.address_string(), fmt % args)

    return ThreadingHTTPServer((host, port), Handler)


def start_scheduler(env=os.environ) -> threading.Thread | None:
    """Run job.serve_scheduler in a daemon thread when MSZE_SCHEDULER=1 and a key exists."""
    if env.get("MSZE_SCHEDULER", "0").strip() != "1":
        return None
    key = env.get("OPENROUTER_API_KEY", "").strip()
    if not key:
        log.error("MSZE_SCHEDULER=1 but OPENROUTER_API_KEY is not set; "
                  "serving without the scheduler")
        return None
    from job import make_runner, serve_scheduler  # deferred: the API itself needs no LLM client
    run = make_runner(key)
    t = threading.Thread(target=serve_scheduler, args=(run,), name="scheduler", daemon=True)
    t.start()
    log.info("nightly scheduler started")
    return t


def main() -> None:
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(name)s %(levelname)s %(message)s")
    port = int(os.environ.get("PORT", "8091"))
    data_dir = os.environ.get("DATA_DIR", "/data")
    churches_path = os.environ.get("CHURCHES") or os.path.join(
        os.path.dirname(os.path.abspath(__file__)), "churches.yaml")
    churches = load_churches(churches_path)
    store = Store(os.path.join(data_dir, "cache.json"))
    try:
        start_scheduler()
    except Exception:
        log.exception("scheduler failed to start; serving without it")
    httpd = make_server(port, store, churches)
    log.info("listening on :%d, data in %s", port, data_dir)
    httpd.serve_forever()


if __name__ == "__main__":
    main()
