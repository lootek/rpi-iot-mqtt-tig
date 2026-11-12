"""Nightly extraction job and in-process scheduler.

Per distinct source church (a filia such as Brzoskwinia reads its source's data):
- exceptions every night: the ogłoszenia pages plus the newest bulletin links found on them
- regular schedule on Fridays, with --force-schedule, or when it was never fetched
- the LLM is skipped when the combined content hash equals the stored one
A failing source records `error` in its Part, keeps its previous entries and does not
stop the others. The heartbeat is written at the end of every run.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import logging
import os
import re
import sys
import threading
from datetime import date, datetime, time, timedelta, timezone
from urllib.parse import urlparse
from zoneinfo import ZoneInfo

from config import Church, load_churches
from extract import ExtractionError, OpenRouter, extract_exceptions, extract_regular
from fetch import Doc, FetchError, fetch_doc, find_bulletin_links
from store import Store, merge_exceptions

log = logging.getLogger("msze.job")

WARSAW = ZoneInfo("Europe/Warsaw")
FRIDAY = 4
BULLETINS_PER_LISTING = 2
LISTING_CANDIDATES = 500  # find_bulletin_links limit: see them all, then pick newest
MAX_ERROR_CHARS = 200
WAIT_CHUNK_S = 60

_URL_DATE_RE = re.compile(r"/(\d{4})/(\d{2})/(\d{2})(?:/|$)")


def is_schedule_day(now: datetime) -> bool:
    """Friday in Europe/Warsaw."""
    if now.tzinfo is None:
        now = now.replace(tzinfo=WARSAW)
    return now.astimezone(WARSAW).weekday() == FRIDAY


# ------------------------------------------------------------- link ordering

def _url_date(url: str) -> date | None:
    m = _URL_DATE_RE.search(urlparse(url).path)
    if not m:
        return None
    try:
        return date(int(m[1]), int(m[2]), int(m[3]))
    except ValueError:
        return None


def order_bulletin_links(links: list[str]) -> list[str]:
    """Dated links (YYYY/MM/DD in the path) newest first, then undated in document order."""
    dated = [(u, _url_date(u)) for u in links]
    with_date = sorted((x for x in dated if x[1]), key=lambda x: x[1], reverse=True)
    return [u for u, _ in with_date] + [u for u, d in dated if d is None]


# ------------------------------------------------------------------- helpers

def _err(e: BaseException) -> str:
    if isinstance(e, (FetchError, ExtractionError)):
        msg = " ".join(str(e).split())
    else:  # unknown errors may carry anything; keep only the type
        msg = type(e).__name__
    return msg[:MAX_ERROR_CHARS]


def _combined_hash(docs: list[Doc]) -> str:
    h = hashlib.sha256()
    for d in docs:
        h.update(d.url.encode("utf-8") + b"\0" + d.sha256.encode("utf-8") + b"\n")
    return h.hexdigest()


def _empty_part() -> dict:
    return {"entries": [], "fetched_at": None, "content_hash": None, "error": None}


class _Fetches:
    """Per-source memo so a URL shared by schedule and ogłoszenia is fetched once."""

    def __init__(self, fetcher, headers: dict):
        self._fetcher, self._headers, self._memo = fetcher, headers, {}

    def get(self, url: str) -> Doc:
        if url not in self._memo:
            self._memo[url] = self._fetcher(url, headers=self._headers)
        return self._memo[url]


def _bulletin_docs(src: Church, fx: _Fetches) -> list[Doc]:
    docs: list[Doc] = []
    seen: set[str] = set()

    def add(d: Doc) -> None:
        if d.url not in seen:
            seen.add(d.url)
            docs.append(d)

    for url in src.ogloszenia_urls:
        listing = fx.get(url)  # failure here fails the source
        add(listing)
        if not listing.html:
            continue
        links = find_bulletin_links(listing.html, listing.url, src.bulletin_link_regex,
                                    limit=LISTING_CANDIDATES)
        for link in order_bulletin_links(links)[:BULLETINS_PER_LISTING]:
            try:
                add(fx.get(link))
            except FetchError as e:
                log.warning("%s: bulletin %s skipped: %s", src.id, link, _err(e))
    return docs


def _schedule_docs(src: Church, fx: _Fetches) -> list[Doc]:
    return [fx.get(u) for u in src.schedule_urls]


def _ok(part: dict, entries: list[dict], docs: list[Doc], now: datetime) -> None:
    part.update(entries=entries, fetched_at=now.isoformat(),
                content_hash=_combined_hash(docs), error=None)


# ----------------------------------------------------------------- the job

def run_nightly(now: datetime, store: Store, churches: list[Church], llm,
                fetcher=fetch_doc, force_schedule: bool = False, dry_run: bool = False) -> dict:
    if now.tzinfo is None:
        raise ValueError("now must be timezone-aware")
    today = now.astimezone(WARSAW).date()
    by_id = {c.id: c for c in churches}
    source_ids = list(dict.fromkeys(c.source_id for c in churches))
    cache = store.load()
    sources = cache.setdefault("sources", {})
    summary: dict[str, dict] = {}

    for sid in source_ids:
        src = by_id.get(sid)
        res = summary[sid] = {"regular": "skipped", "exceptions": "skipped"}
        if src is None:
            res["regular"] = res["exceptions"] = "error:source church not configured"
            continue
        entry = sources.setdefault(sid, {})
        reg = entry.setdefault("regular", _empty_part())
        exc = entry.setdefault("exceptions", _empty_part())
        fx = _Fetches(fetcher, src.headers)

        # ---- exceptions: every run
        if src.ogloszenia_urls:
            try:
                docs = _bulletin_docs(src, fx)
                digest = _combined_hash(docs)
                if not dry_run and digest == exc.get("content_hash"):
                    exc.update(entries=merge_exceptions(exc.get("entries") or [], [], today),
                               fetched_at=now.isoformat(), error=None)
                    res["exceptions"] = "skipped"
                else:
                    new = extract_exceptions(llm, src.name, docs, today)
                    merged = merge_exceptions(exc.get("entries") or [], new, today)
                    _ok(exc, merged, docs, now)
                    res["exceptions"] = "ok"
                    if dry_run:
                        res["exception_entries"] = new
            except Exception as e:  # one source must never stop the others
                log.warning("%s: exceptions failed: %s", sid, _err(e))
                exc["error"] = _err(e)
                res["exceptions"] = f"error:{_err(e)}"

        # ---- regular schedule: Fridays, forced, or never fetched
        never = not reg.get("fetched_at") or not reg.get("entries")
        if not (is_schedule_day(now) or force_schedule or never):
            res["regular"] = "not_due"
            continue
        try:
            docs = _schedule_docs(src, fx)
            digest = _combined_hash(docs)
            if not dry_run and digest == reg.get("content_hash") and reg.get("entries"):
                reg.update(fetched_at=now.isoformat(), error=None)
                res["regular"] = "skipped"
            else:
                new = extract_regular(llm, src.name, docs)
                _ok(reg, new, docs, now)
                res["regular"] = "ok"
                if dry_run:
                    res["regular_entries"] = new
        except Exception as e:
            log.warning("%s: regular schedule failed: %s", sid, _err(e))
            reg["error"] = _err(e)
            res["regular"] = f"error:{_err(e)}"

    if not dry_run:
        cache["heartbeat"] = now.isoformat()
        store.save(cache)
    return summary


# --------------------------------------------------------------- scheduler

def next_run_after(now: datetime, hour: int = 3) -> datetime:
    """Next `hour`:00 Europe/Warsaw strictly after `now` (aware datetime, Warsaw tz)."""
    if now.tzinfo is None:
        raise ValueError("now must be timezone-aware")
    local = now.astimezone(WARSAW)
    target = datetime.combine(local.date(), time(hour), tzinfo=WARSAW)
    if target.astimezone(timezone.utc) <= local.astimezone(timezone.utc):
        target = datetime.combine(local.date() + timedelta(days=1), time(hour), tzinfo=WARSAW)
    return target


def serve_scheduler(run, hour: int = 3, stop=None, now_fn=None, chunk: float = WAIT_CHUNK_S) -> None:
    """Call run() every night at `hour`:00 Warsaw until `stop` (a threading.Event) is set.
    Sleeps in chunks of at most `chunk` seconds; run() exceptions are logged, the loop goes on."""
    stop = stop if stop is not None else threading.Event()
    now_fn = now_fn or (lambda: datetime.now(WARSAW))
    while not stop.is_set():
        target = next_run_after(now_fn(), hour)
        log.info("next run at %s", target.isoformat())
        while not stop.is_set():
            remaining = (target.astimezone(timezone.utc) - now_fn().astimezone(timezone.utc)).total_seconds()
            if remaining <= 0:
                break
            stop.wait(min(chunk, remaining))
        if stop.is_set():
            return
        try:
            run()
        except Exception:
            log.exception("nightly run failed")


# --------------------------------------------------------------------- CLI

def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description="msze-collector nightly extraction job")
    ap.add_argument("--once", action="store_true", help="run once and exit (default: nightly scheduler)")
    ap.add_argument("--force-schedule", action="store_true", help="refresh the regular schedule today")
    ap.add_argument("--dry-run", action="store_true", help="print extractions, do not write the cache")
    args = ap.parse_args(argv)

    key = os.environ.get("OPENROUTER_API_KEY", "").strip()
    if not key:
        print("OPENROUTER_API_KEY is not set", file=sys.stderr)
        raise SystemExit(2)
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(name)s %(levelname)s %(message)s")

    data_dir = os.environ.get("DATA_DIR", "/data")
    churches_path = os.environ.get("CHURCHES") or os.path.join(
        os.path.dirname(os.path.abspath(__file__)), "churches.yaml")
    churches = load_churches(churches_path)
    store = Store(os.path.join(data_dir, "cache.json"))
    llm = OpenRouter(key)

    def run(**kw) -> dict:
        return run_nightly(datetime.now(WARSAW), store, churches, llm, fetcher=fetch_doc, **kw)

    if args.once:
        summary = run(force_schedule=args.force_schedule, dry_run=args.dry_run)
        print(json.dumps(summary, ensure_ascii=False, indent=2))
        return 0
    serve_scheduler(lambda: run(force_schedule=args.force_schedule))
    return 0


if __name__ == "__main__":
    sys.exit(main())
