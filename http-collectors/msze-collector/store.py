"""JSON cache store: atomic writes, exception merge."""
from __future__ import annotations

import json
import logging
import os
import tempfile
from datetime import date

log = logging.getLogger("msze.store")


def _empty() -> dict:
    return {"sources": {}, "heartbeat": None}


class Store:
    def __init__(self, path: str):
        self.path = path

    def load(self) -> dict:
        try:
            with open(self.path, encoding="utf-8") as f:
                data = json.load(f)
        except FileNotFoundError:
            return _empty()
        except (OSError, ValueError) as err:
            log.warning("cache %s unreadable (%s); starting empty", self.path, err)
            return _empty()
        if not isinstance(data, dict):
            log.warning("cache %s has unexpected shape; starting empty", self.path)
            return _empty()
        data.setdefault("sources", {})
        data.setdefault("heartbeat", None)
        return data

    def save(self, cache: dict) -> None:
        directory = os.path.dirname(os.path.abspath(self.path))
        os.makedirs(directory, exist_ok=True)
        fd, tmp = tempfile.mkstemp(dir=directory, prefix=".cache-", suffix=".tmp")
        try:
            with os.fdopen(fd, "w", encoding="utf-8") as f:
                json.dump(cache, f, ensure_ascii=False, indent=2)
                f.flush()
                os.fsync(f.fileno())
            os.replace(tmp, self.path)
        except BaseException:
            try:
                os.unlink(tmp)
            except OSError:
                pass
            raise


def _key(x: dict) -> tuple:
    return (x.get("date"), x.get("time"), x.get("action"), x.get("location"))


def merge_exceptions(old: list[dict], new: list[dict], today: date) -> list[dict]:
    """Union keyed by (date, time, action, location); new wins; past dates dropped."""
    iso = today.isoformat()
    merged: dict[tuple, dict] = {}
    for x in list(old) + list(new):
        if (x.get("date") or "") < iso:
            continue
        merged[_key(x)] = x
    return sorted(merged.values(), key=lambda x: (x.get("date") or "", x.get("time") or ""))
