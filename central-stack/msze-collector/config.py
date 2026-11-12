"""Church configuration loader."""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Literal

import yaml

MODES = ("car", "walk")


@dataclass(frozen=True)
class Church:
    id: str
    name: str
    mode: Literal["car", "walk"]
    eta_min: int
    eta_max: int
    eta_confirmed: bool = False
    location: str | None = None  # place within a multi-location page; None = whole page
    schedule_urls: tuple[str, ...] = ()
    ogloszenia_urls: tuple[str, ...] = ()
    schedule_from: str | None = None  # source church id for a filia
    headers: dict[str, str] = field(default_factory=dict)
    bulletin_link_regex: str | None = None

    @property
    def source_id(self) -> str:
        return self.schedule_from or self.id


def _parse(raw: dict) -> Church:
    cid = raw.get("id")
    if not cid:
        raise ValueError("church without id")
    for key in ("name", "mode", "eta_min", "eta_max"):
        if key not in raw:
            raise ValueError(f"{cid}: missing {key}")
    mode = raw["mode"]
    if mode not in MODES:
        raise ValueError(f"{cid}: unknown mode {mode!r}")
    eta_min, eta_max = raw["eta_min"], raw["eta_max"]
    if not (isinstance(eta_min, int) and isinstance(eta_max, int)) or eta_min < 0:
        raise ValueError(f"{cid}: ETA must be non-negative integers")
    if eta_min > eta_max:
        raise ValueError(f"{cid}: eta_min {eta_min} > eta_max {eta_max}")
    schedule_urls = tuple(raw.get("schedule_urls") or ())
    schedule_from = raw.get("schedule_from")
    if not schedule_urls and not schedule_from:
        raise ValueError(f"{cid}: needs schedule_urls or schedule_from")
    return Church(
        id=cid,
        name=raw["name"],
        mode=mode,
        eta_min=eta_min,
        eta_max=eta_max,
        eta_confirmed=bool(raw.get("eta_confirmed", False)),
        location=raw.get("location"),
        schedule_urls=schedule_urls,
        ogloszenia_urls=tuple(raw.get("ogloszenia_urls") or ()),
        schedule_from=schedule_from,
        headers=dict(raw.get("headers") or {}),
        bulletin_link_regex=raw.get("bulletin_link_regex"),
    )


def load_churches(path: str) -> list[Church]:
    with open(path, encoding="utf-8") as f:
        data = yaml.safe_load(f)
    items = (data or {}).get("churches")
    if not items:
        raise ValueError("no churches configured")
    churches = [_parse(r) for r in items]
    ids = [c.id for c in churches]
    if len(set(ids)) != len(ids):
        raise ValueError("duplicate church ids")
    known = set(ids)
    for c in churches:
        if c.schedule_from and c.schedule_from not in known:
            raise ValueError(f"{c.id}: schedule_from {c.schedule_from!r} unknown")
    return churches
