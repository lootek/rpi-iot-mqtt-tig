import os

import pytest

from config import Church, load_churches

REAL = os.path.join(os.path.dirname(__file__), "..", "churches.yaml")


def _write(tmp_path, body):
    p = tmp_path / "c.yaml"
    p.write_text(body)
    return str(p)


def _church_yaml(**over):
    base = {
        "id": "x",
        "name": "X",
        "mode": "car",
        "eta_min": 5,
        "eta_max": 10,
        "schedule_urls": ["https://example.org/"],
    }
    base.update(over)
    lines = ["churches:", "  - " + "\n    ".join(f"{k}: {v!r}" for k, v in base.items())]
    return "\n".join(lines) + "\n"


def test_load_real_config_has_seven_churches():
    ids = [c.id for c in load_churches(REAL)]
    assert ids == ["morawica", "brzoskwinia", "dominikanie", "olszanica", "mydlniki", "karmel", "bronowice"]


def test_brzoskwinia_is_walk_filia_of_morawica():
    c = {c.id: c for c in load_churches(REAL)}["brzoskwinia"]
    assert isinstance(c, Church)
    assert c.mode == "walk"
    assert c.schedule_from == "morawica"
    assert c.location == "Brzoskwinia"
    assert c.source_id == "morawica"


def test_dominikanie_has_chrome_user_agent():
    c = {c.id: c for c in load_churches(REAL)}["dominikanie"]
    assert "Chrome/" in c.headers["User-Agent"]


def test_eta_min_not_above_eta_max_rejected(tmp_path):
    with pytest.raises(ValueError):
        load_churches(_write(tmp_path, _church_yaml(eta_min=20, eta_max=10)))


def test_unknown_mode_rejected(tmp_path):
    with pytest.raises(ValueError):
        load_churches(_write(tmp_path, _church_yaml(mode="bike")))
