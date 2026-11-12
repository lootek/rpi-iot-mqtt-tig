from datetime import date

from calendar_pl import day_key, in_winter, polish_holidays, winter_period


def test_holidays_2026():
    h = polish_holidays(2026)
    for d in (date(2026, 11, 1), date(2026, 12, 25), date(2026, 4, 5),
              date(2026, 4, 6), date(2026, 6, 4)):
        assert d in h
    assert date(2026, 5, 24) in h  # Pentecost
    assert date(2026, 11, 2) not in h


def test_day_key_2026_11_01_is_sun_holiday():
    assert day_key(date(2026, 11, 1)) == "sun_holiday"


def test_day_key_2026_11_11_is_wed_independence_day_is_sun_holiday():
    assert date(2026, 11, 11).weekday() == 2
    assert day_key(date(2026, 11, 11)) == "sun_holiday"


def test_day_key_plain_saturday():
    assert day_key(date(2026, 11, 14)) == "sat"


def test_day_key_plain_weekday():
    assert day_key(date(2026, 11, 12)) == "thu"


def test_winter_period_2026():
    assert winter_period(2026) == (date(2026, 10, 25), date(2027, 3, 28))


def test_in_winter_boundaries():
    assert not in_winter(date(2026, 10, 24))
    assert in_winter(date(2026, 10, 25))
    assert in_winter(date(2027, 1, 15))
    assert in_winter(date(2027, 3, 28))
    assert not in_winter(date(2027, 3, 29))
    assert not in_winter(date(2026, 7, 1))
    assert in_winter(date(2026, 2, 1))  # belongs to the 2025 winter
