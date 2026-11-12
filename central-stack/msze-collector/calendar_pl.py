"""Polish public holidays, day type and winter period."""
from datetime import date, timedelta

_DAYS = ("mon", "tue", "wed", "thu", "fri", "sat")


def _easter(year: int) -> date:
    """Easter Sunday (anonymous Gregorian algorithm)."""
    a = year % 19
    b, c = divmod(year, 100)
    d, e = divmod(b, 4)
    f = (b + 8) // 25
    g = (b - f + 1) // 3
    h = (19 * a + b - d - g + 15) % 30
    i, k = divmod(c, 4)
    l = (32 + 2 * e + 2 * i - h - k) % 7
    m = (a + 11 * h + 22 * l) // 451
    month, day = divmod(h + l - 7 * m + 114, 31)
    return date(year, month, day + 1)


def polish_holidays(year: int) -> set[date]:
    easter = _easter(year)
    fixed = [(1, 1), (1, 6), (5, 1), (5, 3), (8, 15), (11, 1), (11, 11),
             (12, 25), (12, 26)]
    days = {date(year, m, d) for m, d in fixed}
    days.add(easter)
    days.add(easter + timedelta(days=1))    # Easter Monday
    days.add(easter + timedelta(days=49))   # Pentecost
    days.add(easter + timedelta(days=60))   # Corpus Christi
    return days


def day_key(d: date) -> str:
    if d.weekday() == 6 or d in polish_holidays(d.year):
        return "sun_holiday"
    return _DAYS[d.weekday()]


def _last_sunday(year: int, month: int) -> date:
    nxt = date(year + (month == 12), month % 12 + 1, 1)
    last = nxt - timedelta(days=1)
    return last - timedelta(days=(last.weekday() + 1) % 7)


def winter_period(year: int) -> tuple[date, date]:
    return _last_sunday(year, 10), _last_sunday(year + 1, 3)


def in_winter(d: date) -> bool:
    for y in (d.year - 1, d.year):
        start, end = winter_period(y)
        if start <= d <= end:
            return True
    return False
