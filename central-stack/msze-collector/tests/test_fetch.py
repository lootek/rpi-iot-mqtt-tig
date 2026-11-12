import hashlib
import io
from pathlib import Path

import pytest
import requests
from pypdf import PdfWriter

from config import load_churches
from fetch import Doc, FetchError, fetch_doc, find_bulletin_links, html_to_text

FIX = Path(__file__).parent / "fixtures"
PUBLIC = "93.184.216.34"


def resolves_to(*ips):
    def resolver(host, port, *a, **kw):
        return [(2, 1, 6, "", (ip, port or 0)) for ip in ips]
    return resolver


@pytest.fixture(autouse=True)
def _public_dns(monkeypatch):
    """Existing fetch tests use fake sessions and made-up hosts: resolve them all to a public IP."""
    monkeypatch.setattr("fetch.socket.getaddrinfo", resolves_to(PUBLIC))
CHURCHES = Path(__file__).parent.parent / "churches.yaml"


class FakeResponse:
    def __init__(self, body=b"", status=200, content_type="text/html; charset=utf-8", headers=None):
        self.status_code = status
        self.headers = {"Content-Type": content_type, **(headers or {})}
        self._body = body
        self.chunks_read = 0

    def iter_content(self, chunk_size=1):
        for i in range(0, len(self._body), chunk_size):
            self.chunks_read += 1
            yield self._body[i : i + chunk_size]

    def close(self):
        pass


class FakeSession:
    def __init__(self, response=None, exc=None):
        self.response = response
        self.exc = exc
        self.calls = []

    def get(self, url, headers=None, timeout=None, stream=False, allow_redirects=True):
        self.calls.append({"url": url, "headers": headers, "timeout": timeout})
        self.allow_redirects = allow_redirects
        if self.exc:
            raise self.exc
        if isinstance(self.response, list):  # one response per request, in order
            return self.response.pop(0)
        return self.response


def make_text_pdf(pages: list[str]) -> bytes:
    """Hand-built PDF with a Helvetica text layer, one string per page (ASCII only)."""
    n = len(pages)
    objs: dict[int, bytes] = {1: b"<< /Type /Catalog /Pages 2 0 R >>"}
    kids = " ".join(f"{3 + 2 * i} 0 R" for i in range(n))
    objs[2] = f"<< /Type /Pages /Kids [{kids}] /Count {n} >>".encode()
    font_id = 3 + 2 * n
    for i, text in enumerate(pages):
        page_id, cont_id = 3 + 2 * i, 4 + 2 * i
        objs[page_id] = (
            f"<< /Type /Page /Parent 2 0 R /MediaBox [0 0 612 792] "
            f"/Resources << /Font << /F1 {font_id} 0 R >> >> /Contents {cont_id} 0 R >>"
        ).encode()
        esc = text.replace("\\", "\\\\").replace("(", "\\(").replace(")", "\\)")
        stream = f"BT /F1 12 Tf 50 700 Td ({esc}) Tj ET".encode()
        objs[cont_id] = b"<< /Length %d >>\nstream\n" % len(stream) + stream + b"\nendstream"
    objs[font_id] = b"<< /Type /Font /Subtype /Type1 /BaseFont /Helvetica >>"
    out = bytearray(b"%PDF-1.4\n")
    offsets = {}
    for num in sorted(objs):
        offsets[num] = len(out)
        out += b"%d 0 obj\n" % num + objs[num] + b"\nendobj\n"
    xref = len(out)
    out += b"xref\n0 %d\n0000000000 65535 f \n" % (len(objs) + 1)
    for num in sorted(objs):
        out += b"%010d 00000 n \n" % offsets[num]
    out += b"trailer\n<< /Size %d /Root 1 0 R >>\nstartxref\n%d\n%%%%EOF\n" % (len(objs) + 1, xref)
    return bytes(out)


def make_blank_pdf(n_pages: int) -> bytes:
    w = PdfWriter()
    for _ in range(n_pages):
        w.add_blank_page(width=200, height=200)
    buf = io.BytesIO()
    w.write(buf)
    return buf.getvalue()


LONG = "Ogloszenia parafialne: w przyszla niedziele Msza swieta o godzinie dziewiatej zostaje przesunieta na godzine dziesiata trzydziesci. " * 2


def test_html_to_text_strips_scripts_and_tags():
    html = (
        "<html><head><title>T</title><style>.a{color:red}</style></head><body>"
        "<script>var x = 'Msza 03:00';</script><noscript>nojs</noscript>"
        "<h1>Msze &amp; nabo&#380;e&#324;stwa</h1><p>w niedziele:&nbsp;7:00,   9:00</p>"
        "<ul><li>pon 18:00</li><li>wt 18:00</li></ul></body></html>"
    )
    text = html_to_text(html)
    assert "Msza 03:00" not in text and "color:red" not in text and "nojs" not in text
    assert "<" not in text and "&amp;" not in text
    assert "Msze & nabożeństwa" in text
    assert "w niedziele: 7:00, 9:00" in text
    assert text.splitlines()[-2:] == ["pon 18:00", "wt 18:00"]


def test_find_links_dominikanie_listing():
    html = (FIX / "dominikanie_listing.html").read_text(encoding="utf-8")
    base = "https://krakow.dominikanie.pl/kategoria/ogloszenia/"
    links = find_bulletin_links(html, base)
    assert links == [
        "https://krakow.dominikanie.pl/2026/10/31/ogloszenia-duszpasterskie-uroczystosc-wszystkich-swietych-01-11-2026/",
        "https://krakow.dominikanie.pl/2026/10/24/ogloszenia-duszpasterskie-xxx-niedziela-zwykla-25-10-2026/",
    ]
    # document order, de-duplicated (each post is linked twice), limit honoured
    assert len(find_bulletin_links(html, base, limit=5)) == 3
    assert find_bulletin_links(html, base, limit=1) == links[:1]


def test_find_links_pdf_href():
    html = (
        '<a href="/ogloszenia">Ogłoszenia</a><a href="/photos/Nie samym chlebem 1.pdf">Numer 1</a>'
        '<a href="gazetka.PDF">Numer 2</a><a href="/kontakt">Kontakt</a><a href="#top">Ogłoszenia</a>'
    )
    links = find_bulletin_links(html, "https://example.org/parafia/", regex=r"\.pdf$", limit=5)
    assert links == [
        "https://example.org/photos/Nie samym chlebem 1.pdf",
        "https://example.org/parafia/gazetka.PDF",
    ]
    # default regex finds both the ogłoszenia link and the pdfs, fragment link skipped
    default = find_bulletin_links(html, "https://example.org/parafia/", limit=5)
    assert default[0] == "https://example.org/ogloszenia" and len(default) == 3


def test_find_links_respects_regex_override():
    html = (FIX / "bronowice_home.html").read_text(encoding="utf-8")
    base = "https://franciszkaniebronowice.pl/"
    default = find_bulletin_links(html, base, limit=10)
    assert "https://franciszkaniebronowice.pl/ogloszenia" in default  # menu link eats a slot
    only = find_bulletin_links(html, base, regex=r"/ogloszenia/[^/]+,\d+$", limit=10)
    assert only == [
        "https://franciszkaniebronowice.pl/ogloszenia/uroczystosc-wszystkich-swietych-01112026r,145",
        "https://franciszkaniebronowice.pl/ogloszenia/xxx-niedziela-w-ciagu-roku,144",
        "https://franciszkaniebronowice.pl/ogloszenia/ogloszenia-duszpasterskie-xxix-niedziela-zwykla,143",
    ]
    # the regex shipped in churches.yaml for Bronowice behaves the same on this page
    bron = next(c for c in load_churches(str(CHURCHES)) if c.id == "bronowice")
    assert bron.bulletin_link_regex
    assert find_bulletin_links(html, base, regex=bron.bulletin_link_regex, limit=10) == only
    # regex also matches on anchor text
    assert find_bulletin_links(html, base, regex=r"^XXX Niedziela", limit=10) == only[1:2]


def test_fetch_html_returns_doc_with_hash():
    body = (FIX / "bronowice_home.html").read_bytes()
    sess = FakeSession(FakeResponse(body))
    doc = fetch_doc("https://franciszkaniebronowice.pl/", {}, session=sess)
    assert isinstance(doc, Doc)
    assert doc.kind == "html" and doc.pdf_bytes is None
    assert doc.sha256 == hashlib.sha256(body).hexdigest()
    assert "Msze Św. w niedziele i święta: 7:00, 9:00, 10:30, 12:00, 18:00" in doc.text
    assert "<a" in doc.html  # raw markup kept for listing pages
    assert doc.url == "https://franciszkaniebronowice.pl/"


def test_fetch_sends_configured_headers():
    sess = FakeSession(FakeResponse(b"<p>x</p>"))
    headers = {"User-Agent": "Mozilla/5.0 (X11) Chrome/124", "Accept-Language": "pl-PL"}
    fetch_doc("https://example.org/", headers, session=sess, timeout=7)
    assert sess.calls == [{"url": "https://example.org/", "headers": headers, "timeout": 7}]


def test_fetch_pdf_with_text_layer_returns_text():
    pdf = make_text_pdf([LONG])
    sess = FakeSession(FakeResponse(pdf, content_type="application/pdf"))
    doc = fetch_doc("https://example.org/gazetka", {}, session=sess)
    assert doc.kind == "pdf" and doc.pdf_bytes is None
    assert "Msza swieta o godzinie dziewiatej" in doc.text
    assert doc.sha256 == hashlib.sha256(pdf).hexdigest()
    # detected by .pdf suffix when the server lies about the content type
    sess = FakeSession(FakeResponse(pdf, content_type="application/octet-stream"))
    assert fetch_doc("https://example.org/a.pdf", {}, session=sess).kind == "pdf"


def test_fetch_pdf_image_only_keeps_bytes():
    pdf = make_blank_pdf(1)
    sess = FakeSession(FakeResponse(pdf, content_type="application/pdf"))
    doc = fetch_doc("https://example.org/scan.pdf", {}, session=sess)
    assert doc.kind == "pdf"
    assert doc.pdf_bytes == pdf
    assert len(doc.text or "") < 200
    # a short text layer (< 200 chars) counts as image-only too
    short = make_text_pdf(["Msza o 18:00"])
    doc = fetch_doc("https://example.org/s.pdf", {}, session=FakeSession(FakeResponse(short, content_type="application/pdf")))
    assert doc.pdf_bytes == short


def test_fetch_rejects_oversize_body():
    big = b"x" * 5000
    sess = FakeSession(FakeResponse(big))
    with pytest.raises(FetchError, match="exceeds"):
        fetch_doc("https://example.org/", {}, session=sess, max_bytes=1000)
    # declared Content-Length is rejected before the body is read
    resp = FakeResponse(big, headers={"Content-Length": "5000"})
    with pytest.raises(FetchError, match="Content-Length"):
        fetch_doc("https://example.org/", {}, session=FakeSession(resp), max_bytes=1000)
    assert resp.chunks_read == 0
    # exactly at the cap is fine
    assert fetch_doc("https://example.org/", {}, session=FakeSession(FakeResponse(b"y" * 1000)), max_bytes=1000)


def test_pdf_over_page_cap_uses_first_pages_only():
    pages = [f"PAGEMARK{i} " + LONG for i in range(1, 11)]
    pdf = make_text_pdf(pages)
    sess = FakeSession(FakeResponse(pdf, content_type="application/pdf"))
    doc = fetch_doc("https://example.org/n.pdf", {}, session=sess, max_pages=3)
    assert "PAGEMARK1 " in doc.text and "PAGEMARK3 " in doc.text
    assert "PAGEMARK4 " not in doc.text and "PAGEMARK10" not in doc.text
    default = fetch_doc("https://example.org/n.pdf", {}, session=FakeSession(FakeResponse(pdf, content_type="application/pdf")))
    assert "PAGEMARK8 " in default.text and "PAGEMARK9" not in default.text  # default cap is 8
    # image-only PDF over the cap is truncated before being kept for the model
    from pypdf import PdfReader

    blank = make_blank_pdf(12)
    d = fetch_doc("https://example.org/b.pdf", {}, session=FakeSession(FakeResponse(blank, content_type="application/pdf")), max_pages=8)
    assert len(PdfReader(io.BytesIO(d.pdf_bytes)).pages) == 8


def test_fetch_http_error_raises_fetcherror():
    with pytest.raises(FetchError, match="403"):
        fetch_doc("https://example.org/", {}, session=FakeSession(FakeResponse(b"no", status=403)))
    with pytest.raises(FetchError, match="boom"):
        fetch_doc("https://example.org/", {}, session=FakeSession(exc=requests.ConnectionError("boom")))
    with pytest.raises(FetchError, match="PDF"):
        fetch_doc("https://example.org/x.pdf", {}, session=FakeSession(FakeResponse(b"%PDF-1.4 garbage", content_type="application/pdf")))


# ------------------------------------------------------- SSRF: link discovery


def test_links_to_other_hosts_are_dropped():
    html = (
        '<a href="http://127.0.0.1:8086/ogloszenia">Ogłoszenia</a>'
        '<a href="https://evil.example/ogloszenia.pdf">x</a>'
        '<a href="//192.168.1.1/a.pdf">y</a>'
        '<a href="https://example.org.evil.example/a.pdf">z</a>'
        '<a href="/ogloszenia/1">ok</a>'
    )
    assert find_bulletin_links(html, "https://example.org/", limit=500) == ["https://example.org/ogloszenia/1"]


def test_links_with_non_http_schemes_dropped():
    html = (
        '<a href="ftp://example.org/ogloszenia.pdf">a</a>'
        '<a href="file:///etc/passwd.pdf">b</a>'
        '<a href="data:text/html,ogloszenia">c</a>'
        '<a href="gopher://example.org/ogloszenia">d</a>'
        '<a href="http://example.org/ogloszenia/2">ok</a>'
    )
    assert find_bulletin_links(html, "https://example.org/", limit=500) == ["http://example.org/ogloszenia/2"]


def test_www_variant_same_host_kept():
    html = '<a href="https://www.example.org/ogloszenia/1">a</a><a href="https://EXAMPLE.org./ogloszenia/2">b</a>'
    assert len(find_bulletin_links(html, "https://example.org/", limit=500)) == 2
    html = '<a href="https://example.org/ogloszenia/3">a</a>'
    assert find_bulletin_links(html, "https://www.example.org/x", limit=500) == ["https://example.org/ogloszenia/3"]


# ------------------------------------------------------- SSRF: fetch validation


@pytest.mark.parametrize("ip", ["127.0.0.1", "::1"])
def test_fetch_rejects_loopback(ip):
    sess = FakeSession(FakeResponse(b"secret"))
    with pytest.raises(FetchError):
        fetch_doc("http://example.org/", {}, session=sess, resolver=resolves_to(ip))
    assert sess.calls == []


@pytest.mark.parametrize("ip", ["10.0.0.5", "172.17.0.1", "192.168.1.10"])
def test_fetch_rejects_private_rfc1918(ip):
    sess = FakeSession(FakeResponse(b"secret"))
    with pytest.raises(FetchError):
        fetch_doc("http://example.org/", {}, session=sess, resolver=resolves_to(ip))
    assert sess.calls == []


def test_fetch_rejects_link_local_metadata_ip():
    sess = FakeSession(FakeResponse(b"secret"))
    with pytest.raises(FetchError):
        fetch_doc("http://169.254.169.254/latest/meta-data/", {}, session=sess, resolver=resolves_to("169.254.169.254"))
    assert sess.calls == []


def test_fetch_rejects_host_resolving_to_private_among_public():
    sess = FakeSession(FakeResponse(b"secret"))
    with pytest.raises(FetchError):
        fetch_doc("http://example.org/", {}, session=sess, resolver=resolves_to(PUBLIC, "10.1.2.3"))
    assert sess.calls == []


@pytest.mark.parametrize("url", ["ftp://example.org/a", "file:///etc/passwd", "gopher://example.org/"])
def test_fetch_rejects_non_http_scheme(url):
    sess = FakeSession(FakeResponse(b"x"))
    with pytest.raises(FetchError):
        fetch_doc(url, {}, session=sess, resolver=resolves_to(PUBLIC))
    assert sess.calls == []


def test_fetch_rejects_unresolvable_host():
    def boom(host, port, *a, **kw):
        raise OSError("nxdomain")
    with pytest.raises(FetchError):
        fetch_doc("http://example.org/", {}, session=FakeSession(FakeResponse(b"x")), resolver=boom)


def redirect(location, status=302):
    return FakeResponse(b"", status=status, headers={"Location": location})


def test_redirect_to_private_ip_rejected():
    def resolver(host, port, *a, **kw):
        return resolves_to("192.168.1.1" if host == "www.example.org" else PUBLIC)(host, port)
    sess = FakeSession([redirect("https://www.example.org/x")])
    with pytest.raises(FetchError):
        fetch_doc("https://example.org/", {}, session=sess, resolver=resolver)
    assert [c["url"] for c in sess.calls] == ["https://example.org/"]


def test_redirect_to_other_host_rejected():
    sess = FakeSession([redirect("http://127.0.0.1:8086/")])
    with pytest.raises(FetchError):
        fetch_doc("https://example.org/", {}, session=sess, resolver=resolves_to(PUBLIC))
    sess = FakeSession([redirect("https://evil.example/")])  # public, but a different host
    with pytest.raises(FetchError):
        fetch_doc("https://example.org/", {}, session=sess, resolver=resolves_to(PUBLIC))
    assert len(sess.calls) == 1


def test_redirect_chain_over_five_hops_rejected():
    sess = FakeSession([redirect(f"/r{i}") for i in range(1, 7)] + [FakeResponse(b"<p>end</p>")])
    with pytest.raises(FetchError):
        fetch_doc("https://example.org/", {}, session=sess, resolver=resolves_to(PUBLIC))
    ok = FakeSession([redirect(f"/r{i}") for i in range(1, 6)] + [FakeResponse(b"<p>end</p>")])
    assert fetch_doc("https://example.org/", {}, session=ok, resolver=resolves_to(PUBLIC)).text == "end"


def test_redirect_same_host_followed():
    sess = FakeSession([redirect("/new", 301), redirect("https://www.example.org/final", 307), FakeResponse(b"<p>hi</p>")])
    doc = fetch_doc("https://example.org/old", {"X": "1"}, session=sess, resolver=resolves_to(PUBLIC))
    assert doc.text == "hi"
    assert [c["url"] for c in sess.calls] == [
        "https://example.org/old", "https://example.org/new", "https://www.example.org/final"]
    assert sess.allow_redirects is False
    assert all(c["headers"] == {"X": "1"} for c in sess.calls)
