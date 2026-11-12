"""HTTP fetching, HTML to text, bulletin link discovery, PDF handling."""
from __future__ import annotations

import hashlib
import io
import re
from dataclasses import dataclass
from html.parser import HTMLParser
from typing import Literal
from urllib.parse import urldefrag, urljoin, urlparse

import requests
from pypdf import PdfReader, PdfWriter

DEFAULT_BULLETIN_REGEX = r"ogłoszen|ogloszen|\.pdf$"
TEXT_LAYER_MIN_CHARS = 200
_CHUNK = 64 * 1024


class FetchError(Exception):
    """Download or parse failure; the caller records it and keeps old data."""


@dataclass(frozen=True)
class Doc:
    url: str
    kind: Literal["html", "pdf"]
    text: str | None  # html text, or pdf text-layer text
    pdf_bytes: bytes | None  # set when kind == "pdf" and the text layer is < 200 chars (image-only)
    sha256: str  # hash of the raw body
    html: str | None = None  # raw markup of an html page, for find_bulletin_links on listings


# ---------------------------------------------------------------- HTML -> text

_SKIP_TAGS = {"script", "style", "noscript", "template", "head", "svg"}
_BLOCK_TAGS = {
    "p", "div", "br", "li", "ul", "ol", "tr", "table", "section", "article",
    "header", "footer", "nav", "h1", "h2", "h3", "h4", "h5", "h6", "blockquote",
}


class _TextExtractor(HTMLParser):
    def __init__(self) -> None:
        super().__init__(convert_charrefs=True)
        self.parts: list[str] = []
        self._skip = 0

    def handle_starttag(self, tag, attrs):
        if tag in _SKIP_TAGS:
            self._skip += 1
        elif tag in _BLOCK_TAGS:
            self.parts.append("\n")

    def handle_startendtag(self, tag, attrs):
        if tag in _BLOCK_TAGS:
            self.parts.append("\n")

    def handle_endtag(self, tag):
        if tag in _SKIP_TAGS:
            self._skip = max(0, self._skip - 1)
        elif tag in _BLOCK_TAGS:
            self.parts.append("\n")

    def handle_data(self, data):
        if not self._skip:
            self.parts.append(data)


def html_to_text(html: str) -> str:
    """Visible text of an HTML page: no scripts/styles/tags, whitespace collapsed,
    block elements separated by newlines."""
    p = _TextExtractor()
    p.feed(html)
    p.close()
    text = "".join(p.parts).replace("\xa0", " ")
    lines = (re.sub(r"[ \t\r\f\v]+", " ", ln).strip() for ln in text.split("\n"))
    return "\n".join(ln for ln in lines if ln)


# ------------------------------------------------------------ link discovery


class _AnchorCollector(HTMLParser):
    def __init__(self) -> None:
        super().__init__(convert_charrefs=True)
        self.anchors: list[tuple[str, str]] = []  # (href, text)
        self._href: str | None = None
        self._text: list[str] = []

    def handle_starttag(self, tag, attrs):
        if tag == "a":
            self._flush()
            self._href = dict(attrs).get("href") or ""
            self._text = []

    def handle_data(self, data):
        if self._href is not None:
            self._text.append(data)

    def handle_endtag(self, tag):
        if tag == "a":
            self._flush()

    def _flush(self):
        if self._href is not None:
            self.anchors.append((self._href.strip(), " ".join("".join(self._text).split())))
        self._href = None
        self._text = []


def find_bulletin_links(listing_html: str, base_url: str, regex: str | None = None, limit: int = 2) -> list[str]:
    """Absolute URLs of anchors on a listing page whose href or text matches `regex`
    (case-insensitive; default: ogłoszenia or a .pdf href).

    Returned in DOCUMENT order, de-duplicated, at most `limit`; the page itself
    (base_url) and fragment/javascript/mailto links are skipped. Whether the live
    listing is ordered newest-first or oldest-first is the caller's business: this
    function does not sort, so a caller that needs "latest" must pick accordingly.
    """
    pat = re.compile(regex or DEFAULT_BULLETIN_REGEX, re.IGNORECASE)
    parser = _AnchorCollector()
    parser.feed(listing_html)
    parser.close()
    parser._flush()
    self_url = urldefrag(base_url)[0]
    out: list[str] = []
    for href, text in parser.anchors:
        if not href or href.startswith(("#", "javascript:", "mailto:", "tel:")):
            continue
        if not (pat.search(href) or pat.search(text)):
            continue
        absolute = urldefrag(urljoin(base_url, href))[0]
        if absolute == self_url or absolute in out:
            continue
        out.append(absolute)
        if len(out) >= limit:
            break
    return out


# ---------------------------------------------------------------------- fetch


def _charset(content_type: str, head: bytes) -> str:
    m = re.search(r"charset=([\w.-]+)", content_type, re.I) or re.search(
        rb"<meta[^>]+charset=[\"']?([\w.-]+)", head[:4096], re.I
    )
    if not m:
        return "utf-8"
    val = m.group(1)
    return val.decode("ascii", "ignore") if isinstance(val, bytes) else val


def _is_pdf(url: str, content_type: str, body: bytes) -> bool:
    if "pdf" in content_type.lower():
        return True
    if urlparse(url).path.lower().endswith(".pdf"):
        return True
    return body[:5] == b"%PDF-"


def _read_body(resp, max_bytes: int, url: str) -> bytes:
    declared = resp.headers.get("Content-Length")
    if declared and declared.isdigit() and int(declared) > max_bytes:
        raise FetchError(f"{url}: Content-Length {declared} exceeds {max_bytes} bytes")
    buf = bytearray()
    for chunk in resp.iter_content(chunk_size=_CHUNK):
        buf.extend(chunk)
        if len(buf) > max_bytes:
            raise FetchError(f"{url}: body exceeds {max_bytes} bytes")
    return bytes(buf)


def _pdf_doc(url: str, body: bytes, sha: str, max_pages: int) -> Doc:
    try:
        reader = PdfReader(io.BytesIO(body))
        total = len(reader.pages)
        pages = reader.pages[:max_pages]
        text = "\n".join((pg.extract_text() or "") for pg in pages).strip()
        if len(text) >= TEXT_LAYER_MIN_CHARS:
            return Doc(url, "pdf", text, None, sha)
        pdf_bytes = body
        if total > max_pages:  # keep the model upload bounded as well
            w = PdfWriter()
            for pg in pages:
                w.add_page(pg)
            out = io.BytesIO()
            w.write(out)
            pdf_bytes = out.getvalue()
    except FetchError:
        raise
    except Exception as e:  # pypdf raises many error types on broken files
        raise FetchError(f"{url}: unreadable PDF: {e}") from e
    return Doc(url, "pdf", text, pdf_bytes, sha)


def fetch_doc(url: str, headers: dict | None = None, session=None, timeout: int = 30,
              max_bytes: int = 5_000_000, max_pages: int = 8) -> Doc:
    """GET `url` and return an HTML or PDF Doc. Raises FetchError on network/HTTP
    errors, oversize bodies and unreadable PDFs. Only the first `max_pages` pages of
    a PDF are read (and, for image-only PDFs, kept in `pdf_bytes`)."""
    session = session or requests.Session()
    try:
        resp = session.get(url, headers=headers or {}, timeout=timeout, stream=True)
    except requests.RequestException as e:
        raise FetchError(f"{url}: {e}") from e
    try:
        if resp.status_code >= 400:
            raise FetchError(f"{url}: HTTP {resp.status_code}")
        try:
            body = _read_body(resp, max_bytes, url)
        except requests.RequestException as e:
            raise FetchError(f"{url}: {e}") from e
        content_type = resp.headers.get("Content-Type", "") or ""
    finally:
        close = getattr(resp, "close", None)
        if close:
            close()
    sha = hashlib.sha256(body).hexdigest()
    if _is_pdf(url, content_type, body):
        return _pdf_doc(url, body, sha, max_pages)
    html = body.decode(_charset(content_type, body), errors="replace")
    return Doc(url, "html", html_to_text(html), None, sha, html=html)
