#!/usr/bin/env python3
"""Refresh the Wuji official-document catalog from the public documentation site."""

from __future__ import annotations

import argparse
import concurrent.futures
import json
import sys
from dataclasses import dataclass
from datetime import datetime, timezone
from html.parser import HTMLParser
from pathlib import Path
from typing import Iterable
from urllib.parse import urljoin, urlparse
from urllib.request import Request, urlopen


ORIGIN = "https://docs.wuji.tech"
DOCSETS = (
    ("hardware", "Wuji Hand 2 (Beta 1)", "/docs/zh/wuji-hand/latest/"),
    ("hardware", "Wuji Hand", "/docs/zh/wuji-hand/v1/"),
    ("hardware", "Wuji Glove", "/docs/zh/wuji-glove/latest/"),
    ("software", "Wuji Studio", "/docs/zh/wuji-studio/latest/"),
    ("software", "Wuji SDK", "/docs/zh/wuji-sdk/latest/"),
    ("software", "Wuji Hand ROS2", "/docs/zh/wujihandros2/latest/"),
    ("software", "Wuji Hand HMI", "/docs/zh/wuji-hand-hmi/latest/"),
    ("software", "Wuji Hand Upgrader", "/docs/zh/wuji-hand-upgrader/latest/"),
    ("software", "Wuji Hand SDK (wujihandpy)", "/docs/zh/wujihandpy/latest/"),
    ("algorithm-simulation", "Wuji Description", "/docs/zh/wuji-description/latest/"),
    ("algorithm-simulation", "Wuji Retargeting", "/docs/zh/wuji-retargeting/latest/"),
    ("algorithm-simulation", "MuJoCo Sim", "/docs/zh/mujoco-sim/latest/"),
    ("algorithm-simulation", "Isaac Lab Sim", "/docs/zh/isaaclab-sim/latest/"),
)

EXTERNAL_OFFICIAL = (
    {
        "category": "global",
        "name": "Wuji 文档中心首页",
        "url": "https://docs.wuji.tech/zh/",
        "kind": "official-document-index",
    },
    {
        "category": "global",
        "name": "Wuji 全站发布记录",
        "url": "https://docs.wuji.tech/docs/zh/release-notes/",
        "kind": "official-global-release-feed",
    },
    {
        "category": "algorithm-simulation",
        "name": "Wuji Hand Teleop",
        "url": "https://github.com/wuji-technology/wuji-hand-teleop",
        "kind": "official-github-repository",
    },
)

SINGLE_PAGE_DOCSETS = {
    "/docs/zh/mujoco-sim/latest/",
    "/docs/zh/isaaclab-sim/latest/",
}


def clean_text(parts: Iterable[str]) -> str:
    return " ".join("".join(parts).split())


class SidebarParser(HTMLParser):
    """Extract grouped links from the first documentation sidebar."""

    def __init__(self) -> None:
        super().__init__(convert_charrefs=True)
        self.in_sidebar = False
        self.sidebar_depth = 0
        self.current_group = "metadata"
        self.capture_tag: str | None = None
        self.capture_href: str | None = None
        self.capture_parts: list[str] = []
        self.links: list[dict[str, str]] = []

    def handle_starttag(self, tag: str, attrs: list[tuple[str, str | None]]) -> None:
        attr = dict(attrs)
        if not self.in_sidebar and tag == "aside" and attr.get("id") == "nd-sidebar":
            self.in_sidebar = True
            self.sidebar_depth = 1
            return
        if not self.in_sidebar:
            return
        if tag == "aside":
            self.sidebar_depth += 1
        if tag in {"a", "p"} and self.capture_tag is None:
            self.capture_tag = tag
            self.capture_href = attr.get("href") if tag == "a" else None
            self.capture_parts = []

    def handle_endtag(self, tag: str) -> None:
        if not self.in_sidebar:
            return
        if tag == self.capture_tag:
            label = clean_text(self.capture_parts)
            if tag == "p" and label:
                self.current_group = label
            elif tag == "a" and label and self.capture_href:
                self.links.append(
                    {"group": self.current_group, "title": label, "href": self.capture_href}
                )
            self.capture_tag = None
            self.capture_href = None
            self.capture_parts = []
        if tag == "aside":
            self.sidebar_depth -= 1
            if self.sidebar_depth == 0:
                self.in_sidebar = False

    def handle_data(self, data: str) -> None:
        if self.in_sidebar and self.capture_tag:
            self.capture_parts.append(data)


class LinkParser(HTMLParser):
    """Extract every labeled anchor, including links hidden from a collapsed sidebar."""

    def __init__(self) -> None:
        super().__init__(convert_charrefs=True)
        self.anchor_depth = 0
        self.capture_href: str | None = None
        self.capture_parts: list[str] = []
        self.links: list[dict[str, str]] = []

    def handle_starttag(self, tag: str, attrs: list[tuple[str, str | None]]) -> None:
        if tag != "a":
            return
        if self.anchor_depth == 0:
            self.capture_href = dict(attrs).get("href")
            self.capture_parts = []
        self.anchor_depth += 1

    def handle_endtag(self, tag: str) -> None:
        if tag != "a" or self.anchor_depth == 0:
            return
        self.anchor_depth -= 1
        if self.anchor_depth == 0:
            label = clean_text(self.capture_parts)
            if label and self.capture_href:
                self.links.append(
                    {"group": "guide-linked", "title": label, "href": self.capture_href}
                )
            self.capture_href = None
            self.capture_parts = []

    def handle_data(self, data: str) -> None:
        if self.anchor_depth:
            self.capture_parts.append(data)


class HeadingParser(HTMLParser):
    """Extract h1-h4 headings from the first documentation article."""

    def __init__(self) -> None:
        super().__init__(convert_charrefs=True)
        self.in_article = False
        self.article_depth = 0
        self.capture_tag: str | None = None
        self.capture_id: str | None = None
        self.capture_parts: list[str] = []
        self.headings: list[dict[str, object]] = []

    def handle_starttag(self, tag: str, attrs: list[tuple[str, str | None]]) -> None:
        attr = dict(attrs)
        if not self.in_article and tag == "article":
            self.in_article = True
            self.article_depth = 1
            return
        if not self.in_article:
            return
        if tag == "article":
            self.article_depth += 1
        if tag in {"h1", "h2", "h3", "h4"} and self.capture_tag is None:
            self.capture_tag = tag
            self.capture_id = attr.get("id")
            self.capture_parts = []

    def handle_endtag(self, tag: str) -> None:
        if not self.in_article:
            return
        if tag == self.capture_tag:
            title = clean_text(self.capture_parts)
            if title:
                self.headings.append(
                    {
                        "level": int(tag[1]),
                        "title": title,
                        **({"id": self.capture_id} if self.capture_id else {}),
                    }
                )
            self.capture_tag = None
            self.capture_id = None
            self.capture_parts = []
        if tag == "article":
            self.article_depth -= 1
            if self.article_depth == 0:
                self.in_article = False

    def handle_data(self, data: str) -> None:
        if self.in_article and self.capture_tag:
            self.capture_parts.append(data)


def fetch(url: str, timeout: float) -> str:
    request = Request(
        url,
        headers={
            "User-Agent": "lookup-wuji-docs/1.0 (+documentation catalog refresh)",
            "Accept-Language": "zh-CN,zh;q=0.9,en;q=0.5",
        },
    )
    with urlopen(request, timeout=timeout) as response:
        charset = response.headers.get_content_charset() or "utf-8"
        return response.read().decode(charset, errors="replace")


def canonical_doc_links(html: str, base_path: str) -> list[dict[str, str]]:
    sidebar_parser = SidebarParser()
    sidebar_parser.feed(html)
    link_parser = LinkParser()
    link_parser.feed(html)
    result: list[dict[str, str]] = []
    seen: set[str] = set()
    for item in [*sidebar_parser.links, *link_parser.links]:
        url = urljoin(ORIGIN + base_path, item["href"])
        parsed = urlparse(url)
        if parsed.netloc != urlparse(ORIGIN).netloc or not parsed.path.startswith(base_path):
            continue
        normalized_path = parsed.path if parsed.path.endswith("/") else parsed.path + "/"
        canonical = parsed._replace(path=normalized_path, fragment="", query="").geturl()
        if canonical in seen:
            continue
        seen.add(canonical)
        result.append({"group": item["group"], "title": item["title"], "url": canonical})
    return result


@dataclass(frozen=True)
class PageJob:
    docset_index: int
    page_index: int
    url: str


def parse_headings(html: str) -> list[dict[str, object]]:
    parser = HeadingParser()
    parser.feed(html)
    return parser.headings


def build_catalog(timeout: float, workers: int) -> dict[str, object]:
    docsets: list[dict[str, object]] = []
    failures: list[str] = []

    def fetch_docset(item: tuple[str, str, str]) -> tuple[tuple[str, str, str], str]:
        category, name, path = item
        return item, fetch(ORIGIN + path, timeout)

    index_html: dict[str, str] = {}
    with concurrent.futures.ThreadPoolExecutor(max_workers=workers) as executor:
        futures = {executor.submit(fetch_docset, item): item for item in DOCSETS}
        for future in concurrent.futures.as_completed(futures):
            category, name, path = futures[future]
            try:
                _, html = future.result()
                index_html[path] = html
            except Exception as exc:  # noqa: BLE001 - report every network/parser failure
                failures.append(f"{name} ({ORIGIN + path}): {exc}")

    if failures:
        raise RuntimeError("failed to fetch docset indexes:\n- " + "\n- ".join(sorted(failures)))

    for category, name, path in DOCSETS:
        pages = canonical_doc_links(index_html[path], path)
        if path in SINGLE_PAGE_DOCSETS:
            pages = [{"group": "single-page", "title": name, "url": ORIGIN + path}]
        if not pages:
            failures.append(f"{name} ({ORIGIN + path}): no sidebar pages found")
        docsets.append(
            {
                "category": category,
                "name": name,
                "base_url": ORIGIN + path,
                "pages": pages,
            }
        )

    if failures:
        raise RuntimeError("invalid docset indexes:\n- " + "\n- ".join(sorted(failures)))

    jobs = [
        PageJob(docset_index, page_index, page["url"])
        for docset_index, docset in enumerate(docsets)
        for page_index, page in enumerate(docset["pages"])
    ]

    def fetch_page(job: PageJob) -> tuple[PageJob, list[dict[str, object]]]:
        return job, parse_headings(fetch(job.url, timeout))

    with concurrent.futures.ThreadPoolExecutor(max_workers=workers) as executor:
        futures = {executor.submit(fetch_page, job): job for job in jobs}
        for future in concurrent.futures.as_completed(futures):
            job = futures[future]
            try:
                _, headings = future.result()
                if not headings:
                    raise ValueError("no article headings found")
                docsets[job.docset_index]["pages"][job.page_index]["headings"] = headings
            except Exception as exc:  # noqa: BLE001
                failures.append(f"{job.url}: {exc}")

    if failures:
        raise RuntimeError("failed to fetch complete page outlines:\n- " + "\n- ".join(sorted(failures)))

    return {
        "schema_version": 1,
        "generated_at": datetime.now(timezone.utc).isoformat(timespec="seconds"),
        "source_origin": ORIGIN,
        "language": "zh",
        "docsets": docsets,
        "external_official_sources": list(EXTERNAL_OFFICIAL),
    }


def main() -> int:
    default_output = Path(__file__).resolve().parent.parent / "references" / "official-catalog.json"
    parser = argparse.ArgumentParser(
        description="Rebuild the complete Chinese Wuji documentation page/heading catalog."
    )
    parser.add_argument("--output", type=Path, default=default_output)
    parser.add_argument("--timeout", type=float, default=30.0)
    parser.add_argument("--workers", type=int, default=8)
    args = parser.parse_args()

    try:
        catalog = build_catalog(args.timeout, max(1, args.workers))
    except Exception as exc:  # noqa: BLE001
        print(f"refresh failed; existing catalog was not changed:\n{exc}", file=sys.stderr)
        return 1

    args.output.parent.mkdir(parents=True, exist_ok=True)
    temporary = args.output.with_suffix(args.output.suffix + ".tmp")
    temporary.write_text(
        json.dumps(catalog, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    temporary.replace(args.output)
    page_count = sum(len(item["pages"]) for item in catalog["docsets"])
    print(
        f"wrote {args.output}: {len(catalog['docsets'])} docsets, "
        f"{page_count} pages, generated {catalog['generated_at']}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
