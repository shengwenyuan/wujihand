#!/usr/bin/env python3
"""Search the local Wuji documentation catalog without loading it into context."""

from __future__ import annotations

import argparse
import json
import re
from pathlib import Path
from typing import Any
from urllib.parse import quote


ALIASES = (
    {"teleop", "teleoperation", "遥操作", "远程操作", "遥控"},
    {"retarget", "retargeting", "重定向", "动作映射", "姿态映射"},
    {"simulation", "sim", "仿真", "模拟"},
    {"mujoco", "mjcf"},
    {"isaac", "isaaclab", "isaac lab", "isaac sim", "usd", "强化学习", "rl"},
    {"model", "description", "模型", "urdf", "mjcf", "usd", "stl", "step", "cad"},
    {"sdk", "api", "接口", "开发", "python", "c++", "wujihandpy"},
    {"ros", "ros2", "rviz", "topic", "service", "launch"},
    {"glove", "手套", "emf", "imu", "tactile", "触觉", "手部追踪"},
    {"hand2", "hand 2", "二代", "beta 1", "beta1", "beta 2", "beta2", "wuji hand 2"},
    {"hand1", "hand v1", "一代", "旧版", "旧代", "wujihandpy"},
    {"install", "installation", "安装", "环境", "依赖"},
    {"connect", "connection", "连接", "发现", "scan"},
    {"ethernet", "network", "网络", "网口", "ip"},
    {"usb", "serial", "串口", "直连"},
    {"control", "command", "控制", "指令", "mit", "关节"},
    {"record", "recording", "录制", "mcap", "回放", "playback"},
    {"calibration", "calibrate", "标定", "校准", "零位"},
    {"release", "changelog", "version", "版本", "发布记录", "更新"},
    {"v2026.7.23", "2026.7.23", "2026.07.23", "2026-07-23", "model revision", "模型版本"},
    {"troubleshoot", "troubleshooting", "debug", "故障", "排查", "错误", "异常"},
)

STOP_TERMS = {"wuji", "hand", "1", "2", "v1", "latest"}


def known_warning(docset: str, page_url: str) -> str | None:
    if page_url == "https://docs.wuji.tech/docs/zh/wuji-hand/latest/sdk-reference/":
        return (
            "Hand 2 is beta hardware. This page showed pre-v2026.7.1 signatures at the "
            "catalog snapshot; compare product/SDK release notes, hardware stage, firmware, "
            "and the target wuji-sdk tag."
        )
    if docset == "Wuji Hand 2 (Beta 1)":
        return (
            "The official manual is for Beta 1 samples and does not represent final product "
            "behavior. Check product release notes for Beta 2-only hardware and firmware."
        )
    if docset == "Wuji Description":
        return (
            "v2026.7.23 replaced the earlier Hand 2 model revision. Do not mix hand2_beta "
            "with hand2/hand2_beta1 names, roots, mappings, collision rules, or datasets."
        )
    if docset == "Wuji Hand ROS2":
        return (
            "The current guide mainly validates first-generation USB hardware; verify Hand 2 "
            "Ethernet support against release notes, dependencies, examples, and the target tag."
        )
    if docset == "Wuji Retargeting":
        return (
            "The global v2026.7.1 release feed says the standalone repository paused updates "
            "after Retargeting moved into wuji-sdk; verify the intended tag and maintenance state."
        )
    return None


def normalize(text: str) -> str:
    return re.sub(r"[\s_\-/()（）]+", " ", text.casefold()).strip()


def alias_in_query(alias: str, query: str) -> bool:
    """Use word boundaries for ASCII aliases so `imu` does not match `simulation`."""
    if any(ord(character) > 127 for character in alias):
        return alias in query
    return re.search(rf"(?<![a-z0-9]){re.escape(alias)}(?![a-z0-9])", query) is not None


def expanded_terms(query: str) -> list[str]:
    normalized_query = normalize(query)
    raw_terms = {
        normalized_query,
        *(part for part in normalized_query.split() if part not in STOP_TERMS),
    }
    terms = set(raw_terms)
    for group in ALIASES:
        normalized_group = {normalize(item) for item in group}
        if any(item and alias_in_query(item, normalized_query) for item in normalized_group):
            terms.update(normalized_group)
    return sorted((term for term in terms if term), key=lambda item: (-len(item), item))


def heading_url(page_url: str, heading: dict[str, Any]) -> str:
    identifier = heading.get("id")
    return f"{page_url}#{quote(identifier, safe='-._~')}" if identifier else page_url


def search(catalog: dict[str, Any], query: str, category: str | None) -> list[dict[str, Any]]:
    query_norm = normalize(query)
    terms = expanded_terms(query)
    results: list[dict[str, Any]] = []
    asks_hand2 = any(
        alias_in_query(normalize(item), query_norm)
        for item in (
            "hand 2",
            "hand2",
            "wuji hand 2",
            "二代",
            "beta 1",
            "beta1",
            "beta 2",
            "beta2",
        )
    )
    asks_hand1 = any(
        alias_in_query(normalize(item), query_norm)
        for item in ("hand 1", "hand1", "hand v1", "一代", "wujihandpy")
    )
    asks_teleop = any(
        alias_in_query(normalize(item), query_norm)
        for item in ("teleop", "teleoperation", "遥操作", "远程操作", "遥控")
    )
    asks_simulation = any(
        alias_in_query(normalize(item), query_norm)
        for item in ("sim", "simulation", "仿真", "模拟")
    )
    asks_release = any(
        alias_in_query(normalize(item), query_norm)
        for item in ("release", "changelog", "version", "版本", "发布记录", "更新")
    )
    asks_hand2_model_revision = any(
        item in query_norm
        for item in (
            "v2026.7.23",
            "2026.7.23",
            "2026.07.23",
            "2026-07-23",
            "model revision",
            "模型版本",
        )
    )
    asks_mediapipe = alias_in_query("mediapipe", query_norm)
    asks_isaac = any(
        alias_in_query(normalize(item), query_norm)
        for item in ("isaac", "isaaclab", "isaac lab", "isaac sim")
    )
    mentions_specific_product = any(
        alias_in_query(normalize(item), query_norm)
        for item in (
            "hand",
            "hand 2",
            "glove",
            "sdk",
            "studio",
            "ros2",
            "hmi",
            "upgrader",
            "description",
            "retargeting",
            "mujoco",
            "isaac",
            "灵巧手",
            "手套",
        )
    )
    requested_legacy_docsets: set[str] = set()
    if alias_in_query("ros2", query_norm) or alias_in_query("ros", query_norm):
        requested_legacy_docsets.add("Wuji Hand ROS2")
    if alias_in_query("hmi", query_norm):
        requested_legacy_docsets.add("Wuji Hand HMI")
    if alias_in_query("upgrader", query_norm) or alias_in_query("ota hmi", query_norm):
        requested_legacy_docsets.add("Wuji Hand Upgrader")
    if alias_in_query("wujihandpy", query_norm):
        requested_legacy_docsets.add("Wuji Hand SDK (wujihandpy)")
    hand2_docset = "Wuji Hand 2 (Beta 1)"
    hand1_docsets = {
        "Wuji Hand",
        "Wuji Hand ROS2",
        "Wuji Hand HMI",
        "Wuji Hand Upgrader",
        "Wuji Hand SDK (wujihandpy)",
    }

    for docset in catalog["docsets"]:
        if category and docset["category"] != category:
            continue
        docset_norm = normalize(docset["name"])
        for page in docset["pages"]:
            title_norm = normalize(page["title"])
            group_norm = normalize(page["group"])
            heading_rows = [
                (heading, normalize(str(heading["title"]))) for heading in page.get("headings", [])
            ]
            page_text = " ".join([title_norm, *(row[1] for row in heading_rows)])
            score = 0
            matched_headings: list[dict[str, str]] = []
            if query_norm == title_norm:
                score += 40
            elif query_norm and query_norm in title_norm:
                score += 20
            if query_norm and query_norm in docset_norm:
                score += 12
            elif docset_norm and docset_norm in query_norm:
                score += 16
            for term in terms:
                if term in title_norm:
                    score += 8
                if term in docset_norm:
                    score += 4
                if term in group_norm:
                    score += 2
                for heading, heading_norm in heading_rows:
                    if term in heading_norm:
                        score += 3
                        item = {
                            "title": str(heading["title"]),
                            "url": heading_url(page["url"], heading),
                        }
                        if item not in matched_headings:
                            matched_headings.append(item)
            if asks_hand2:
                if docset["name"] == hand2_docset:
                    score += 10
                elif docset["name"] in requested_legacy_docsets:
                    score += 30
                elif docset["name"] in hand1_docsets:
                    score -= 35
            if asks_hand1:
                if docset["name"] in hand1_docsets:
                    score += 25
                elif docset["name"] == hand2_docset:
                    score -= 25
            if asks_teleop and asks_simulation:
                has_teleop = any(
                    normalize(item) in page_text
                    for item in ("teleop", "teleoperation", "遥操作", "远程操作", "遥控")
                )
                has_simulation = any(
                    normalize(item) in page_text for item in ("sim", "simulation", "仿真", "模拟")
                )
                if has_teleop and has_simulation:
                    score += 25
            if asks_mediapipe:
                if docset["name"] == "Wuji SDK" and "/retargeting/" in page["url"]:
                    score += 30
                elif docset["name"] == "Wuji Retargeting" and any(
                    route in page["url"] for route in ("/quick-start/", "/api/", "/installation/")
                ):
                    score += 20
            if asks_isaac:
                if docset["name"] == "Isaac Lab Sim":
                    score += 15
                elif docset["name"] == "Wuji Description" and "/integration/" in page["url"]:
                    score += 15
            if asks_hand2_model_revision:
                if docset["name"] == "Wuji Description":
                    score += 100 if "/release-notes/" in page["url"] else 25
                elif docset["name"] == hand2_docset and "/release-notes/" in page["url"]:
                    score += 60
            if score > 0:
                result = {
                    "score": score,
                    "category": docset["category"],
                    "docset": docset["name"],
                    "group": page["group"],
                    "page": page["title"],
                    "url": page["url"],
                    "matched_headings": matched_headings[:8],
                }
                warning = known_warning(docset["name"], page["url"])
                if warning:
                    result["warning"] = warning
                results.append(result)

    for source in catalog.get("external_official_sources", []):
        if category and source["category"] != category:
            continue
        name_norm = normalize(source["name"])
        score = sum(10 for term in terms if term in name_norm)
        if query_norm and query_norm in name_norm:
            score += 20
        if source["category"] == "global" and (
            "全站" in query_norm or alias_in_query("global", query_norm)
        ):
            score += 120
        if source["name"] == "Wuji 全站发布记录" and asks_release and not mentions_specific_product:
            score += 100
        if score:
            results.append(
                {
                    "score": score,
                    "category": source["category"],
                    "docset": source["name"],
                    "group": source["kind"],
                    "page": source["name"],
                    "url": source["url"],
                    "matched_headings": [],
                }
            )

    return sorted(results, key=lambda item: (-item["score"], item["docset"], item["page"]))


def main() -> int:
    default_catalog = (
        Path(__file__).resolve().parent.parent / "references" / "official-catalog.json"
    )
    parser = argparse.ArgumentParser(description="Search the cached Wuji official-doc catalog.")
    parser.add_argument("query", help="Chinese or English topic, API, tool, or workflow")
    parser.add_argument("--catalog", type=Path, default=default_catalog)
    parser.add_argument(
        "--category", choices=("global", "hardware", "software", "algorithm-simulation")
    )
    parser.add_argument("--limit", type=int, default=10)
    parser.add_argument("--json", action="store_true", help="emit machine-readable JSON")
    args = parser.parse_args()

    catalog = json.loads(args.catalog.read_text(encoding="utf-8"))
    results = search(catalog, args.query, args.category)[: max(1, args.limit)]
    if args.json:
        print(json.dumps(results, ensure_ascii=False, indent=2))
        return 0

    print(f"catalog generated_at: {catalog['generated_at']}")
    if not results:
        print("no local catalog match; browse the official site search and release notes")
        return 1
    for index, item in enumerate(results, start=1):
        print(
            f"{index}. [{item['category']}] {item['docset']} > "
            f"{item['group']} > {item['page']} (score={item['score']})"
        )
        print(f"   {item['url']}")
        if item.get("warning"):
            print(f"   WARNING: {item['warning']}")
        for heading in item["matched_headings"][:4]:
            print(f"   - {heading['title']}: {heading['url']}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
