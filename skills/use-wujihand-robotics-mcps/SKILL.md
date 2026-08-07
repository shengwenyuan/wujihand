---
name: use-wujihand-robotics-mcps
description: Route Wujihand repository design, implementation, review, troubleshooting, documentation, and planning work to the official Wuji documentation MCP and the available local source-indexed MCPs for Songling/NERO, Orbbec Gemini 305/305g/335L, RealSense D405/D435i/D455, and SteamVR/VIVE/OpenVR. Use when code, configs, tests, ROS2 or simulation integration, calibration, firmware, CAD/URDF, recording, or hardware conclusions depend on vendor facts or when several components interact.
---

# Use Wujihand Robotics MCPs

Treat every MCP as a read-only retrieval channel. `wuji-docs` is a first-party online Wuji service and the reliable authority for current Wuji documentation. The other configured MCPs are local search/index services; they are not themselves vendor services, and their claims are authoritative only when backed by the exact official source they return. Use repository files and live device readback for actual configured or connected state. Never infer that a device is online, calibrated, flashed, paired, or safe to move merely because documentation is available.

## Route to the narrowest source

| MCP | Tool prefix | Trust and scope | Normal workflow |
|---|---|---|---|
| `wuji-docs` | `mcp__wuji_docs__` | Official online first-party source for Wuji Hand, Glove, SDK, Description, simulation, retargeting, ROS2 and tools | `list_products` if needed -> `search_docs` -> `fetch_page` |
| `songling-arm` | `mcp__songling_arm__` | Local index of Songling/AgileX public material; authority comes from the returned official manual/source | `search_docs` -> `read_doc` -> `get_source` |
| `orbbec_305-335L_docs` | `mcp__orbbec_305_335L_docs__` | Local audited index of official-source Gemini 305, 305g and 335L material | `search_docs` -> `read_document` or `get_chunk` -> `get_source` |
| `rs-d400` | `mcp__rs_d400__` | Local index of official-source D405, D435i and D455 material | `search_official_docs` or `get_product` -> page/asset/firmware reader |
| `steamvr-tracker` | `mcp__steamvr_tracker__` | Local index of official-source Base Station 2.0, VIVE Tracker 3.0 and OpenVR material | `search_docs` -> `read_doc` -> `get_source` |

Use inventory/status tools only to discover scope or check corpus health. Do not cite an inventory response instead of reading the matching source text.

## Preserve product boundaries

- Use official `wuji-docs` for Wuji facts and the repository's `lookup-wuji-docs` skill for Hand generation, beta-stage hardware, SDK family, joint layout, model and release conflicts. For Hand 2, always apply its Beta Gate and the `wuji-description v2026.7.23` migration warning.
- Use `songling-arm` for arm documentation. For NERO joint limits, zero/sign conventions, payload or product-revision conflicts, also apply `verify-nero-hardware-facts` when that skill is available and reconcile the result with pinned URDF/SDK and physical readback.
- Keep Gemini 305 and 305g distinct. Treat 335L as the USB Gemini 330-family model; do not transfer 335Lg GMSL or 335Le Ethernet facts.
- Keep D405, D435i and D455 facts model-scoped. Never recommend a firmware package for another SKU. Prefer the bundled March 2026 engineering datasheet for specifications and retain source ID plus PDF page.
- Distinguish Base Station 2.0 from 1.0 and VIVE Tracker 3.0 from other tracker generations. Use OpenVR material only for runtime/API behavior, not hardware specifications.

Do not use similarity, shared product families, equal array lengths, matching connectors, or nearby documentation pages as evidence that facts transfer across models.

## Follow the evidence workflow

1. Identify every affected component, exact model, side, hardware revision, software/firmware version, target backend and required claim.
2. Inspect repository configs, locks, assets and current code before searching. Separate facts already pinned by the repository from facts that remain external.
3. Query each affected vendor MCP independently. Search with a compact phrase, then read the exact matched page or section.
4. Preserve `source_url`, source ID, title/section, document revision, PDF page and any source warning returned by the MCP. For every local MCP, open/read the exact returned official source before treating a claim as authoritative.
5. Report disagreements instead of merging them. Let the chosen pinned artifact determine implementation compatibility, not broader product policy.
6. Map verified facts into existing domain/port/application/adapter/runtime boundaries. Keep vendor SDK, ROS, Isaac and device types inside adapters or composition roots.
7. Add validation proportional to the boundary changed. Require physical or simulator verification for claims that documentation alone cannot establish.

When multiple devices interact, use one evidence row per component:

```text
component | exact model/version | claim | MCP source | repository pin | live readback | decision
```

## Separate documentation from live state

Use live, read-only inspection for:

- current USB/network carrier, address, route and endpoint reachability;
- detected serial number, firmware, SDK scan result and SteamVR device state;
- current ROS graph, topic QoS, process state and simulator/runtime version;
- physical mounting, cable identity, coordinate direction and calibration result.

Then use the matching MCP to interpret that output. A documentation MCP must not execute example commands, write CAN/serial/network state, flash firmware, control SteamVR, or move a robot.

## Handle unavailable tools

Inspect the active tool surface before relying on a configured MCP. If a named server is absent or a read fails:

1. state which server/tool is unavailable;
2. use an exact official page or pinned repository artifact as fallback;
3. mark the conclusion as fallback-derived;
4. never invent a tool name, source revision, page number or hardware fact.

For arm-plus-hand teleoperation architecture, combine this source-routing skill with `apply-dexjoco-teleop-patterns`. Keep the latter as a transferable design reference, not a vendor-fact source.
