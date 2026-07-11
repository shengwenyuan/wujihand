---
name: lookup-wuji-docs
description: Locate, verify, and explain authoritative Wuji Technology documentation with product-generation and version awareness. Use for Chinese or English questions about 舞肌灵巧手、仿真、遥操作、重定向, Wuji Hand 2 Beta 1, first-generation Wuji Hand, Wuji Glove, wuji-sdk, wujihandpy, Studio, ROS2, HMI, Upgrader, URDF/MJCF/USD models, MuJoCo, Isaac Lab, retargeting, teleop, calibration, tactile/EMF/IMU data, APIs, compatibility, troubleshooting, or when designing or modifying this repository's software, simulation, data-recording, testing, and adapter architecture from Wuji docs.
---

# Lookup Wuji Docs

Use the local catalog to locate sources quickly, then verify unstable facts against the live official page or the intended official repository tag before analyzing or designing.

## Follow the lookup workflow

### 1. Fix the scope before searching

Identify these dimensions from the request or surrounding project:

```text
product: Hand 2 Beta 1 / Hand v1 / Wuji Glove / unknown
side: left / right / both / irrelevant
task: hardware / SDK / ROS2 / model / simulation / retargeting / teleop / troubleshooting
time: current behavior / a named version / historical reproduction
target: visualization / physical simulation / real hardware
```

Do not ask the user if repository imports, URLs, config names, device transport, or existing files reveal the answer. If the product generation remains unknown and changes the result, present separate branches rather than silently choosing one.

Normalize common legacy names:

```text
Qt HMI / WujiHand Qt HMI -> Wuji Hand HMI
OTA HMI / WujiHand OTA   -> Wuji Hand Upgrader
Hand latest              -> inspect URL; /wuji-hand/latest is currently Hand 2
Wuji Hand SDK            -> disambiguate wuji-sdk from wujihandpy
```

### 2. Search the local catalog

Run from this skill directory:

```bash
python scripts/search_catalog.py "user topic" --limit 10
python scripts/search_catalog.py "teleop simulation Hand 2" --category algorithm-simulation
python scripts/search_catalog.py "joint command API" --category software --json
```

Use results to select exact pages and heading anchors. Treat `references/official-catalog.json` as a machine index; do not load its full contents into context unless debugging the catalog.

If a term has no match, search the official docs site and official `wuji-technology` GitHub organization directly. Search engines may retain removed routes; confirm that a result remains in the current site navigation or target release.

### 3. Read only the relevant routing references

- Read [references/navigation.md](references/navigation.md) for the complete three-branch documentation tree and task-to-entry mapping.
- Read [references/hardware-versions.md](references/hardware-versions.md) for Hand 2 versus Hand v1 versus Glove, hardware constraints, data shapes, and generation-specific cross-links.
- Read [references/software.md](references/software.md) for SDK/API selection, Studio, ROS2, HMI, Upgrader, version transitions, and known documentation conflicts.
- Read [references/simulation-teleop.md](references/simulation-teleop.md) for model assets, MuJoCo, Isaac Lab, Retargeting, input sources, real-device paths, and the ROS2 Teleop stack.
- Read [references/project-architecture.md](references/project-architecture.md) before placing or changing code, configs, datasets, tests, external assets, or formal project documentation in this repository.

For simulation or teleop questions, read both `hardware-versions.md` and `simulation-teleop.md`. Also read `software.md` when the target is real hardware, ROS2, or an SDK integration.

### 4. Verify live sources at the right granularity

Open the exact official page returned by the catalog. For claims that can change, also open its product release notes and the [global release feed](https://docs.wuji.tech/docs/zh/release-notes/) when multiple components interact. Use official GitHub only under `github.com/wuji-technology` and inspect the intended tag/commit when commands or code behavior matter.

Match source type to claim:

| Claim | Required evidence |
|---|---|
| Product limits, power, safety | Product page + user notice/usage constraints + product release notes |
| Python/C API | Current SDK docs + product-specific reference + SDK release notes |
| CLI flags or config paths | README/docs and actual script/config at the intended repository tag |
| Model path, joint/link names | `wuji-description` target tag and model file; do not infer from a different generation |
| Compatibility | Both components' release notes plus installation/requirements at target tags |
| Historical reproduction | Fixed tag/commit, dependency lock, firmware and submodule commit; avoid rolling `latest` |

Treat `latest/` as a moving pointer. Record the page URL and lookup date. Prefer a fixed tag or commit for reproducible designs.

### 5. Preserve official disagreements

Do not merge contradictory official sources into a synthetic answer. Report:

```text
source A says ... (page/tag/date)
source B says ... (page/tag/date)
the implementation decision therefore depends on ...
```

Use the target tag's code to settle what that exact artifact accepts, but do not claim that code resolves the documentation policy or support status.

Known conflict classes to check explicitly:

- Hand 2's product-specific `sdk-reference` page may still show pre-v2026.7.1 `control_mode`, singular `joint_state`, `joint_command_publisher`, and three-array send signatures while current release notes require resource subscribe/publish APIs.
- Hand 2 ROS2 support claims versus the current `wujihandros2` guide, dependencies, examples, and tag.
- Wuji SDK integrated Retargeting versus the independent `wuji-retargeting` repository's maintenance state and broader simulation/config tools.
- Documentation-center input lists versus newer repository README/CHANGELOG/CLI flags.
- Current 24×31 Glove tactile data versus historical 24×32 recordings.
- Current common-SDK `TactileGloveFrame` documentation versus the first-generation Hand accessory's legacy 24×32/768 contract.

### 6. Answer with traceable scope

Lead with the result. Include, as applicable:

1. Selected product generation and software/API family.
2. Direct official page links, near the facts they support.
3. Version or lookup date for rolling documentation.
4. Exact units, shapes, joint naming/order, transport, and control mode.
5. Constraints, unresolved conflicts, and beta/legacy status.
6. Only then provide analysis, architecture, code, or a design recommendation.

Never present a local-reference summary as a citation. Cite the official page or repository that the reference routes to.

## Apply the key routing rules

### Distinguish hardware generations

```text
Hand 2 Beta 1:
  /docs/zh/wuji-hand/latest/
  Ethernet, MIT/resources, wuji-sdk, Studio, hand2_beta models

Hand v1:
  /docs/zh/wuji-hand/v1/
  USB, position control, wuji-sdk.WujiHand or wujihandpy, ROS2/HMI/Upgrader

Current Wuji Hand ROS2 docs:
  /docs/zh/wujihandros2/latest/
  treat old /wuji-hand/v1/ros2-user-guide/... routes as historical mirrors, not canonical current pages

Wuji Glove:
  /docs/zh/wuji-glove/latest/
  tactile + EMF + IMU + 21-DoF human-hand products
```

Do not confuse the independent Wuji Glove with the first-generation Hand's older tactile accessory.

### Distinguish software APIs

Do not exchange these contracts:

```text
wuji-sdk Hand 2: joint_states(), publish(), JointCommand payloads, Ethernet resources
wuji-sdk Hand v1: joint_state(), publisher(), position payloads, realtime controller
wujihandpy: legacy first-generation Hand/Finger/Joint USB API
```

Check whether Hand 2 code crosses the 2026.7.1 unified-resource breaking change. Match firmware and SDK versions.

### Distinguish simulation and teleop paths

```text
Wuji Description      -> authoritative model assets
MuJoCo/Isaac Lab Sim  -> minimal model/trajectory smoke demos
Wuji Retargeting      -> open simulation, inputs, configs, tuning, sim/real examples
wuji-sdk Retargeting  -> current SDK-integrated 21x3 -> 20-joint session and examples
Wuji Hand Teleop      -> first-generation ROS2 whole-stack real teleoperation
RViz                  -> visualization, not physical simulation
```

For Hand 2 real teleop, prefer the current Wuji SDK/Retargeting Ethernet path unless a newer official ROS2 release explicitly verifies Hand 2. Do not route Hand 2 through a first-generation USB stack by analogy.

For Hand 2 + Vision Pro simulation, verify that the selected AVP config actually points to `hand2_beta` URDF/MJCF and Hand 2 link mappings. A default AVP config may still target the first-generation model.

## Guard analysis and implementation

Before proposing code or architecture, verify:

```text
21 human landmarks vs 20 robot joints
meters vs centimeters vs radians
left/right frame and handedness
API-specific joint names/order/nid mapping
URDF vs ROS URDF vs MJCF vs USD
USB vs Ethernet
position control vs MIT control
command effort vs measured current vs torque/contact force
firmware/SDK/model/submodule compatibility
enable, limit, stop, disconnect, and fault behavior
```

Do not treat Hand 2 motor current as contact force. Do not assume a Hand 2 soft-body model exists. Do not call an Isaac Lab loading demo a complete reinforcement-learning task unless task and training code are present.

## Map verified facts into this repository

Keep the first vertical slice explicit:

```text
MediaPipe -> canonical 21×3 observation -> retargeting -> 20-joint intent
          -> supervision -> Isaac execution -> multi-stream trajectory dataset
```

Preserve these boundaries when adding code:

```text
domain/       canonical types, units, frames, joint layouts, invariants
ports/        external-independent input, retarget, execution, state, recorder protocols
application/  calibration, retargeting, supervision, recording, teleop orchestration
adapters/     MediaPipe, Glove, external Retargeting, Isaac, MuJoCo, Wuji SDK, ROS2, PI, storage
runtime/      config loading, CLI, dependency composition
```

Do not let MediaPipe, Isaac, ROS2, or Wuji SDK objects cross an adapter boundary. Record raw observation, joint intent, safety decision, sent command, and backend feedback as separate signals. Store official source/version/commit provenance with models, derived configs, algorithms, and datasets.

Treat future Glove, exoskeleton, MuJoCo, ROS2, and PI folders as reserved extension points until code, contract tests, and component docs exist. Keep local dynamic work in ignored `plans/`; put stable behavior and real code entry points in versioned `docs/`.

For the complete placement and validation rules, read `references/project-architecture.md`; for the formal project baseline, consult `docs/000-project-charter-and-architecture.md` from the repository root.

## Refresh the catalog when needed

Refresh when the catalog is old relative to the question, a documented route returns 404, a top navigation entry changes, or the user asks for the latest complete structure:

```bash
python scripts/refresh_catalog.py
```

The script reads only official public pages and atomically replaces `references/official-catalog.json` only after all document sets and page outlines parse successfully. If refresh fails, keep the old catalog, state its `generated_at`, and browse the target pages manually.
