# WujiHand mini dataset exporter

This isolated Python 3.12 environment converts only already accepted, immutable q54
episodes into LeRobot v3. Raw ROS 2 MCAP, q21, Tracker, privileged simulation truth and
lossless RGB inputs remain outside the policy artifact.

The LeRobot source is pinned to tag `v0.6.1`, commit
`7e241bd630a3719a56157a497ce5d08f244784f1`. The exporter refuses to overwrite an
existing revision, performs `finalize()`, reopens the local dataset, decodes every row and
writes checksums plus q54/source-map sidecars before atomic publication.

Create the environment with the checked-in lock once available on the deployment host:

```bash
uv sync --project analysis/mini_dataset_export --frozen
```

Export only after every selected episode is registered as `accepted`, has a passing release
artifact, and has a checksum-closed bundle. A normal immutable local revision is created with:

```bash
analysis/mini_dataset_export/.venv/bin/wujihand-export-mini-dataset \
  --project-root "$PWD" \
  --collection-root artifacts/datasets/isaac_nero_hand2_triview_q54_mini_dataset_v1/collection \
  --collection-id isaac_nero_hand2_triview_q54_mini_dataset_v1 \
  --destination artifacts/datasets/isaac_nero_hand2_triview_q54_mini_dataset_v1/revisions/<REVISION_ID> \
  --repo-id local/wujihand-mini-sim \
  --revision-id <REVISION_ID>
```

The destination must remain inside the project root and must not already exist. The exporter
validates accepted registry records, release and bundle digests, q54/camera provenance, then
finalizes, reopens and decodes the dataset before atomically publishing it. It also records the
revision in the collection export registry. A rejected or later-restored source episode marks a
dependent revision stale; revisions are never updated in place.

The exporter is offline-only. It does not import ROS, OpenVR, glove SDKs or Isaac control
entrypoints.
