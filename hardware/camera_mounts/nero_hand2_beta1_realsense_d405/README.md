# NERO—Hand 2 Beta1—D405 wrist mount v2

`nero_hand2_beta1_realsense_d405_wrist_mount_v2.scad` keeps the accepted right
mount as canonical geometry. The asset recipe exports a separate left STL with
the `Y -> -Y` reflection baked by OpenSCAD; Isaac must never mirror either STL
with a negative USD scale.

The checked-in `generated/` directory is reproducible with OpenSCAD 2021.01:

```bash
python tools/build_d405_wrist_rig_assets.py --overwrite
```

The generator validates finite bounds, watertightness, winding, welded-vertex
`body_count == 1`, an independent shared-edge face-component count, XZ mirror
symmetry, proper optical rotations, and collision-proxy coverage/gaps. Inputs,
tool hashes and output hashes are frozen in `generation_report.json` and
`third_party/sources.lock.yaml`.

The collision YAML files describe compound child shapes: discrete base/plate
boxes and routed-strut capsules for the mount, plus a D405 housing box. They do
not authorize rigid bodies, mass, inertia, joints or articulation roots.

> SIMULATION ONLY: synthetic 140-degree HFOV; not a physical RealSense D405
> specification or calibration.
