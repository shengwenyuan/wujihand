# NERO + Hand 2 Beta1 + Gemini 305 wrist-mount inspection

`nero_hand2_beta1_gemini305_wrist_mount_v1.scad` is the editable v1 source.
The Isaac inspection candidate is frozen at:

- right-hand dorsal side `-X`
- external flange plate `Z=-4..0 mm`
- two `13.8 x 5.8 x 1.8 mm` capsule locating keys
- two circular `3.4 mm` M3 flange through-holes
- camera centre offset `70 mm`
- rear-plane height `42 mm`
- camera tilt `16 deg` toward the hand
- camera plate thickness `3.2 mm`
- operator-requested acceptance camera `140 deg` horizontal FOV

The official Gemini 305 RGB specification is `94 deg` horizontal FOV. The
`140 deg` view is therefore an explicitly synthetic Isaac acceptance lens. Its
projection centre is placed `1 mm` ahead of the visual camera housing so that
the housing does not occlude its own synthetic wide-angle image.

The Isaac overlay is visual-only. It does not add rigid bodies, collisions or
joints, and it does not qualify physical fit, screw engagement, cable routing,
payload or robot motion.

## Preserved v0

`nero_hand2_beta1_gemini305_wrist_mount_v0.scad` is an exact copy of the
pre-v1 candidate (SHA-256
`a1db1894e3260a14461056314b8ad7ad65e3e53bafebe7864b26a8ee2ce42ba9`).
It intentionally preserves the old diagonal slots, `52/28/-2 deg` camera
placement and the known `Z=0..4 mm` Hand2 flange overlap. Keep it for design
history and comparison; do not treat it as the current fit candidate.

## Export the mount

Generated meshes remain under ignored `artifacts/`:

```bash
mkdir -p artifacts/derived/camera_mounts/nero_hand2_beta1_gemini305
openscad \
  -D show_reference_preview=false \
  -o artifacts/derived/camera_mounts/nero_hand2_beta1_gemini305/nero_hand2_beta1_gemini305_wrist_mount_v1.stl \
  hardware/camera_mounts/nero_hand2_beta1_gemini305/nero_hand2_beta1_gemini305_wrist_mount_v1.scad
```

OpenSCAD can emit identical facets in a different order, so the runner verifies
an order-independent geometry SHA-256:
`60dba54d756765c7292bc44bc0c2e281cf3375c487e616416a53c0b2ef82f1e8`.
The STL used for the v1 screenshots has file SHA-256
`e6db31e7784fc2bc6760ab96bbc0e9aa2baa02e71d80e34fb579784a51b24da1`.
Use `--no-verify-mount-digest` only while deliberately iterating on the SCAD.

## Open the inspection GUI

From the active Lenovo desktop session:

```bash
cd /home/lenovo/swy/wujihand_mount
DISPLAY=:0 XAUTHORITY=/run/user/1000/gdm/Xauthority \
  /home/lenovo/.venvs/isaacsim-6.0.1/bin/python \
  tools/run_isaac_nero_hand2_gemini305_mount_inspection.py \
  --mount-stl artifacts/derived/camera_mounts/nero_hand2_beta1_gemini305/nero_hand2_beta1_gemini305_wrist_mount_v1.stl \
  --camera-stl analysis/wrist_mount/cad/gemini305-v1.1-scad-local-rear-origin.stl \
  --gui --initial-view assembly
```

The private camera STL is optional. Without `--camera-stl`, the runner uses a
programmatic `42 x 42 x 23 mm` proxy with the `20 mm` rear-hole pitch. The
private STL must be in millimetres, with its rear face at local `Z=0` and the
rear-hole row along local `Y`; it is not redistributed by this repository.

The stage starts paused. Orbit the `Perspective` view to inspect the mount, or
select `ColorOpticalFrame` from the viewport Camera menu to inspect the
synthetic `140 deg` acceptance image.
