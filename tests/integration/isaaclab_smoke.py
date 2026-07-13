#!/usr/bin/env python3
"""Integration smoke: exercise the pinned Isaac Lab runtime for a finite run."""

from __future__ import annotations

import argparse
import json

from isaaclab.app import AppLauncher


parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--frames", type=int, default=120)
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import isaaclab.sim as sim_utils  # noqa: E402


def _checkpoint(name: str) -> None:
    print(f"ISAACLAB_SMOKE_STAGE={name}", flush=True)


def main() -> None:
    if args_cli.frames < 2:
        raise ValueError("--frames must be at least 2")

    _checkpoint("create_simulation_context")
    sim = sim_utils.SimulationContext(
        sim_utils.SimulationCfg(dt=1.0 / 120.0, device=args_cli.device)
    )
    _checkpoint("spawn_scene")
    sim_utils.GroundPlaneCfg().func("/World/Ground", sim_utils.GroundPlaneCfg())
    light_cfg = sim_utils.DistantLightCfg(intensity=2500.0)
    light_cfg.func("/World/Light", light_cfg, translation=(0.0, 0.0, 3.0))
    cube_cfg = sim_utils.CuboidCfg(
        size=(0.1, 0.1, 0.1),
        rigid_props=sim_utils.RigidBodyPropertiesCfg(),
        mass_props=sim_utils.MassPropertiesCfg(mass=0.1),
        collision_props=sim_utils.CollisionPropertiesCfg(),
    )
    cube_cfg.func("/World/Cube", cube_cfg, translation=(0.0, 0.0, 1.0))

    _checkpoint("reset")
    sim.reset()
    _checkpoint("step")
    cube_prim = sim.stage.GetPrimAtPath("/World/Cube")
    if not cube_prim.IsValid():
        raise RuntimeError("Isaac Lab did not create /World/Cube")
    for frame in range(args_cli.frames):
        if not simulation_app.is_running():
            raise RuntimeError("Isaac Sim stopped before the smoke test completed")
        sim.step(render=False)
        if frame == 0 or frame + 1 == args_cli.frames:
            _checkpoint(f"frame_{frame + 1}")
    result = {
        "status": "passed",
        "device": str(sim.device),
        "frames": args_cli.frames,
        "cube_prim_valid": cube_prim.IsValid(),
        "simulation_app_running": simulation_app.is_running(),
    }
    print("ISAACLAB_SMOKE=" + json.dumps(result, ensure_ascii=False, sort_keys=True), flush=True)


if __name__ == "__main__":
    try:
        main()
    finally:
        # This bounded CI-style probe owns no Replicator writers or external resources.
        # Isaac Sim 5.1 exposes this documented immediate-exit path specifically for
        # avoiding a potentially long full-cleanup phase.
        simulation_app.close(wait_for_replicator=False, skip_cleanup=True)
