from __future__ import annotations

import ast
from pathlib import Path


ROOT = Path(__file__).parents[2]
RUNNER = ROOT / "tools/run_isaac_nero_hand2_dual_twin.py"


def _runner_tree() -> ast.Module:
    return ast.parse(
        RUNNER.read_text(encoding="utf-8"),
        filename=str(RUNNER),
    )


def _function(tree: ast.Module, name: str) -> ast.FunctionDef:
    matches = [
        node
        for node in tree.body
        if isinstance(node, ast.FunctionDef) and node.name == name
    ]
    assert len(matches) == 1
    return matches[0]


def test_tracker_runner_never_blocks_gui_on_stdin() -> None:
    tree = _runner_tree()

    input_calls = [
        node
        for node in ast.walk(tree)
        if isinstance(node, ast.Call)
        and isinstance(node.func, ast.Name)
        and node.func.id == "input"
    ]

    assert input_calls == []


def test_tracker_gui_routes_to_persistent_interactive_loop() -> None:
    tree = _runner_tree()
    live = _function(tree, "_run_tracker_live")
    interactive = _function(tree, "_run_tracker_interactive")

    interactive_calls = [
        node
        for node in ast.walk(live)
        if isinstance(node, ast.Call)
        and isinstance(node.func, ast.Name)
        and node.func.id == "_run_tracker_interactive"
    ]
    running_loops = [
        node
        for node in ast.walk(interactive)
        if isinstance(node, ast.While)
        and isinstance(node.test, ast.Call)
        and isinstance(node.test.func, ast.Attribute)
        and isinstance(node.test.func.value, ast.Name)
        and node.test.func.value.id == "simulation_app"
        and node.test.func.attr == "is_running"
    ]

    assert len(interactive_calls) == 1
    assert len(running_loops) == 1
