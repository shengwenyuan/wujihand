from __future__ import annotations

import ast
from pathlib import Path


ROOT = Path(__file__).parents[2]
SRC = ROOT / "src/wujihand"


def _imports(path: Path) -> set[str]:
    tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
    package = path.parent.relative_to(SRC.parent).parts
    modules: set[str] = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            modules.update(alias.name for alias in node.names)
        elif isinstance(node, ast.ImportFrom):
            modules.add(_resolve_import_from(package, node))
    return modules


def _resolve_import_from(
    package: tuple[str, ...], node: ast.ImportFrom
) -> str:
    if node.level == 0:
        return node.module or ""
    keep = len(package) - node.level + 1
    if keep < 0:
        return node.module or ""
    prefix = package[:keep]
    suffix = () if node.module is None else tuple(node.module.split("."))
    return ".".join((*prefix, *suffix))


def test_relative_imports_are_normalized_to_absolute_packages() -> None:
    node = ast.parse("from ..runtime import SessionResolver").body[0]
    assert isinstance(node, ast.ImportFrom)
    assert (
        _resolve_import_from(("wujihand", "adapters"), node)
        == "wujihand.runtime"
    )


def test_specs_do_not_depend_on_runtime_adapters_or_external_sdks() -> None:
    forbidden_prefixes = (
        "wujihand.runtime",
        "wujihand.adapters",
        "mujoco",
        "isaacsim",
        "omni",
        "openvr",
        "pxr",
        "rclpy",
        "wuji_sdk",
    )

    violations = {
        path.relative_to(ROOT).as_posix(): sorted(
            module
            for module in _imports(path)
            if module.startswith(forbidden_prefixes)
        )
        for path in (SRC / "specs").rglob("*.py")
    }

    assert not {path: imports for path, imports in violations.items() if imports}


def test_adapters_do_not_import_runtime() -> None:
    violations = {
        path.relative_to(ROOT).as_posix(): sorted(
            module
            for module in _imports(path)
            if module.startswith("wujihand.runtime")
        )
        for path in (SRC / "adapters").rglob("*.py")
    }

    assert not {path: imports for path, imports in violations.items() if imports}


def test_domain_ports_and_application_preserve_inward_dependency_direction() -> None:
    forbidden_by_layer = {
        "domain": (
            "wujihand.specs",
            "wujihand.ports",
            "wujihand.application",
            "wujihand.adapters",
            "wujihand.runtime",
        ),
        "ports": (
            "wujihand.specs",
            "wujihand.application",
            "wujihand.adapters",
            "wujihand.runtime",
        ),
        "application": (
            "wujihand.specs",
            "wujihand.adapters",
            "wujihand.runtime",
        ),
    }
    external_sdks = (
        "isaacsim",
        "mujoco",
        "omni",
        "openvr",
        "pxr",
        "rclpy",
        "wuji_sdk",
    )

    violations: dict[str, list[str]] = {}
    for layer, forbidden_internal in forbidden_by_layer.items():
        forbidden = (*forbidden_internal, *external_sdks)
        for path in (SRC / layer).rglob("*.py"):
            imports = sorted(
                module
                for module in _imports(path)
                if module.startswith(forbidden)
            )
            if imports:
                violations[path.relative_to(ROOT).as_posix()] = imports

    assert not violations


def test_compat_contracts_are_isolated_from_runtime_adapters_and_external_sdks() -> None:
    forbidden_prefixes = (
        "wujihand.runtime",
        "wujihand.adapters",
        "mujoco",
        "isaacsim",
        "omni",
        "openvr",
        "pxr",
        "rclpy",
    )
    violations = {
        path.relative_to(ROOT).as_posix(): sorted(
            module
            for module in _imports(path)
            if module.startswith(forbidden_prefixes)
        )
        for path in (SRC / "compat").rglob("*.py")
    }

    assert not {path: imports for path, imports in violations.items() if imports}
