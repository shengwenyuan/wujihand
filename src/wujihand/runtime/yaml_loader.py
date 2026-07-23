"""Fail-closed YAML loading shared by runtime configuration boundaries."""

from __future__ import annotations

from typing import cast

import yaml


class StrictSafeLoader(yaml.SafeLoader):
    """SafeLoader variant that rejects duplicate mapping keys at every depth."""

    def construct_mapping(
        self,
        node: yaml.nodes.MappingNode,
        deep: bool = False,
    ) -> dict[object, object]:
        self.flatten_mapping(node)
        mapping: dict[object, object] = {}
        for key_node, value_node in node.value:
            key = self.construct_object(key_node, deep=deep)
            try:
                duplicate = key in mapping
            except TypeError as exc:
                raise ValueError("YAML mapping keys must be hashable") from exc
            if duplicate:
                raise ValueError(f"duplicate YAML mapping key: {key!r}")
            mapping[key] = self.construct_object(value_node, deep=deep)
        return mapping


def load_yaml_strict(text: str) -> object:
    """Load safe YAML while preserving duplicate-key failures as ValueError."""

    return cast(object, yaml.load(text, Loader=StrictSafeLoader))


__all__ = ["StrictSafeLoader", "load_yaml_strict"]
