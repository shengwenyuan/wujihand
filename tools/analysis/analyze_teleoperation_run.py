#!/usr/bin/env python3
"""Repository-local launcher for the independent offline analysis package."""

from __future__ import annotations

import sys
from pathlib import Path

PACKAGE_SOURCE = Path(__file__).resolve().parents[2] / "analysis" / "teleoperation_quality" / "src"


def main() -> None:
    """Load the isolated package without adding it to production dependencies."""

    sys.path.insert(0, str(PACKAGE_SOURCE))
    from teleoperation_quality.cli import main as package_main

    package_main()

if __name__ == "__main__":
    main()
