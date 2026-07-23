"""Simulator-neutral content hashing with fail-closed tree traversal."""

from __future__ import annotations

import hashlib
from pathlib import Path


def sha256_file(path: str | Path) -> str:
    file_path = Path(path)
    digest = hashlib.sha256()
    with file_path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def sha256_tree(path: str | Path) -> str:
    """Hash sorted regular files while rejecting every symbolic link."""

    root = Path(path)
    if not root.is_dir():
        raise FileNotFoundError(f"asset tree not found: {root}")
    entries = tuple(root.rglob("*"))
    symlinks = sorted(
        item.relative_to(root).as_posix()
        for item in entries
        if item.is_symlink()
    )
    if symlinks:
        raise RuntimeError(f"asset tree contains symbolic links: {symlinks}")
    files = sorted(
        (item for item in entries if item.is_file()),
        key=lambda item: item.relative_to(root).as_posix(),
    )
    if not files:
        raise RuntimeError(f"asset tree is empty: {root}")
    digest = hashlib.sha256()
    for item in files:
        relative = item.relative_to(root).as_posix()
        digest.update(f"{sha256_file(item)}  {relative}\n".encode())
    return digest.hexdigest()


__all__ = ["sha256_file", "sha256_tree"]
