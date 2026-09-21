"""YAML configuration loader for FR3C teleoperation entry points."""

from pathlib import Path

import yaml


def load_config(path: str | None, mode: str) -> dict:
    """Load common settings plus the requested ``single``/``dual`` section."""
    if not path:
        return {}
    config_path = Path(path).expanduser()
    if not config_path.exists():
        raise FileNotFoundError(f"FR3C config not found: {config_path}")
    with config_path.open("r", encoding="utf-8") as handle:
        raw = yaml.safe_load(handle) or {}
    if not isinstance(raw, dict):
        raise ValueError("FR3C YAML root must be a mapping")
    common = raw.get("common", {}) or {}
    section = raw.get(mode, {}) or {}
    if not isinstance(common, dict) or not isinstance(section, dict):
        raise ValueError("FR3C YAML common and mode sections must be mappings")
    merged = dict(common)
    for key, value in section.items():
        if isinstance(value, dict) and isinstance(merged.get(key), dict):
            nested = dict(merged[key])
            nested.update(value)
            merged[key] = nested
        else:
            merged[key] = value
    return merged


def config_value(config: dict, key: str, current):
    """Return a config value, preserving the entry point default if absent."""
    value = config.get(key, current)
    return current if value is None else value
