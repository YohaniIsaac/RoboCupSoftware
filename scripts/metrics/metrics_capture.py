"""Captura de métricas para scripts de integración. Guarda JSON en LOG/."""
import json
import time
from pathlib import Path

LOG_DIR = Path(__file__).parent.parent.parent / "LOG"


def save_metrics(script_name: str, data: dict) -> Path:
    """Guarda dict de métricas en LOG/<script_name>_<timestamp>.json."""
    LOG_DIR.mkdir(exist_ok=True)
    ts = time.strftime("%Y%m%d_%H%M%S")
    out = LOG_DIR / f"{script_name}_{ts}.json"
    data.update({
        "timestamp_iso": time.strftime("%Y-%m-%dT%H:%M:%S"),
        "script": script_name,
    })
    out.write_text(json.dumps(data, indent=2, ensure_ascii=False), encoding="utf-8")
    return out
