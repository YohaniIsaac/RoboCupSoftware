#!/usr/bin/env python3
"""[metrics] Lanza test_behavior_1robot.py, guarda el log de la corrida y, al cerrarse,
genera el JSON de episodios de empuje. Un comando por corrida → un .log + un .json.

Es un WRAPPER externo: no toca el script de integración ni el pipeline (src/). Captura el
stdout/stderr que el test ya emite (eco en vivo a tu terminal + copia a LOG/), y al terminar
post-procesa con metrics_push_episodes.parse. Pensado para correr varias veces y luego pasar
las rutas de los JSON.

Uso (los args pasan tal cual al test):
    python scripts/metrics/capture_1robot.py --robot-id 1
    python scripts/metrics/capture_1robot.py --robot-id 1 --serial-port /dev/ttyUSB0

Cierra el test como siempre (ESC / cerrar ventana); el JSON se escribe al salir.
"""
from __future__ import annotations

import os
import subprocess
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from scripts.metrics.metrics_push_episodes import parse          # noqa: E402
from scripts.metrics.metrics_capture import save_metrics, LOG_DIR  # noqa: E402

TEST_SCRIPT = ROOT / "scripts" / "integration" / "test_behavior_1robot.py"


def run_and_capture(cmd: list[str], log_path: Path) -> list[str]:
    """Corre cmd con eco en vivo a stdout y copia a log_path. Devuelve las líneas."""
    lines: list[str] = []
    env = dict(os.environ, PYTHONUNBUFFERED="1")  # evitar block-buffering en el pipe
    with open(log_path, "w", encoding="utf-8") as f:
        proc = subprocess.Popen(
            cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
            text=True, bufsize=1, env=env,
        )
        try:
            assert proc.stdout is not None
            for line in proc.stdout:
                sys.stdout.write(line)
                sys.stdout.flush()
                f.write(line)
                lines.append(line)
        except KeyboardInterrupt:
            proc.terminate()
        proc.wait()
    return lines


def main(argv: list[str]) -> int:
    LOG_DIR.mkdir(exist_ok=True)
    ts = time.strftime("%Y%m%d_%H%M%S")
    log_path = LOG_DIR / f"run_1robot_{ts}.log"

    cmd = [sys.executable, str(TEST_SCRIPT), *argv[1:]]
    print(f"[capture] log -> {log_path}")
    lines = run_and_capture(cmd, log_path)

    data = parse(lines)
    out = save_metrics("push_episodes", data)
    s = data["summary"]
    print(f"\n[capture] episodios de empuje: {data['n_episodes']}  ->  {out}")
    print(f"[capture]   por fin: {s['by_end_type']}")
    print(f"[capture]   avance px: {s['avance_px']}")
    print(f"[capture]   creep cv : {s['creep_pwm_cv']}")
    print(f"[capture] log de la corrida: {log_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))
