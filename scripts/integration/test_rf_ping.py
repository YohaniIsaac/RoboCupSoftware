#!/usr/bin/env python3
"""Mide el RTT del enlace RF con cada robot mediante el comando 'ping'.

El transmisor sondea secuencialmente a cada robot y emite una línea por
respuesta ("Robot N responded!" o "Robot N NO RESPONSE"). El RTT por robot
se mide como el tiempo desde la emisión del 'ping' hasta la llegada de la
línea de respuesta correspondiente.

Se abre el puerto serial directamente (sin la cola de comandos del
RFController) para medir el timing del enlace sin el rate limiter ni el
encolado de comandos de motores.

Uso:
    python scripts/integration/test_rf_ping.py
    python scripts/integration/test_rf_ping.py --serial-port /dev/ttyUSB0 --trials 30

Produce LOG/rf_ping_<timestamp>.json con, por robot (1-4):
    n_trials, n_success, rtt_ms (lista), rtt_mean_ms, rtt_std_ms
"""

import sys
import time
import logging
import argparse
import statistics
from pathlib import Path

ROOT_DIR = Path(__file__).parent.parent.parent
sys.path.insert(0, str(ROOT_DIR / "src"))
sys.path.insert(0, str(ROOT_DIR / "scripts"))

import re
import serial

from metrics.metrics_capture import save_metrics

logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s %(levelname)-8s %(message)s',
    datefmt='%H:%M:%S'
)
log = logging.getLogger(__name__)

BAUD_RATE = 115200
WARMUP = 3            # pings descartados para estabilizar el enlace
INTER_PING_S = 0.4    # pausa entre pings (el sondeo de 4 robots tarda ~300 ms)
NO_DATA_TIMEOUT_S = 0.7   # silencio tras el que se asume fin de respuestas
PING_DEADLINE_S = 6.0     # tiempo máximo absoluto por ping

RE_OK = re.compile(r"Robot\s+(\d+)\s+responded", re.IGNORECASE)
RE_NOK = re.compile(r"Robot\s+(\d+)\s+NO\s+RESPONSE", re.IGNORECASE)


def run_ping_session(port, n_trials):
    """Ejecuta WARMUP + n_trials pings y agrega RTT por robot.

    Returns:
        rtts     : {rid: [ms, ...]} RTT exitosos por robot (1-4).
        failures : {rid: int} intentos sin respuesta por robot.
    """
    rtts = {r: [] for r in range(1, 5)}
    failures = {r: 0 for r in range(1, 5)}

    with serial.Serial(port, BAUD_RATE, timeout=0.1) as ser:
        log.info("Conectado a %s. Esperando boot del Arduino (2 s)...", port)
        time.sleep(2.0)
        ser.reset_input_buffer()

        total = WARMUP + n_trials
        for i in range(total):
            is_warmup = i < WARMUP
            etiqueta = (f"calentamiento {i + 1}/{WARMUP}" if is_warmup
                        else f"ping {i - WARMUP + 1}/{n_trials}")

            t0 = time.perf_counter()
            ser.write(b"ping\n")
            ser.flush()

            last_data = time.perf_counter()
            responded = set()

            while True:
                ahora = time.perf_counter()
                if ahora - t0 > PING_DEADLINE_S or ahora - last_data > NO_DATA_TIMEOUT_S:
                    break

                raw = ser.readline()
                if not raw:
                    continue

                t_line = time.perf_counter()
                last_data = t_line
                line = raw.decode("utf-8", errors="replace").strip()

                m_ok = RE_OK.search(line)
                m_nok = RE_NOK.search(line)
                if m_ok:
                    rid = int(m_ok.group(1))
                    if rid in rtts and rid not in responded and not is_warmup:
                        rtts[rid].append((t_line - t0) * 1000.0)
                        responded.add(rid)
                elif m_nok:
                    rid = int(m_nok.group(1))
                    if rid in failures and rid not in responded and not is_warmup:
                        failures[rid] += 1
                        responded.add(rid)

            estado = ", ".join(
                f"R{r}={'OK' if r in responded else '--'}" for r in range(1, 5)
            )
            log.info("[%s] %s", etiqueta, estado if not is_warmup else "descartado")
            time.sleep(INTER_PING_S)

    return rtts, failures


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--serial-port", default="/dev/ttyUSB0",
                        help="Puerto serial del transmisor (default /dev/ttyUSB0)")
    parser.add_argument("--trials", type=int, default=20,
                        help="Número de pings medidos por robot (default 20)")
    args = parser.parse_args()

    rtts, failures = run_ping_session(args.serial_port, args.trials)

    data = {}
    log.info("=== Resumen RTT ===")
    for rid in range(1, 5):
        muestras = rtts[rid]
        n_suc = len(muestras)
        n_total = n_suc + failures[rid]
        mean = round(statistics.mean(muestras), 2) if muestras else None
        std = round(statistics.stdev(muestras), 2) if len(muestras) > 1 else (0.0 if muestras else None)
        data[f"robot_{rid}"] = {
            "robot_id": rid,
            "n_trials": n_total,
            "n_success": n_suc,
            "rtt_ms": [round(v, 2) for v in muestras],
            "rtt_mean_ms": mean,
            "rtt_std_ms": std,
        }
        mean_str = f"{mean} ± {std} ms" if mean is not None else "sin datos"
        log.info("  Robot %d: %d/%d éxitos — RTT %s", rid, n_suc, n_total, mean_str)

    out = save_metrics("rf_ping", data)
    log.info("Guardado: %s", out)


if __name__ == "__main__":
    main()
