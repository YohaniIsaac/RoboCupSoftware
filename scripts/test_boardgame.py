#!/usr/bin/env python3
"""Captura de latencia serial por comando del marcador (T1-T5).

Envía cada TableroCommand N veces (default 20) midiendo el RTT serial
con time.perf_counter() y guarda LOG/test_boardgame_<ts>.json con
latency_per_cmd (mean/std/min/max/samples_ms) y success_count_per_cmd.

Uso:
    python scripts/test_boardgame.py [--port /dev/ttyUSB0] [--n-tests 20]
"""

import sys
import logging
import argparse
import statistics
import time
from pathlib import Path

ROOT_DIR = Path(__file__).parent.parent
sys.path.insert(0, str(ROOT_DIR / "src"))
sys.path.insert(0, str(ROOT_DIR / "scripts"))

from robot_soccer.controllers.rf_transmitter import RFTransmitter, TableroCommand
from metrics.metrics_capture import save_metrics

logging.basicConfig(level=logging.INFO, format='%(message)s')
log = logging.getLogger(__name__)


COMMANDS = [
    ('T1', TableroCommand.TOGGLE_PAUSE),
    ('T2', TableroCommand.GOAL_TEAM1),
    ('T3', TableroCommand.GOAL_TEAM2),
    ('T4', TableroCommand.RESET_GOALS),
    ('T5', TableroCommand.RESET_TIME),
]

LABELS_MEANING = {
    'T1': 'TOGGLE_PAUSE',
    'T2': 'GOAL_TEAM1',
    'T3': 'GOAL_TEAM2',
    'T4': 'RESET_GOALS',
    'T5': 'RESET_TIME',
}


def _stats_ms(samples: list) -> dict:
    """Estadísticas descriptivas en ms (mean, std, min, max, n)."""
    if not samples:
        return {"mean": None, "std": None, "min": None, "max": None, "n": 0}
    return {
        "mean": round(statistics.mean(samples), 3),
        "std": round(statistics.stdev(samples), 3) if len(samples) > 1 else 0.0,
        "min": round(min(samples), 3),
        "max": round(max(samples), 3),
        "n": len(samples),
    }


def _build_summary(args, latency_per_cmd: dict, success_count_per_cmd: dict) -> dict:
    return {
        "n_per_cmd_requested": args.n_tests,
        "interval_s": args.interval,
        "port": args.port,
        "baudrate": args.baudrate,
        "commands": [label for label, _ in COMMANDS],
        "labels_meaning": LABELS_MEANING,
        "latency_per_cmd": {
            label: {
                **_stats_ms(samples),
                "samples_ms": [round(x, 3) for x in samples],
            }
            for label, samples in latency_per_cmd.items()
        },
        "success_count_per_cmd": dict(success_count_per_cmd),
        "success_rate_per_cmd": {
            label: round(success_count_per_cmd[label] / len(latency_per_cmd[label]), 4)
            if latency_per_cmd[label] else 0.0
            for label in latency_per_cmd
        },
        "note": (
            "latency_per_cmd mide el RTT serial entre write() y la respuesta "
            "'OK' del Arduino transmisor (RFTransmitter.send_tablero_command, "
            "timeout serial 1 s). No confirma actualizacion visual del marcador "
            "fisico; esa verificacion se hace en F3.12."
        ),
    }


def _print_summary(latency_per_cmd: dict, success_count_per_cmd: dict, n_iter: int) -> None:
    print("\n" + "=" * 60)
    print(f"MÉTRICAS MARCADOR — N={n_iter} por comando")
    print("=" * 60)
    print(f"{'CMD':<4} {'meaning':<14} {'mean':>8} {'std':>7} {'min':>7} {'max':>7}  success")
    for label, _ in COMMANDS:
        s = _stats_ms(latency_per_cmd[label])
        n_done = len(latency_per_cmd[label])
        ok = success_count_per_cmd[label]
        rate = (ok / n_done * 100) if n_done else 0.0
        if s["n"]:
            print(f"{label:<4} {LABELS_MEANING[label]:<14} "
                  f"{s['mean']:>8.3f} {s['std']:>7.3f} {s['min']:>7.3f} {s['max']:>7.3f}  "
                  f"{ok}/{n_done} ({rate:.1f}%)")
        else:
            print(f"{label:<4} {LABELS_MEANING[label]:<14}  (sin muestras)")
    print("=" * 60)


def main() -> int:
    parser = argparse.ArgumentParser(
        description='Latencia serial de comandos T1-T5 al marcador'
    )
    parser.add_argument('--port', default='/dev/ttyUSB0',
                        help='Puerto serial del Arduino transmisor (default: /dev/ttyUSB0)')
    parser.add_argument('--baudrate', type=int, default=115200,
                        help='Baudrate serial (default: 115200)')
    parser.add_argument('--n-tests', type=int, default=20,
                        help='Repeticiones por comando T1-T5 (default: 20)')
    parser.add_argument('--interval', type=float, default=0.05,
                        help='Pausa entre envíos en segundos (default: 0.05)')

    args = parser.parse_args()

    latency_per_cmd: dict = {label: [] for label, _ in COMMANDS}
    success_count_per_cmd: dict = {label: 0 for label, _ in COMMANDS}

    print("\n🔍 Latencia serial T1-T5 — N={} por comando".format(args.n_tests))
    print(f"Puerto: {args.port}  baudrate: {args.baudrate}")

    tx = RFTransmitter(port=args.port, baudrate=args.baudrate)
    if not tx.connect():
        log.error("No se pudo conectar al transmisor en %s", args.port)
        return 1

    # Drenar el banner de arranque del Arduino antes de medir: al abrir el
    # puerto la placa se reinicia y emite su mensaje de bienvenida; sin vaciarlo
    # los primeros readline() leen esas líneas en lugar de la confirmación 'OK'.
    time.sleep(0.5)
    while tx.serial.in_waiting:
        tx.serial.readline()
    tx.serial.reset_input_buffer()

    try:
        for label, cmd in COMMANDS:
            print(f"\n[{label} {LABELS_MEANING[label]}]")
            for i in range(args.n_tests):
                t0 = time.perf_counter()
                ok = tx.send_tablero_command(cmd)
                dt_ms = (time.perf_counter() - t0) * 1000.0
                latency_per_cmd[label].append(dt_ms)
                if ok:
                    success_count_per_cmd[label] += 1
                print(f"\r  [{i+1}/{args.n_tests}] {dt_ms:7.1f} ms  ok={ok}",
                      end='', flush=True)
                if i < args.n_tests - 1:
                    time.sleep(args.interval)
            print()

        _print_summary(latency_per_cmd, success_count_per_cmd, args.n_tests)
        return 0

    except KeyboardInterrupt:
        log.info("Prueba interrumpida por usuario")
        return 0

    except Exception as e:
        log.error("Error durante la prueba: %s", e)
        return 1

    finally:
        try:
            summary = _build_summary(args, latency_per_cmd, success_count_per_cmd)
            save_metrics("test_boardgame", summary)
        except Exception as e:
            log.warning("No se pudieron guardar metricas boardgame: %s", e)
        tx.disconnect()


if __name__ == '__main__':
    sys.exit(main())
