#!/usr/bin/env python3
"""Script para probar conexiones RF con robots.

Este script envía un comando 'ping' al transmisor RF y muestra
qué robots y tablero están respondiendo.

Uso:
    python scripts/test_rf_ping.py [--port /dev/ttyUSB0]
"""

import sys
import logging
import argparse
import time
from pathlib import Path

# Agregar src y scripts al path
ROOT_DIR = Path(__file__).parent.parent
sys.path.insert(0, str(ROOT_DIR / "src"))
sys.path.insert(0, str(ROOT_DIR / "scripts"))

import statistics

from robot_soccer.communication.rf_controller import RFController
from metrics.metrics_capture import save_metrics

# Configurar logging
logging.basicConfig(
    level=logging.INFO,
    format='%(message)s'
)
log = logging.getLogger(__name__)

# Silenciar logs de submódulos
logging.getLogger('robot_soccer.communication.serial_manager').setLevel(logging.WARNING)


def print_connection_status(connections):
    """Imprime el estado de las conexiones de forma clara.

    Args:
        connections: Diccionario con estado de conexiones
    """
    print("\n" + "=" * 50)
    print("ESTADO DE CONEXIONES RF")
    print("=" * 50)

    # Tablero
    tablero_status = "✅ CONECTADO" if connections['tablero'] else "❌ SIN RESPUESTA"
    print(f"📡 Tablero:  {tablero_status}")

    print("\n🤖 Robots:")
    for i in range(1, 5):
        robot_key = f'robot_{i}'
        robot_status = "✅ CONECTADO" if connections[robot_key] else "❌ SIN RESPUESTA"
        print(f"   Robot {i}: {robot_status}")

    # Resumen
    connected_robots = sum(1 for i in range(1, 5) if connections[f'robot_{i}'])
    print("\n" + "-" * 50)
    print(f"📊 Resumen: {connected_robots}/4 robots conectados")
    print("=" * 50 + "\n")


def _rtt_stats(samples: list) -> dict:
    """Estadísticas descriptivas de RTT en ms (mean, std, min, max, n)."""
    if not samples:
        return {"mean": None, "std": None, "min": None, "max": None, "n": 0}
    return {
        "mean": round(statistics.mean(samples), 3),
        "std": round(statistics.stdev(samples), 3) if len(samples) > 1 else 0.0,
        "min": round(min(samples), 3),
        "max": round(max(samples), 3),
        "n": len(samples),
    }


def _print_metrics_summary(samples: list, success_counts: dict, n_iter: int) -> None:
    """Imprime resumen RTT agregado + success_rate por dispositivo."""
    s = _rtt_stats(samples)
    n_done = len(samples)
    print("\n" + "=" * 50)
    print(f"MÉTRICAS RF — {n_done}/{n_iter} pings completados")
    print("=" * 50)
    if s["n"]:
        print(f"RTT agregado (ms): mean={s['mean']}  std={s['std']}  "
              f"min={s['min']}  max={s['max']}")
    print("Success rate por dispositivo:")
    for key, count in success_counts.items():
        rate = count / n_done if n_done else 0.0
        print(f"  {key:10s}: {count}/{n_done}  ({rate * 100:.1f}%)")
    print("=" * 50)


def main():
    """Función principal."""
    parser = argparse.ArgumentParser(
        description='Prueba de conexiones RF con robots'
    )
    parser.add_argument(
        '--port',
        type=str,
        default='/dev/ttyUSB0',
        help='Puerto serial del transmisor RF (default: /dev/ttyUSB0)'
    )
    parser.add_argument(
        '--repeat',
        type=int,
        default=1,
        help='Número de veces que repetir el ping (default: 1, modo legacy)'
    )
    parser.add_argument(
        '--interval',
        type=float,
        default=None,
        help='Intervalo entre pings (default: 2.0 s legacy, 0.1 s en modo métricas)'
    )
    parser.add_argument(
        '--n-pings',
        type=int,
        nargs='?',
        const=100,
        default=None,
        help='Modo métricas: N pings con timing RTT y guardado JSON. '
             'Sin valor usa 100. Si se omite el flag, comportamiento legacy con --repeat.'
    )

    args = parser.parse_args()

    metrics_mode = args.n_pings is not None
    if metrics_mode:
        n_iter = args.n_pings
        interval = args.interval if args.interval is not None else 0.1
    else:
        n_iter = args.repeat
        interval = args.interval if args.interval is not None else 2.0

    rtt_ms_samples: list = []
    success_counts: dict = {
        'tablero': 0,
        'robot_1': 0,
        'robot_2': 0,
        'robot_3': 0,
        'robot_4': 0,
    }

    print("\n🔍 Iniciando prueba de conexiones RF...")
    print(f"Puerto serial: {args.port}")

    # Crear controlador RF
    rf_controller = RFController(port=args.port, enable_calibration=False)

    # Inicializar comunicación
    if not rf_controller.initialize():
        log.error("❌ Error: No se pudo inicializar la comunicación RF")
        log.error(f"   Verifica que el transmisor esté conectado en {args.port}")
        return 1

    log.info("✅ Comunicación RF inicializada correctamente\n")

    try:
        # Ejecutar ping(s)
        for i in range(n_iter):
            if not metrics_mode and n_iter > 1:
                print(f"\n{'=' * 50}")
                print(f"PING #{i+1} de {n_iter}")
                print(f"{'=' * 50}")

            # Enviar ping y obtener respuestas (cronometrado)
            t0 = time.perf_counter()
            connections = rf_controller.test_connections()
            rtt_ms = (time.perf_counter() - t0) * 1000.0

            if metrics_mode:
                rtt_ms_samples.append(rtt_ms)
                for key in success_counts:
                    if connections.get(key):
                        success_counts[key] += 1
                print(f"\r[{i+1}/{n_iter}] RTT={rtt_ms:7.1f} ms",
                      end='', flush=True)
            else:
                print_connection_status(connections)

            # Esperar antes del siguiente ping (si hay más)
            if i < n_iter - 1:
                time.sleep(interval)

        if metrics_mode:
            print()  # newline tras la barra de progreso
            _print_metrics_summary(rtt_ms_samples, success_counts, n_iter)

        return 0

    except KeyboardInterrupt:
        log.info("\n⏹️  Prueba interrumpida por usuario")
        return 0

    except Exception as e:
        log.error(f"\n❌ Error durante la prueba: {e}")
        return 1

    finally:
        if metrics_mode:
            try:
                summary = {
                    "n_pings_requested": n_iter,
                    "n_pings_completed": len(rtt_ms_samples),
                    "interval_s": interval,
                    "port": args.port,
                    "rtt_aggregate_ms": _rtt_stats(rtt_ms_samples),
                    "success_count": dict(success_counts),
                    "success_rate": {
                        k: round(v / len(rtt_ms_samples), 4) if rtt_ms_samples else 0.0
                        for k, v in success_counts.items()
                    },
                    "note": (
                        "rtt_aggregate_ms mide el RTT de un ping atomico que cubre "
                        "tablero + 4 robots en una sola transaccion "
                        "(RFController.test_connections, timeout 5 s). Per-robot RTT "
                        "requiere instrumentar src/ o firmware; fuera de scope de F1.6."
                    ),
                }
                save_metrics("test_rf_ping", summary)
            except Exception as e:
                log.warning("No se pudieron guardar metricas RF: %s", e)
        rf_controller.shutdown()
        log.info("🔌 Conexión RF cerrada")


if __name__ == '__main__':
    sys.exit(main())
