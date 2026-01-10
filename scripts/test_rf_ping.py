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

# Agregar src al path
ROOT_DIR = Path(__file__).parent.parent
sys.path.insert(0, str(ROOT_DIR / "src"))

from robot_soccer.communication.rf_controller import RFController

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
        help='Número de veces que repetir el ping (default: 1)'
    )
    parser.add_argument(
        '--interval',
        type=float,
        default=2.0,
        help='Intervalo en segundos entre pings repetidos (default: 2.0)'
    )

    args = parser.parse_args()

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
        for i in range(args.repeat):
            if args.repeat > 1:
                print(f"\n{'=' * 50}")
                print(f"PING #{i+1} de {args.repeat}")
                print(f"{'=' * 50}")

            # Enviar ping y obtener respuestas
            connections = rf_controller.test_connections()

            # Mostrar resultados
            print_connection_status(connections)

            # Esperar antes del siguiente ping (si hay más)
            if i < args.repeat - 1:
                time.sleep(args.interval)

        return 0

    except KeyboardInterrupt:
        log.info("\n⏹️  Prueba interrumpida por usuario")
        return 0

    except Exception as e:
        log.error(f"\n❌ Error durante la prueba: {e}")
        return 1

    finally:
        rf_controller.shutdown()
        log.info("🔌 Conexión RF cerrada")


if __name__ == '__main__':
    sys.exit(main())
