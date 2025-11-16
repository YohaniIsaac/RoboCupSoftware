#!/usr/bin/env python3
"""Robot Soccer Simulation."""

import argparse
import logging
import multiprocessing
from robot_soccer.core.game_controller import execute_multiprocessing

# Configuración global del logging
logging.basicConfig(
    level=logging.DEBUG,
    format='%(levelname)-8s - %(filename)-15s - %(message)s'
)

# Control de niveles por módulo
logging.getLogger('robot_soccer.core').setLevel(logging.INFO)
logging.getLogger('robot_soccer.core.process').setLevel(logging.DEBUG)
logging.getLogger('robot_soccer.core.physics').setLevel(logging.WARNING)
logging.getLogger('robot_soccer.perception').setLevel(logging.INFO)
logging.getLogger('robot_soccer.entities').setLevel(logging.ERROR)
logging.getLogger('robot_soccer.ai').setLevel(logging.DEBUG)
logging.getLogger('robot_soccer.ai.path_planning').setLevel(logging.WARNING)
logging.getLogger('robot_soccer.utils').setLevel(logging.INFO)

# Librerías externas
logging.getLogger('pygame').setLevel(logging.WARNING)
logging.getLogger('opencv').setLevel(logging.ERROR)
logging.getLogger('numpy').setLevel(logging.ERROR)

def main():
    """Función principal que inicia la simulación del juego."""
    # Configurar argumentos de línea de comandos
    parser = argparse.ArgumentParser(
        description='Robot Soccer Simulation',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Ejemplos de uso:

  MODO COMPLETO (default - todos los módulos):
    python -m robot_soccer                    # Con simulación
    python -m robot_soccer --camera           # Con cámara física

  SOLO PERCEPCIÓN (probar detección):
    python -m robot_soccer --perception       # Simulación + detección pelota/jugadores
    python -m robot_soccer --perception --camera  # Cámara + detección pelota/jugadores

  SOLO PLANIFICACIÓN DE RUTAS:
    python -m robot_soccer --path-planning    # Solo planificador de trayectorias

  MÓDULOS COMBINADOS:
    python -m robot_soccer --perception --path-planning  # Percepción + rutas

  SOLO VIDEO (debug):
    python -m robot_soccer --video-only               # Solo simulación, sin procesamiento
    python -m robot_soccer --video-only --camera      # Solo cámara, sin procesamiento

  OTRAS OPCIONES:
    python -m robot_soccer --camera --camera-id 0  # Usar cámara específica

Nota: Para --camera, inicia droidcam-cli primero:
  cd AlgortimosBasicos/ArucoTag && ./start_droidcam.sh
        """
    )

    parser.add_argument(
        '--camera',
        action='store_true',
        help='Usar cámara física en lugar de simulación'
    )

    parser.add_argument(
        '--camera-id',
        type=int,
        default=2,
        help='ID de la cámara a usar (default: 2 para DroidCam)'
    )

    # Flags para módulos específicos
    parser.add_argument(
        '--perception',
        action='store_true',
        help='Ejecutar solo módulo de percepción (cámara/simulación + detección pelota + jugadores)'
    )

    parser.add_argument(
        '--path-planning',
        action='store_true',
        help='Ejecutar solo módulo de planificación de rutas'
    )

    parser.add_argument(
        '--full',
        action='store_true',
        help='Ejecutar todos los módulos (comportamiento default)'
    )

    parser.add_argument(
        '--video-only',
        action='store_true',
        help='Ejecutar solo el proceso de captura de video (cámara/simulación) sin procesamiento'
    )

    args = parser.parse_args()

    # Determinar qué módulos ejecutar
    # Si se especifica --video-only, solo ejecutar captura de video
    if args.video_only:
        modules = {
            'perception': False,
            'path_planning': False,
            'full': False,
            'video_only': True
        }
    # Si no se especifica ninguna flag de módulo, ejecutar todo (--full implícito)
    elif not (args.perception or args.path_planning):
        modules = {
            'perception': True,
            'path_planning': True,
            'full': False,
            'video_only': False
        }
    else:
        modules = {
            'perception': args.perception,
            'path_planning': args.path_planning,
            'full': args.full,
            'video_only': False
        }

    # Configurar multiprocessing si es necesario
    multiprocessing.set_start_method('spawn')

    # Crear e iniciar el controlador principal del juego
    execute_multiprocessing(
        use_camera=args.camera,
        camera_id=args.camera_id,
        modules=modules
    )


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nPrograma terminado por el usuario")
    except Exception as e:
        print(f"Error inesperado: {e}")
