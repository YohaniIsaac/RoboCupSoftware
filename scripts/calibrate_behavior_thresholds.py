#!/usr/bin/env python3
"""Script de calibración interactiva para umbrales de comportamiento del robot.

ETAPA 4 del proceso de calibración: Ajustar políticas de control y umbrales
de comportamiento de alto nivel.

Este script permite ajustar:
- Umbral de distancia para considerar waypoint alcanzado
- Umbral angular para considerar orientación correcta
- Umbral angular para iniciar movimiento lineal (vs girar en lugar)
- Corrección angular máxima durante movimiento lineal

Prerequisitos:
    - ETAPA 0: Calibración de hardware (perspectiva, pelota) completada
    - ETAPA 1: Calibración de motores individuales completada
    - ETAPA 2: Calibración de límites PWM completada
    - ETAPA 3: Calibración de controladores PID completada

Utiliza multiprocessing para separar la percepción del control, permitiendo
detección continua sin interrupciones.

Uso:
    # Robot con ID 0 (default)
    python scripts/calibrate_behavior_thresholds.py --robot-id 0

    # Robot con ID específico (0, 1, 2, 3)
    python scripts/calibrate_behavior_thresholds.py --robot-id 2

    # Especificar cámara y puerto serial
    python scripts/calibrate_behavior_thresholds.py --robot-id 1 --camera-id 2 --serial-port /dev/ttyUSB0

Controles:
    MOVIMIENTO DEL WAYPOINT:
    Flechas (↑↓←→): Mover waypoint objetivo (10px por paso)

    THRESHOLDS (±1 por pulsación):
    9/0: Aumentar/Disminuir position_threshold (px)
    -/=: Disminuir/Aumentar angle_threshold (grados)
    [/]: Disminuir/Aumentar linear_start_angle_threshold (grados)

    CORRECCIÓN ANGULAR (±1 PWM por pulsación):
    ,/.: Disminuir/Aumentar max_angular_correction_pwm

    CALIBRACIÓN DE CAPTURA (±1 px por pulsación):
    U/J: Aumentar/Disminuir capture_activate_distance_px
    I/K: Aumentar/Disminuir capture_overshoot_px
    O/L: Aumentar/Disminuir capture_confirm_distance_px

    KICK_POINT (±1 px por pulsación, calibración geométrica del solenoide):
    / / \\ : Aumentar/Disminuir KICK_POINT_OFFSET_PX
             (centro marker ArUco → punto de impacto del solenoide)
    '  / `  : Aumentar/Disminuir KICK_POINT_TOLERANCE_PX
             (tolerancia bola↔kick_point para confirmar contacto)

    Procedimiento del kick_point:
    1. Coloca un robot detectado y la pelota delante de él tocando el dribbler.
    2. Verifica visualmente que el robot esté en posición ideal de kick.
    3. Ajusta el offset (/ y \\) hasta que la cruz cyan del kick_point
       coincida con el centro de la pelota.
    4. Ajusta la tolerancia (' y `) — debe abrazar el rango razonable de
       posiciones aceptables de la pelota antes del disparo.
    5. ENTER guarda los valores en config.py.

    Nota sobre distorsión de cámara: la homografía warpPerspective NO
    corrige fish-eye/barrel ni paralaje del marker (que está sobre el
    robot). El offset puede variar 5-15 px entre centro y bordes del
    campo. Si la varianza importa, calibrar en el centro y mover el
    robot a varias zonas para verificar que la cruz sigue cerca de la
    pelota; usar tolerancia más generosa si la varianza es ≤10 px.

    DRIBBLER (PWM directo 0-255, ±1 por pulsación):
    1/2: Disminuir/Aumentar dribbler_capture_power (PWM durante captura)
    3/4: Disminuir/Aumentar dribbler_hold_power (PWM al mantener pelota)

    PULSO INTERMITENTE (±10ms por pulsación, 0=continuo):
    5/6: Disminuir/Aumentar dribbler_pulse_on_ms (tiempo encendido)
    7/8: Disminuir/Aumentar dribbler_pulse_off_ms (tiempo apagado)

    CONTROL DE MOVIMIENTO:
    ESPACIO: Fase 1 - Robot navega a posición DETRÁS de la pelota, luego alinea
    D: Fase 2 - Creep lento hacia la pelota hasta confirmar contacto
    G: Fase 3 - Dispara SOLENOIDE (kick) solo si fase = CONFIRMADO
    X: Cancelar waypoint y reiniciar

    OTROS CONTROLES:
    ENTER: Guardar parámetros a config.py
    ESC: Salir

Objetivo:
    - Ajustar cuándo el robot considera que "llegó" al waypoint
    - Ajustar cuándo el robot considera que está "bien orientado"
    - Ajustar cuándo el robot cambia de "girar en lugar" a "moverse mientras corrige"
    - Ajustar cuánto puede corregir ángulo durante movimiento lineal

Metodología:
    1. Test de precisión de posición:
       - Crear waypoint y observar si robot para en el lugar correcto
       - Si se pasa: aumentar position_threshold
       - Si para muy lejos: disminuir position_threshold

    2. Test de precisión angular:
       - Crear waypoint que requiera orientación específica
       - Si oscila: aumentar angle_threshold
       - Si queda desalineado: disminuir angle_threshold

    3. Test de política de movimiento:
       - Crear waypoints con diferentes ángulos iniciales
       - Si gira demasiado antes de moverse: aumentar linear_start_angle_threshold
       - Si se mueve cuando está mal orientado: disminuir linear_start_angle_threshold

    4. Test de corrección angular:
       - Observar si robot mantiene trayectoria recta
       - Si oscila durante movimiento: disminuir max_angular_correction_pwm
       - Si se desvía mucho: aumentar max_angular_correction_pwm

Arquitectura:
    - Proceso de percepción: Detección continua de robots con ArUco
    - Proceso de control: UI, teclado y comandos RF
    - Comunicación no bloqueante entre procesos via pipes
"""

import sys
import logging
import argparse
from pathlib import Path

# Agregar scripts al path para importar módulos de multiprocess_calibration
SCRIPTS_DIR = Path(__file__).parent
sys.path.insert(0, str(SCRIPTS_DIR))

from multiprocess_calibration.calibrate_behavior_mp import run_behavior_calibration

# Configurar logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s [%(processName)-12s] %(levelname)-8s %(message)s',
    datefmt='%H:%M:%S'
)

log = logging.getLogger(__name__)


def main():
    """Función principal - launcher de calibración de comportamiento."""
    parser = argparse.ArgumentParser(
        description='Calibración de umbrales de comportamiento con multiprocessing (ETAPA 4)',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__
    )
    parser.add_argument(
        '--robot-id',
        type=int,
        choices=[0, 1, 2, 3],
        required=True,
        help='ID del robot a controlar (0-3)'
    )
    parser.add_argument(
        '--serial-port',
        type=str,
        default='/dev/ttyUSB0',
        help='Puerto serial para comunicación RF (default: /dev/ttyUSB0)'
    )
    parser.add_argument(
        '--camera-id',
        type=int,
        default=None,
        help='ID de la cámara (si no se especifica, auto-detecta DroidCam)'
    )

    args = parser.parse_args()

    log.info("=" * 70)
    log.info("  ETAPA 4: CALIBRACIÓN DE UMBRALES DE COMPORTAMIENTO")
    log.info("=" * 70)
    log.info("Objetivo: Ajustar políticas de control de alto nivel")
    log.info("Prerequisitos: ETAPA 1, 2 y 3 completadas")
    log.info("=" * 70)
    log.info("")

    # Ejecutar calibración de comportamiento
    run_behavior_calibration(
        robot_id=args.robot_id,
        serial_port=args.serial_port,
        camera_id=args.camera_id
    )


if __name__ == '__main__':
    main()
