#!/usr/bin/env python3
"""Script para encontrar el rango PWM útil de cada robot.

Este script permite probar diferentes valores PWM y ver si la cámara
puede detectar al robot a esa velocidad. Útil para determinar el
rango operativo real antes de calibrar.

Uso:
    python scripts/find_pwm_range.py --robot-id 0

Controles:
    ESPACIO     : Iniciar/detener movimiento hacia adelante
    BACKSPACE   : Iniciar/detener movimiento hacia atrás
    ↑/↓         : Ajustar PWM ±5
    w/s         : Ajustar PWM ±1 (fino)
    +/-         : Ajustar duración ±0.5s
    [/]         : Ajustar duración ±0.1s (fino)
    ESC         : Salir
"""

import sys
import cv2
import time
import logging
import argparse
import numpy as np
from pathlib import Path

# Agregar src al path
ROOT_DIR = Path(__file__).parent.parent
sys.path.insert(0, str(ROOT_DIR / "src"))

from robot_soccer.perception.aruco_detector import ArucoDetector
from robot_soccer.communication.rf_controller import RFController
from robot_soccer.utils.camera_utils import get_camera_index

# Configurar logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
log = logging.getLogger(__name__)

# Silenciar logs excesivos
logging.getLogger('robot_soccer.communication.serial_manager').setLevel(logging.WARNING)
logging.getLogger('robot_soccer.communication.rf_controller').setLevel(logging.WARNING)


def main():
    """Función principal."""
    parser = argparse.ArgumentParser(
        description='Encuentra el rango PWM útil para calibración'
    )
    parser.add_argument(
        '--robot-id',
        type=int,
        required=True,
        choices=[0, 1, 2, 3],
        help='ID del robot a probar (0-3)'
    )
    parser.add_argument(
        '--port',
        type=str,
        default='/dev/ttyUSB0',
        help='Puerto serial RF (default: /dev/ttyUSB0)'
    )
    parser.add_argument(
        '--camera',
        type=int,
        default=None,
        help='ID de cámara (default: auto-detectar)'
    )

    args = parser.parse_args()
    robot_id = args.robot_id
    firmware_id = robot_id + 1

    # Auto-detectar cámara
    camera_id = args.camera
    if camera_id is None:
        camera_id = get_camera_index(prefer_droidcam=True, fallback_index=0)
        log.info(f"📷 Cámara auto-detectada: /dev/video{camera_id}")

    print("\n" + "=" * 70)
    print("BÚSQUEDA DE RANGO PWM ÚTIL")
    print("=" * 70)
    print(f"Robot ID: {robot_id}")
    print(f"Puerto serial: {args.port}")
    print(f"Cámara: /dev/video{camera_id}")
    print("=" * 70)
    print("\nControles:")
    print("  ESPACIO     → Mover adelante")
    print("  BACKSPACE   → Mover atrás")
    print("  ↑/↓         → PWM ±5")
    print("  w/s         → PWM ±1 (fino)")
    print("  +/-         → Duración ±0.5s")
    print("  [/]         → Duración ±0.1s (fino)")
    print("  ESC         → Salir")
    print("=" * 70 + "\n")

    # Inicializar cámara
    cap = cv2.VideoCapture(camera_id)
    if not cap.isOpened():
        log.error(f"❌ No se pudo abrir la cámara /dev/video{camera_id}")
        return 1

    # Configurar resolución
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    cap.set(cv2.CAP_PROP_FPS, 30)

    # Inicializar detector ArUco
    aruco_detector = ArucoDetector()

    # Inicializar RF controller
    rf_controller = RFController(port=args.port, enable_calibration=False, min_command_interval=0.005)
    if not rf_controller.initialize():
        log.error("❌ No se pudo inicializar RF controller")
        cap.release()
        return 1

    log.info("✅ Sistema inicializado\n")

    # Parámetros de prueba
    current_pwm = 30
    movement_duration = 1.0
    movement_active = False
    movement_direction = 1  # 1=adelante, -1=atrás
    movement_start_time = 0
    last_detection_time = 0
    detection_count = 0

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                log.warning("⚠️  No se pudo leer frame de cámara")
                break

            # Detectar robots
            robot_positions = aruco_detector.detect_robots(frame)
            frame_display = aruco_detector.draw_robots(frame, robot_positions)

            # Info del robot target
            robot_detected = robot_id in robot_positions
            current_time = time.time()

            if robot_detected:
                last_detection_time = current_time
                detection_count += 1
                robot_pos = robot_positions[robot_id]
                robot_info = (
                    f"Robot {robot_id}: DETECTADO | "
                    f"Pos: ({robot_pos['x']:.0f}, {robot_pos['y']:.0f}) | "
                    f"Ángulo: {robot_pos['angle']:.1f}°"
                )
                color = (0, 255, 0)  # Verde
            else:
                robot_info = f"Robot {robot_id}: NO DETECTADO"
                color = (0, 0, 255)  # Rojo

            # Panel de información (más grande)
            panel_height = 180
            panel = np.zeros((panel_height, frame_display.shape[1], 3), dtype=np.uint8)

            # Información del robot
            cv2.putText(panel, robot_info, (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)

            # Estado del movimiento
            if movement_active:
                elapsed = current_time - movement_start_time
                remaining = max(0, movement_duration - elapsed)
                direction_text = "ADELANTE" if movement_direction > 0 else "ATRAS"
                status = f"MOVIENDO {direction_text} | Tiempo: {remaining:.1f}s"
                status_color = (0, 255, 255)  # Amarillo
            else:
                status = "DETENIDO (ESPACIO=adelante, BACKSPACE=atras)"
                status_color = (200, 200, 200)  # Gris

            cv2.putText(panel, status, (10, 65),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, status_color, 2)

            # Parámetros actuales
            cv2.putText(panel, f"PWM: {current_pwm}  (↑/↓: ±5, w/s: ±1)", (10, 100),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            cv2.putText(panel, f"Duracion: {movement_duration:.1f}s  (+/-: ±0.5s, [/]: ±0.1s)", (10, 130),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

            # Estadísticas
            time_since_detection = current_time - last_detection_time if last_detection_time > 0 else 999
            stats = f"Detecciones: {detection_count} | Ultima: {time_since_detection:.1f}s"
            cv2.putText(panel, stats, (10, 160),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (150, 150, 150), 1)

            # Combinar panel y frame
            combined = np.vstack([panel, frame_display])
            cv2.imshow('Busqueda de Rango PWM', combined)

            # Control de movimiento
            if movement_active:
                time_elapsed = current_time - movement_start_time

                if time_elapsed < movement_duration:
                    # Enviar comando
                    pwm_value = current_pwm * movement_direction
                    rf_controller.set_motors(firmware_id, pwm_value, pwm_value)
                else:
                    # Detener
                    rf_controller.set_motors(firmware_id, 0, 0)
                    movement_active = False
                    print(f"⏹️  Movimiento completado")

            # Procesar teclas
            key = cv2.waitKey(1) & 0xFF

            if key == 27:  # ESC
                print("\n👋 Saliendo...")
                break

            elif key == ord(' '):  # ESPACIO - adelante
                if not movement_active:
                    movement_active = True
                    movement_direction = 1
                    movement_start_time = time.time()
                    print(f"▶️  Iniciando movimiento ADELANTE | PWM: {current_pwm} | Duración: {movement_duration}s")
                else:
                    rf_controller.set_motors(firmware_id, 0, 0)
                    movement_active = False
                    print("⏹️  Movimiento detenido")

            elif key == 8:  # BACKSPACE - atrás
                if not movement_active:
                    movement_active = True
                    movement_direction = -1
                    movement_start_time = time.time()
                    print(f"◀️  Iniciando movimiento ATRAS | PWM: {current_pwm} | Duración: {movement_duration}s")
                else:
                    rf_controller.set_motors(firmware_id, 0, 0)
                    movement_active = False
                    print("⏹️  Movimiento detenido")

            elif key == 82:  # ↑
                current_pwm = min(127, current_pwm + 5)
                print(f"⬆️  PWM: {current_pwm}")

            elif key == 84:  # ↓
                current_pwm = max(5, current_pwm - 5)
                print(f"⬇️  PWM: {current_pwm}")

            elif key == ord('w'):
                current_pwm = min(127, current_pwm + 1)
                print(f"↗️  PWM: {current_pwm}")

            elif key == ord('s'):
                current_pwm = max(5, current_pwm - 1)
                print(f"↘️  PWM: {current_pwm}")

            elif key == ord('=') or key == ord('+'):
                movement_duration = min(10.0, movement_duration + 0.5)
                print(f"⏱️  Duración: {movement_duration:.1f}s")

            elif key == ord('-') or key == ord('_'):
                movement_duration = max(0.5, movement_duration - 0.5)
                print(f"⏱️  Duración: {movement_duration:.1f}s")

            elif key == ord(']'):
                movement_duration = min(10.0, movement_duration + 0.1)
                print(f"⏱️  Duración: {movement_duration:.1f}s")

            elif key == ord('['):
                movement_duration = max(0.1, movement_duration - 0.1)
                print(f"⏱️  Duración: {movement_duration:.1f}s")

            time.sleep(0.001)

    except KeyboardInterrupt:
        print("\n⏹️  Interrumpido por usuario")

    finally:
        # Detener robot
        rf_controller.set_motors(firmware_id, 0, 0)
        time.sleep(0.1)
        rf_controller.shutdown()
        cap.release()
        cv2.destroyAllWindows()

        print("\n" + "=" * 70)
        print("RESUMEN")
        print("=" * 70)
        print(f"Último PWM probado: {current_pwm}")
        print(f"Total de detecciones: {detection_count}")
        print("\n💡 Recomendación:")
        print("   Prueba aumentando PWM hasta que la cámara deje de detectar")
        print("   al robot de forma consistente. Ese será tu PWM_max útil.")
        print("=" * 70 + "\n")

    return 0


if __name__ == '__main__':
    sys.exit(main())
