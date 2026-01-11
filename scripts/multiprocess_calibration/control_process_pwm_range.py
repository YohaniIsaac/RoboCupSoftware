"""Proceso de control para encontrar rango PWM útil.

Este proceso maneja la UI y control RF para determinar el PWM máximo
que la cámara puede detectar consistentemente.
"""

import sys
import time
import logging
from pathlib import Path

import cv2
import numpy as np

# Agregar src al path
ROOT_DIR = Path(__file__).parent.parent.parent
sys.path.insert(0, str(ROOT_DIR / "src"))

# pylint: disable=wrong-import-position
from robot_soccer.communication.rf_controller import RFController

log = logging.getLogger(__name__)


def control_loop_pwm_range(robot_positions_pipe, frame_pipe, robot_id, serial_port):
    """Bucle principal del proceso de control para PWM range.

    Args:
        robot_positions_pipe: Pipe para recibir posiciones
        frame_pipe: Pipe para recibir frames procesados
        robot_id: ID del robot a probar (0-3)
        serial_port: Puerto serial para comunicación RF
    """
    log.info(f"🎮 Proceso de búsqueda PWM iniciado para Robot ID {robot_id}")

    # Inicializar RF controller
    rf_controller = None
    try:
        log.info("🔌 Iniciando comunicación RF...")
        rf_controller = RFController(
            port=serial_port,
            enable_calibration=False,
            min_command_interval=0.005
        )
        if rf_controller.initialize():
            log.info("✅ Conexión Serial establecida")
        else:
            log.warning("⚠️  No se pudo conectar al transmisor")
    except Exception as e:
        log.warning(f"⚠️  Error RF: {e}")

    # Parámetros de prueba
    current_pwm = 30
    movement_duration = 1.0
    movement_active = False
    movement_direction = 1  # 1=adelante, -1=atrás
    movement_start_time = 0
    current_left_speed = 0
    current_right_speed = 0

    # Estadísticas
    detection_count = 0
    last_detection_time = 0

    def send_motor_command(left_speed, right_speed):
        """Envía comando de motor."""
        if not rf_controller:
            return
        firmware_id = robot_id + 1
        rf_controller.set_motors(firmware_id, left_speed, right_speed)

    try:
        while True:
            current_time = time.time()

            # ===== CONTROL DE MOVIMIENTO (PRIMERA PRIORIDAD) =====
            if movement_active:
                time_elapsed = current_time - movement_start_time

                if time_elapsed < movement_duration:
                    # Enviar comando continuamente
                    send_motor_command(current_left_speed, current_right_speed)
                else:
                    # Detener
                    send_motor_command(0, 0)
                    movement_active = False
                    print(f"⏹️  Movimiento completado ({time_elapsed:.2f}s)")

            # ===== RECIBIR DATOS DE PERCEPCIÓN =====
            robot_positions = {}
            frame_display = None

            # Recibir posiciones (non-blocking)
            if robot_positions_pipe.poll():
                robot_positions = robot_positions_pipe.recv()

            # Recibir frame (non-blocking)
            if frame_pipe.poll():
                frame_display = frame_pipe.recv()

            # Si no hay frame, esperar al siguiente ciclo
            if frame_display is None:
                time.sleep(0.001)
                continue

            # Verificar detección del robot
            robot_detected = robot_id in robot_positions
            if robot_detected:
                last_detection_time = current_time
                detection_count += 1
                robot_pos = robot_positions[robot_id]
                robot_info = (
                    f"Robot {robot_id}: DETECTADO | "
                    f"Pos: ({robot_pos['x']:.0f}, {robot_pos['y']:.0f}) | "
                    f"Ángulo: {robot_pos['angulo']:.1f}°"
                )
                color = (0, 255, 0)  # Verde
            else:
                robot_info = f"Robot {robot_id}: NO DETECTADO"
                color = (0, 0, 255)  # Rojo

            # ===== PANEL DE INFORMACIÓN =====
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
                status = "DETENIDO (ESPACIO=adelante, BACKSPACE=atras, x=detener)"
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

            # ===== PROCESAR TECLAS =====
            key = cv2.waitKey(1) & 0xFF

            if key == 27:  # ESC
                log.info("ESC presionado - Saliendo...")
                break

            elif key == ord(' '):  # ESPACIO - adelante
                movement_active = True
                movement_direction = 1
                movement_start_time = time.time()
                current_left_speed = current_pwm
                current_right_speed = current_pwm
                send_motor_command(current_left_speed, current_right_speed)
                print(f"▶️  Movimiento ADELANTE | PWM: {current_pwm} | Duración: {movement_duration}s")

            elif key == 8:  # BACKSPACE - atrás
                movement_active = True
                movement_direction = -1
                movement_start_time = time.time()
                current_left_speed = -current_pwm
                current_right_speed = -current_pwm
                send_motor_command(current_left_speed, current_right_speed)
                print(f"◀️  Movimiento ATRAS | PWM: {current_pwm} | Duración: {movement_duration}s")

            elif key == ord('x'):  # X - Detener
                if movement_active:
                    send_motor_command(0, 0)
                    movement_active = False
                    print("🛑 Movimiento detenido manualmente")

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
        log.info("⏹️  Proceso de control detenido por usuario")
    except Exception as e:
        log.error(f"❌ Error en proceso de control: {e}", exc_info=True)
    finally:
        # Detener robot
        if rf_controller:
            firmware_id = robot_id + 1
            rf_controller.set_motors(firmware_id, 0, 0)
            rf_controller.shutdown()
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
