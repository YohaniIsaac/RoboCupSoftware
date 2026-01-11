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
from robot_soccer.controllers.robot_calibration_multipoint import RobotCalibrationMultipoint

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

    # Inicializar calibration manager
    calibration = RobotCalibrationMultipoint()
    pwm_min_saved, pwm_max_saved = calibration.get_pwm_range(robot_id)
    log.info(f"📊 Rango PWM actual en JSON: [{pwm_min_saved}, {pwm_max_saved}]")

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

    # Rango PWM temporal (para determinar antes de guardar)
    pwm_min_temp = pwm_min_saved
    pwm_max_temp = pwm_max_saved

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
            panel_height = 220
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
            cv2.putText(panel, f"PWM actual: {current_pwm}  (↑/↓: ±5, w/s: ±1)", (10, 100),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            cv2.putText(panel, f"Duracion: {movement_duration:.1f}s  (+/-: ±0.5s, [/]: ±0.1s)", (10, 130),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

            # Rango PWM (temporal - no guardado aún)
            range_text = f"Rango: [{pwm_min_temp}, {pwm_max_temp}]  (n/m: PWM_min, ,/.: PWM_max)"
            cv2.putText(panel, range_text, (10, 160),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (100, 255, 100), 2)

            # Estadísticas y controles
            time_since_detection = current_time - last_detection_time if last_detection_time > 0 else 999
            stats = f"Detecciones: {detection_count} | Ultima: {time_since_detection:.1f}s | g=Guardar rango"
            cv2.putText(panel, stats, (10, 190),
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

            # Ajustar rango PWM temporal
            elif key == ord('n'):  # PWM_min -1
                pwm_min_temp = max(5, pwm_min_temp - 1)
                print(f"📉 PWM_min: {pwm_min_temp}")

            elif key == ord('m'):  # PWM_min +1
                pwm_min_temp = min(pwm_max_temp - 1, pwm_min_temp + 1)
                print(f"📈 PWM_min: {pwm_min_temp}")

            elif key == ord(','):  # PWM_max -1
                pwm_max_temp = max(pwm_min_temp + 1, pwm_max_temp - 1)
                print(f"📉 PWM_max: {pwm_max_temp}")

            elif key == ord('.'):  # PWM_max +1
                pwm_max_temp = min(127, pwm_max_temp + 1)
                print(f"📈 PWM_max: {pwm_max_temp}")

            elif key == ord('r'):  # Usar current_pwm como referencia
                print(f"\n💡 Sugerencias basadas en PWM actual ({current_pwm}):")
                print(f"   PWM_min sugerido: {max(5, current_pwm - 50)}")
                print(f"   PWM_max sugerido: {min(127, current_pwm + 50)}")
                print("   Usa n/m para ajustar PWM_min, ,/. para PWM_max\n")

            elif key == ord('g'):  # Guardar rango
                if pwm_min_temp >= pwm_max_temp:
                    print(f"❌ Error: PWM_min ({pwm_min_temp}) debe ser menor que PWM_max ({pwm_max_temp})")
                else:
                    print(f"\n💾 Guardando rango PWM: [{pwm_min_temp}, {pwm_max_temp}]")
                    calibration.set_pwm_range(robot_id, pwm_min_temp, pwm_max_temp)
                    calibration.save()
                    print(f"✅ Rango guardado para Robot {robot_id}")
                    print(f"⚠️  Los puntos de calibración fueron regenerados con valores neutros")
                    print(f"   Ejecuta 'calibrate_robot_motors_multipoint.py --robot-id {robot_id}' para calibrar\n")

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
        print(f"Rango temporal: [{pwm_min_temp}, {pwm_max_temp}]")

        # Verificar si el rango fue guardado
        current_saved_min, current_saved_max = calibration.get_pwm_range(robot_id)
        if (current_saved_min, current_saved_max) == (pwm_min_temp, pwm_max_temp):
            print(f"✅ Rango GUARDADO en JSON")
        else:
            print(f"⚠️  Rango NO guardado (presiona 'g' para guardar)")

        print("\n💡 Proceso sugerido:")
        print("   1. Encuentra PWM_min: PWM más bajo donde el robot se mueve consistentemente")
        print("   2. Encuentra PWM_max: PWM más alto donde la cámara detecta al robot")
        print("   3. Ajusta rango con n/m (min) y ,/. (max)")
        print("   4. Presiona 'g' para guardar")
        print("   5. Calibra con: calibrate_robot_motors_multipoint.py --robot-id " + str(robot_id))
        print("=" * 70 + "\n")
