"""Proceso de control para calibración de motores individuales.

Este proceso se ejecuta independientemente y:
- Recibe frames procesados desde el proceso de percepción
- Maneja la interfaz gráfica con panel de calibración
- Permite ajustar factores de calibración con el teclado
- Envía comandos RF al robot seleccionado
- Controla movimientos temporales con duración configurable
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

from robot_soccer.controllers.robot_calibration import RobotCalibration
from robot_soccer.communication.rf_controller import RFController

log = logging.getLogger(__name__)


def control_loop_motors(robot_positions_pipe, frame_pipe, robot_id, serial_port):
    """Bucle principal del proceso de control de calibración de motores.

    Args:
        robot_positions_pipe: Pipe para recibir posiciones de robots desde percepción
        frame_pipe: Pipe para recibir frames procesados desde percepción
        robot_id: ID del robot a calibrar (0-3)
        serial_port: Puerto serial para comunicación RF
    """
    log.info(f"🎮 Proceso de control de motores iniciado para Robot ID {robot_id}")

    # Inicializar RF controller con min_command_interval bajo para calibración
    rf_controller = None
    robot_available = False
    try:
        log.info("🔌 Iniciando comunicación RF...")
        rf_controller = RFController(
            port=serial_port,
            enable_calibration=False,  # Aplicar calibración manual en este script
            min_command_interval=0.005  # 5ms para movimiento ultra-fluido
        )
        if rf_controller.initialize():
            log.info("✅ Conexión Serial establecida")

            connections = rf_controller.test_connections()
            robot_key = f'robot_{robot_id + 1}'
            robot_available = connections.get(robot_key, False)

            if robot_available:
                log.info(f"✅ Robot {robot_id} disponible via RF")
            else:
                log.warning(f"⚠️  Robot {robot_id} NO responde via RF")
        else:
            log.warning("⚠️  No se pudo conectar al transmisor")
    except Exception as e:
        log.warning(f"⚠️  Error RF: {e}")

    # Cargar calibración existente
    calibration = RobotCalibration()
    max_left, max_right, bias = calibration.get_calibration(robot_id)
    log.info(f"📊 Calibración cargada - L={max_left:.3f}, R={max_right:.3f}, B={bias:+.3f}")

    # Estado de movimiento
    running = True
    current_left_speed = 0
    current_right_speed = 0
    movement_active = False
    movement_start_time = 0
    movement_duration = 0.3  # Duración por defecto

    def apply_calibration_manually(left_speed, right_speed):
        """Aplica calibración manual a las velocidades."""
        # Aplicar max_speed
        left_cal = left_speed * max_left
        right_cal = right_speed * max_right

        # Aplicar bias (corrección de deriva)
        if left_cal != 0 or right_cal != 0:
            bias_adjustment = bias * 127
            left_cal += bias_adjustment
            right_cal -= bias_adjustment

        # Limitar a rango PWM (-127 a 127)
        left_cal = max(-127, min(127, int(left_cal)))
        right_cal = max(-127, min(127, int(right_cal)))

        return left_cal, right_cal

    def send_motor_command(left_speed, right_speed):
        """Envía comando de motor con calibración aplicada."""
        if not rf_controller:
            return

        left_cal, right_cal = apply_calibration_manually(left_speed, right_speed)
        firmware_id = robot_id + 1
        rf_controller.set_motors(firmware_id, left_cal, right_cal)

    def stop_robot():
        """Detiene el robot."""
        if rf_controller:
            firmware_id = robot_id + 1
            rf_controller.set_motors(firmware_id, 0, 0)

    def create_control_panel():
        """Crea panel de información de calibración."""
        panel = np.zeros((680, 600, 3), dtype=np.uint8)

        # Título
        cv2.putText(panel, f"Calibracion Robot ID: {robot_id}", (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)

        # Estado de conexión RF
        y_offset = 60
        rf_status_color = (0, 255, 0) if robot_available else (0, 0, 255)
        rf_status_text = f"RF: {'CONECTADO' if robot_available else 'DESCONECTADO'}"
        cv2.putText(panel, rf_status_text, (10, y_offset),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, rf_status_color, 2)

        # Valores actuales
        y_offset = 100
        cv2.putText(panel, "Valores Actuales:", (10, y_offset),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

        y_offset += 35
        cv2.putText(panel, f"max_speed_left:   {max_left:.3f}", (20, y_offset),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

        y_offset += 30
        cv2.putText(panel, f"max_speed_right:  {max_right:.3f}", (20, y_offset),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

        y_offset += 30
        cv2.putText(panel, f"bias_correction:  {bias:+.3f}", (20, y_offset),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

        # Duración configurada
        y_offset += 50
        cv2.putText(panel, f"Duracion movimiento: {movement_duration:.3f}s", (20, y_offset),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)

        # Controles
        y_offset = 280
        cv2.putText(panel, "Controles:", (10, y_offset),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

        controls = [
            "AJUSTE GRUESO:",
            "  q/a: max_left   (+/-0.05)",
            "  w/s: max_right  (+/-0.05)",
            "  e/d: bias       (+/-0.01)",
            "",
            "AJUSTE FINO:",
            "  1/2: max_left   (+/-0.01)",
            "  3/4: max_right  (+/-0.01)",
            "  5/6: bias       (+/-0.005)",
            "",
            "MOVIMIENTO:",
            "  Flechas: Mover robot",
            "  [/]: Duracion   (+/-0.05s)",
            "  -/=: Duracion   (+/-0.01s)",
            "  ESPACIO: Detener",
            "",
            "ENTER: Guardar | ESC: Salir"
        ]

        y_offset += 25
        for control in controls:
            cv2.putText(panel, control, (20, y_offset),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 200, 200), 1)
            y_offset += 20

        return panel

    # Crear ventanas
    cv2.namedWindow('Robot View', cv2.WINDOW_NORMAL)
    cv2.namedWindow('Calibration Panel', cv2.WINDOW_NORMAL)

    log.info("✅ Proceso de control iniciado - Ventanas creadas")
    log.info("💡 Usa las teclas para calibrar. Presiona ESC para salir.")

    last_frame = None
    last_robots_data = []

    try:
        while running:
            # Recibir frame procesado desde percepción (sin bloqueo)
            if frame_pipe.poll():
                last_frame = frame_pipe.recv()

            # Recibir posiciones de robots (sin bloqueo)
            if robot_positions_pipe.poll():
                data = robot_positions_pipe.recv()
                last_robots_data = data.get('robots', [])

            # Si no hay frame, crear uno negro
            if last_frame is None:
                last_frame = np.zeros((480, 640, 3), dtype=np.uint8)
                cv2.putText(last_frame, "Esperando frames de percepcion...", (50, 240),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

            # Clonar frame para no modificar el original
            display_frame = last_frame.copy()

            # Marcar el robot objetivo
            robot_found = False
            for robot in last_robots_data:
                if robot['id'] == robot_id:
                    robot_found = True
                    # Dibujar borde destacado
                    cv2.circle(display_frame, (robot['x'], robot['y']),
                             60, (0, 255, 255), 3)
                    cv2.putText(display_frame, f"CALIBRANDO ID {robot_id}",
                              (robot['x'] - 80, robot['y'] - 70),
                              cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

            # Mostrar estado de detección
            status_color = (0, 255, 0) if robot_found else (0, 0, 255)
            status_text = f"Robot {robot_id}: {'DETECTADO' if robot_found else 'NO DETECTADO'}"
            cv2.putText(display_frame, status_text, (10, 30),
                      cv2.FONT_HERSHEY_SIMPLEX, 0.7, status_color, 2)

            # Mostrar comandos actuales
            motor_text = f"Motors: L={current_left_speed} R={current_right_speed}"
            cv2.putText(display_frame, motor_text,
                      (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)

            # Mostrar duración configurada
            duration_text = f"Duracion: {movement_duration:.3f}s ([/] ±0.05s, -/= ±0.01s)"
            cv2.putText(display_frame, duration_text,
                      (10, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)

            # Indicador de movimiento activo
            if movement_active:
                current_time = time.time()
                time_elapsed = current_time - movement_start_time
                time_remaining = max(0, movement_duration - time_elapsed)
                progress_pct = min(100, (time_elapsed / movement_duration) * 100)

                direction = ""
                if current_left_speed > 0 and current_right_speed > 0:
                    direction = "ADELANTE ↑"
                elif current_left_speed < 0 and current_right_speed < 0:
                    direction = "ATRAS ↓"
                elif current_left_speed < 0 and current_right_speed > 0:
                    direction = "GIRO IZQ ←"
                elif current_left_speed > 0 and current_right_speed < 0:
                    direction = "GIRO DER →"

                status_text = f"MOVIMIENTO: {direction} | {time_remaining:.3f}s ({progress_pct:.0f}%)"
                cv2.putText(display_frame, status_text,
                          (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            else:
                cv2.putText(display_frame, "MOVIMIENTO: DETENIDO",
                          (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

            # Crear panel de control
            panel = create_control_panel()

            # Mostrar ventanas
            cv2.imshow('Robot View', display_frame)
            cv2.imshow('Calibration Panel', panel)

            # Procesar teclas (espera 1ms)
            key = cv2.waitKey(1) & 0xFF

            # Procesar tecla
            if key == 27:  # ESC - Salir
                print("\n⚠️  Calibración cancelada - no se guardaron cambios")
                running = False

            elif key == 13:  # ENTER - Guardar
                calibration.set_calibration(robot_id, max_left, max_right, bias)
                calibration.save()
                print("\n✅ Calibración guardada exitosamente!")
                print(f"   Robot {robot_id}: L={max_left:.3f}, R={max_right:.3f}, B={bias:+.3f}")

            elif key == 32:  # ESPACIO - Stop
                movement_active = False
                stop_robot()
                current_left_speed = 0
                current_right_speed = 0
                print("⏹️  Detenido")

            # Flechas - Movimiento temporal
            elif key in [82, 0]:  # Flecha arriba
                current_left_speed = 100
                current_right_speed = 100
                movement_active = True
                movement_start_time = time.time()
                send_motor_command(100, 100)
                print(f"⬆️  Adelante ({movement_duration:.3f}s)")

            elif key in [84, 1]:  # Flecha abajo
                current_left_speed = -100
                current_right_speed = -100
                movement_active = True
                movement_start_time = time.time()
                send_motor_command(-100, -100)
                print(f"⬇️  Atrás ({movement_duration:.3f}s)")

            elif key in [81, 2]:  # Flecha izquierda
                current_left_speed = -80
                current_right_speed = 80
                movement_active = True
                movement_start_time = time.time()
                send_motor_command(-80, 80)
                print(f"⬅️  Girar izquierda ({movement_duration:.3f}s)")

            elif key in [83, 3]:  # Flecha derecha
                current_left_speed = 80
                current_right_speed = -80
                movement_active = True
                movement_start_time = time.time()
                send_motor_command(80, -80)
                print(f"➡️  Girar derecha ({movement_duration:.3f}s)")

            # Ajustar duración - GRUESO
            elif key == ord('['):
                movement_duration = max(0.05, movement_duration - 0.05)
                print(f"⏱️  Duración: {movement_duration:.2f}s")

            elif key == ord(']'):
                movement_duration = min(5.0, movement_duration + 0.05)
                print(f"⏱️  Duración: {movement_duration:.2f}s")

            # Ajustar duración - FINO
            elif key == ord('-'):
                movement_duration = max(0.05, movement_duration - 0.01)
                print(f"⏱️  Duración FINO: {movement_duration:.3f}s")

            elif key == ord('='):
                movement_duration = min(5.0, movement_duration + 0.01)
                print(f"⏱️  Duración FINO: {movement_duration:.3f}s")

            # Ajustar max_left - GRUESO
            elif key == ord('q'):
                max_left = min(2.0, max_left + 0.05)
                print(f"↗️  max_left: {max_left:.3f}")

            elif key == ord('a'):
                max_left = max(0.1, max_left - 0.05)
                print(f"↘️  max_left: {max_left:.3f}")

            # Ajustar max_right - GRUESO
            elif key == ord('w'):
                max_right = min(2.0, max_right + 0.05)
                print(f"↗️  max_right: {max_right:.3f}")

            elif key == ord('s'):
                max_right = max(0.1, max_right - 0.05)
                print(f"↘️  max_right: {max_right:.3f}")

            # Ajustar bias - GRUESO
            elif key == ord('e'):
                bias = min(0.5, bias + 0.01)
                print(f"↗️  bias: {bias:+.3f}")

            elif key == ord('d'):
                bias = max(-0.5, bias - 0.01)
                print(f"↘️  bias: {bias:+.3f}")

            # Ajustar max_left - FINO
            elif key == ord('1'):
                max_left = min(2.0, max_left + 0.01)
                print(f"↗️  max_left FINO: {max_left:.3f}")

            elif key == ord('2'):
                max_left = max(0.1, max_left - 0.01)
                print(f"↘️  max_left FINO: {max_left:.3f}")

            # Ajustar max_right - FINO
            elif key == ord('3'):
                max_right = min(2.0, max_right + 0.01)
                print(f"↗️  max_right FINO: {max_right:.3f}")

            elif key == ord('4'):
                max_right = max(0.1, max_right - 0.01)
                print(f"↘️  max_right FINO: {max_right:.3f}")

            # Ajustar bias - FINO
            elif key == ord('5'):
                bias = min(0.5, bias + 0.005)
                print(f"↗️  bias FINO: {bias:+.3f}")

            elif key == ord('6'):
                bias = max(-0.5, bias - 0.005)
                print(f"↘️  bias FINO: {bias:+.3f}")

            # Reset
            elif key == ord('r'):
                max_left = 1.0
                max_right = 1.0
                bias = 0.0
                print(f"🔄 Reset a valores neutros")

            # Control de movimiento temporal
            # Enviar comandos continuos mientras el movimiento está activo
            current_time = time.time()
            if movement_active:
                time_elapsed = current_time - movement_start_time

                # Verificar si el tiempo de movimiento ha expirado
                if time_elapsed < movement_duration:
                    # Enviar comando EN CADA ITERACIÓN para movimiento fluido
                    send_motor_command(current_left_speed, current_right_speed)
                else:
                    # Tiempo expirado - detener
                    movement_active = False
                    stop_robot()
                    current_left_speed = 0
                    current_right_speed = 0
                    print(f"🛑 Movimiento completado ({movement_duration:.3f}s)")

            # Pequeña pausa para no saturar el CPU
            time.sleep(0.001)

    except KeyboardInterrupt:
        log.info("⏹️  Proceso de control detenido por usuario")
    except Exception as e:
        log.error(f"❌ Error en proceso de control: {e}", exc_info=True)
    finally:
        stop_robot()
        if rf_controller:
            rf_controller.shutdown()
        cv2.destroyAllWindows()
        log.info("🔌 Proceso de control finalizado")
