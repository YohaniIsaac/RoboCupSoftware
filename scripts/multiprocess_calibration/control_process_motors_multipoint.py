"""Proceso de control para calibración multi-punto de motores.

Este proceso permite calibrar motores en 5 puntos diferentes de velocidad
y medir el dead-zone individual de cada motor.
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
from robot_soccer.controllers.robot_calibration_multipoint import (
    RobotCalibrationMultipoint,
    DEFAULT_CALIBRATION_POINTS
)
from robot_soccer.communication.rf_controller import RFController

log = logging.getLogger(__name__)


def control_loop_motors_multipoint(robot_positions_pipe, frame_pipe, robot_id, serial_port):
    # pylint: disable=too-many-branches,too-many-locals,too-many-statements
    """Bucle principal del proceso de control multi-punto.

    Args:
        robot_positions_pipe: Pipe para recibir posiciones
        frame_pipe: Pipe para recibir frames procesados
        robot_id: ID del robot a calibrar (0-3)
        serial_port: Puerto serial para comunicación RF
    """
    log.info(f"🎮 Proceso de calibración multi-punto iniciado para Robot ID {robot_id}")

    # Inicializar RF controller
    rf_controller = None
    robot_available = False
    try:
        log.info("🔌 Iniciando comunicación RF...")
        rf_controller = RFController(
            port=serial_port,
            enable_calibration=False,
            min_command_interval=0.005
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

    # Cargar calibración multi-punto
    calibration = RobotCalibrationMultipoint()

    # Estado de calibración
    current_point_index = 0  # Índice del punto actual (0-9 para 10 puntos bidireccionales)
    # Obtener puntos personalizados del robot (basados en su rango PWM)
    calibration_points = calibration.get_calibration_points_pwm(robot_id)
    pwm_min, pwm_max = calibration.get_pwm_range(robot_id)
    log.info(f"Robot {robot_id}: Usando puntos personalizados en rango [{pwm_min}, {pwm_max}]")
    log.info(f"Puntos: {calibration_points}")

    # Cargar valores existentes o inicializar neutros
    deadzone_left, deadzone_right = calibration.get_deadzone(robot_id)

    # Cargar puntos de calibración existentes
    point_calibrations = []
    for i in range(len(calibration_points)):
        point = calibration.get_calibration_point(robot_id, i)
        if point:
            point_calibrations.append({
                'max_left': point.max_left,
                'max_right': point.max_right,
                'bias': point.bias
            })
        else:
            point_calibrations.append({
                'max_left': 1.0,
                'max_right': 1.0,
                'bias': 0.0
            })

    # Estado de movimiento
    running = True
    current_left_speed = 0
    current_right_speed = 0
    movement_active = False
    movement_start_time = 0
    movement_duration = 0.3

    # Estado de frenado suave
    braking_active = False
    braking_start_time = 0
    braking_initial_left = 0
    braking_initial_right = 0
    braking_duration = 0.15

    # Modo de calibración
    calibration_mode = "MULTIPOINT"  # "MULTIPOINT" o "DEADZONE"
    deadzone_test_left_pwm = 0
    deadzone_test_right_pwm = 0

    def get_current_calibration():
        """Obtiene los valores de calibración del punto actual."""
        return point_calibrations[current_point_index]

    def apply_calibration_manually(left_speed, right_speed):
        """Aplica calibración manual del punto actual."""
        cal = get_current_calibration()

        left_cal = left_speed * cal['max_left']
        right_cal = right_speed * cal['max_right']

        if left_cal != 0 or right_cal != 0:
            bias_adjustment = cal['bias'] * 127
            left_cal += bias_adjustment
            right_cal -= bias_adjustment

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

    def stop_robot_smooth():
        """Inicia frenado suave del robot."""
        nonlocal braking_active, braking_start_time, braking_initial_left, braking_initial_right
        nonlocal movement_active

        if braking_active or (current_left_speed == 0 and current_right_speed == 0):
            return

        braking_active = True
        braking_start_time = time.time()
        braking_initial_left = current_left_speed
        braking_initial_right = current_right_speed
        movement_active = False

    def stop_robot_immediate():
        """Detiene el robot inmediatamente."""
        nonlocal braking_active, current_left_speed, current_right_speed
        if rf_controller:
            firmware_id = robot_id + 1
            rf_controller.set_motors(firmware_id, 0, 0)
        braking_active = False
        current_left_speed = 0
        current_right_speed = 0

    def create_control_panel():
        """Crea panel de información de calibración multi-punto."""
        panel = np.zeros((900, 1100, 3), dtype=np.uint8)

        # Título
        cv2.putText(panel, f"Calibracion Multi-Punto - Robot ID: {robot_id}", (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

        # Estado RF
        y_offset = 60
        rf_status_color = (0, 255, 0) if robot_available else (0, 0, 255)
        rf_status_text = f"RF: {'CONECTADO' if robot_available else 'DESCONECTADO'}"
        cv2.putText(panel, rf_status_text, (10, y_offset),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, rf_status_color, 2)

        # Modo de calibración
        y_offset = 95
        if calibration_mode == "MULTIPOINT":
            mode_text = f"MODO: MULTI-PUNTO ({current_point_index + 1}/10)"
            mode_color = (0, 255, 255)
        else:
            mode_text = "MODO: PRUEBA DEAD-ZONE"
            mode_color = (255, 165, 0)

        cv2.putText(panel, mode_text, (10, y_offset),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, mode_color, 2)

        # Sección de dead-zone
        y_offset = 135
        cv2.putText(panel, "Dead-Zone (PWM minimo):", (10, y_offset),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 200, 0), 2)

        y_offset += 30
        cv2.putText(panel, f"Motor Izq:  {deadzone_left} PWM  (z/x: +/-1)", (20, y_offset),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)

        y_offset += 25
        cv2.putText(panel, f"Motor Der:  {deadzone_right} PWM  (c/v: +/-1)", (20, y_offset),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)

        # Modo de prueba de dead-zone
        if calibration_mode == "DEADZONE":
            y_offset += 30
            cv2.putText(panel, f"PRUEBA: L={deadzone_test_left_pwm} | R={deadzone_test_right_pwm}", (20, y_offset),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 165, 0), 2)

        # Puntos de calibración - DOS COLUMNAS (ATRÁS | ADELANTE)
        y_offset += 45
        y_start = y_offset

        # COLUMNA IZQUIERDA: PUNTOS ATRÁS (negativos)
        cv2.putText(panel, "Puntos ATRAS (reversa):", (10, y_offset),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (100, 150, 255), 2)

        y_offset += 30
        for i, pwm in enumerate(calibration_points):
            if pwm < 0:  # Solo puntos negativos
                cal = point_calibrations[i]
                color = (0, 255, 0) if i == current_point_index else (120, 120, 120)
                marker = "→" if i == current_point_index else " "

                text = (f"{marker}[{i+1:2d}] {pwm:4d}: "
                        f"L={cal['max_left']:.4f} R={cal['max_right']:.4f} "
                        f"B={cal['bias']:+.4f}")
                cv2.putText(panel, text, (10, y_offset),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.42, color, 1)
                y_offset += 22

        # COLUMNA DERECHA: PUNTOS ADELANTE (positivos)
        y_offset = y_start  # Volver al inicio
        cv2.putText(panel, "Puntos ADELANTE:", (560, y_offset),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (100, 255, 150), 2)

        y_offset += 30
        for i, pwm in enumerate(calibration_points):
            if pwm > 0:  # Solo puntos positivos
                cal = point_calibrations[i]
                color = (0, 255, 0) if i == current_point_index else (120, 120, 120)
                marker = "→" if i == current_point_index else " "

                text = (f"{marker}[{i+1:2d}] {pwm:4d}: "
                        f"L={cal['max_left']:.4f} R={cal['max_right']:.4f} "
                        f"B={cal['bias']:+.4f}")
                cv2.putText(panel, text, (560, y_offset),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.42, color, 1)
                y_offset += 22

        # Ajustar y_offset para continuar después de ambas columnas
        y_offset = y_start + 30 + (5 * 22)  # Título + 5 puntos * altura

        # Valores actuales del punto - EN HORIZONTAL
        cal = get_current_calibration()
        y_offset += 25
        pwm_current = calibration_points[current_point_index]
        direction_current = "ATRÁS" if pwm_current < 0 else "ADELANTE"
        cv2.putText(panel, f"Punto Actual: [{current_point_index+1}/10] {pwm_current:+4d} PWM ({direction_current})",
                   (10, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 200, 0), 2)

        y_offset += 30
        text_values = (f"max_left: {cal['max_left']:.4f}  |  "
                       f"max_right: {cal['max_right']:.4f}  |  "
                       f"bias: {cal['bias']:+.4f}")
        cv2.putText(panel, text_values, (20, y_offset),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

        # Parámetros de movimiento - EN HORIZONTAL
        y_offset += 40
        cv2.putText(panel, "Parametros de Movimiento:", (10, y_offset),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 200, 0), 2)

        y_offset += 30
        text_params = f"Duracion: {movement_duration:.3f}s  |  Frenado: {braking_duration*1000:.0f}ms"
        cv2.putText(panel, text_params, (20, y_offset),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)

        # Controles - DOS COLUMNAS
        y_offset += 50
        y_controls_start = y_offset
        cv2.putText(panel, "Controles:", (10, y_offset),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

        if calibration_mode == "MULTIPOINT":
            # COLUMNA IZQUIERDA
            controls_left = [
                "",
                "NAVEGACION:",
                "  PgUp/PgDn: Cambiar punto (1-10)",
                "",
                "CALIBRACION GRUESA:",
                "  q/a: max_left   (+/-0.05)",
                "  w/s: max_right  (+/-0.05)",
                "  e/d: bias       (+/-0.01)",
                "",
                "CALIBRACION FINA (0.001):",
                "  1/2: max_left   (+/-0.001)",
                "  3/4: max_right  (+/-0.001)",
                "  5/6: bias       (+/-0.001)",
                "",
                "DEAD-ZONE:",
                "  z/x: Motor izq  (+/-1 PWM)",
                "  c/v: Motor der  (+/-1 PWM)"
            ]

            # COLUMNA DERECHA
            controls_right = [
                "",
                "PARAMETROS MOVIMIENTO:",
                "  [/]: Duracion   (+/-0.05s)",
                "  -/=: Duracion   (+/-0.01s)",
                "",
                "MOVIMIENTO:",
                "  Flechas: Mover @ velocidad punto",
                "  ESPACIO: Stop emergencia",
                "  r: Reset punto actual",
                "",
                "OTROS:",
                "  t: Modo prueba dead-zone",
                "  ENTER: Guardar | ESC: Salir"
            ]

            y_offset += 25
            for control in controls_left:
                cv2.putText(panel, control, (20, y_offset),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.40, (200, 200, 200), 1)
                y_offset += 18

            y_offset = y_controls_start + 25
            for control in controls_right:
                cv2.putText(panel, control, (560, y_offset),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.40, (200, 200, 200), 1)
                y_offset += 18

        else:
            # MODO DEAD-ZONE - CENTRADO
            controls = [
                "",
                "MODO PRUEBA DEAD-ZONE:",
                "",
                "CONTROL DIRECTO DE MOTORES:",
                "  7/8: Motor izq  (+/-1 PWM)",
                "  9/0: Motor der  (+/-1 PWM)",
                "",
                "AJUSTAR DEAD-ZONE:",
                "  z/x: Guardar motor izq",
                "  c/v: Guardar motor der",
                "",
                "OBJETIVO: Encontrar PWM minimo donde el motor empieza a moverse",
                "",
                "ESPACIO: Stop motores  |  t: Volver a modo multi-punto  |  ESC: Salir"
            ]

            y_offset += 25
            for control in controls:
                cv2.putText(panel, control, (20, y_offset),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.40, (200, 200, 200), 1)
                y_offset += 18

        return panel

    # Crear ventanas
    cv2.namedWindow('Robot View', cv2.WINDOW_NORMAL)
    cv2.namedWindow('Calibration Panel', cv2.WINDOW_NORMAL)

    log.info("✅ Proceso de control iniciado - Ventanas creadas")
    log.info("💡 Usa PgUp/PgDn para cambiar entre 10 puntos de calibración")
    log.info("   5 puntos ATRÁS (negativos) + 5 puntos ADELANTE (positivos)")

    last_frame = None
    last_robots_data = []

    try:
        while running:
            # Recibir frame procesado
            if frame_pipe.poll():
                last_frame = frame_pipe.recv()

            # Recibir posiciones de robots
            if robot_positions_pipe.poll():
                data = robot_positions_pipe.recv()
                last_robots_data = data.get('robots', [])

            # Frame por defecto
            if last_frame is None:
                last_frame = np.zeros((480, 640, 3), dtype=np.uint8)
                cv2.putText(last_frame, "Esperando frames de percepcion...", (50, 240),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

            display_frame = last_frame.copy()

            # Marcar robot objetivo
            robot_found = False
            for robot in last_robots_data:
                if robot['id'] == robot_id:
                    robot_found = True
                    cv2.circle(display_frame, (robot['x'], robot['y']),
                             60, (0, 255, 255), 3)
                    cv2.putText(display_frame, f"CALIBRANDO ID {robot_id}",
                              (robot['x'] - 80, robot['y'] - 70),
                              cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

            # Estado
            status_color = (0, 255, 0) if robot_found else (0, 0, 255)
            status_text = f"Robot {robot_id}: {'DETECTADO' if robot_found else 'NO DETECTADO'}"
            cv2.putText(display_frame, status_text, (10, 30),
                      cv2.FONT_HERSHEY_SIMPLEX, 0.7, status_color, 2)

            # Punto de calibración actual
            pwm_value = calibration_points[current_point_index]
            direction = "ATRAS" if pwm_value < 0 else "ADELANTE"
            point_text = f"Punto {current_point_index + 1}/10 ({pwm_value:+4d} PWM - {direction})"
            cv2.putText(display_frame, point_text,
                      (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 200, 0), 2)

            # Estado de movimiento
            if braking_active:
                current_time = time.time()
                braking_elapsed = current_time - braking_start_time
                braking_progress = min(100, (braking_elapsed / braking_duration) * 100)
                status_text = f"MOVIMIENTO: FRENANDO... ({braking_progress:.0f}%)"
                cv2.putText(display_frame, status_text,
                          (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 165, 255), 2)
            elif movement_active:
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

            # Panel de control
            panel = create_control_panel()

            # Mostrar ventanas
            cv2.imshow('Robot View', display_frame)
            cv2.imshow('Calibration Panel', panel)

            # Procesar teclas
            key = cv2.waitKey(1) & 0xFF

            # ESC - Salir
            if key == 27:
                print("\n⚠️  Calibración cancelada - no se guardaron cambios")
                running = False

            # ENTER - Guardar TODO
            elif key == 13:
                # Guardar dead-zone
                calibration.set_deadzone(robot_id, deadzone_left, deadzone_right)

                # Guardar todos los puntos
                for i in range(len(calibration_points)):
                    cal = point_calibrations[i]
                    calibration.set_calibration_point(
                        robot_id, i,
                        cal['max_left'], cal['max_right'], cal['bias']
                    )

                calibration.save()
                print("\n✅ Calibración multi-punto guardada exitosamente!")
                print(f"   Robot {robot_id}:")
                print(f"   - Dead-zone: L={deadzone_left}, R={deadzone_right}")
                print(f"   - {len(calibration_points)} puntos calibrados")

            # ESPACIO - Stop inmediato
            elif key == 32:
                movement_active = False
                stop_robot_immediate()
                print("⏹️  Detenido (emergencia)")

            # t - Cambiar modo
            elif key == ord('t'):
                if calibration_mode == "MULTIPOINT":
                    calibration_mode = "DEADZONE"
                    movement_active = False
                    stop_robot_immediate()
                    print("🔧 Modo: PRUEBA DEAD-ZONE")
                    print("   Usa 7/8 para motor izq, 9/0 para motor der")
                else:
                    calibration_mode = "MULTIPOINT"
                    stop_robot_immediate()
                    print("📊 Modo: MULTI-PUNTO")

            # PgUp/PgDn - Cambiar punto de calibración
            elif key == 85:  # PgUp
                current_point_index = (current_point_index + 1) % len(calibration_points)
                pwm = calibration_points[current_point_index]
                direction = "ATRÁS" if pwm < 0 else "ADELANTE"
                print(f"📍 Punto {current_point_index + 1}/10 ({pwm:+4d} PWM - {direction})")

            elif key == 86:  # PgDn
                current_point_index = (current_point_index - 1) % len(calibration_points)
                pwm = calibration_points[current_point_index]
                direction = "ATRÁS" if pwm < 0 else "ADELANTE"
                print(f"📍 Punto {current_point_index + 1}/10 ({pwm:+4d} PWM - {direction})")

            # Calibración del punto actual
            elif key == ord('q'):
                cal = get_current_calibration()
                cal['max_left'] = min(2.0, cal['max_left'] + 0.05)
                print(f"↗️  max_left: {cal['max_left']:.3f}")

            elif key == ord('a'):
                cal = get_current_calibration()
                cal['max_left'] = max(0.5, cal['max_left'] - 0.05)
                print(f"↘️  max_left: {cal['max_left']:.3f}")

            elif key == ord('w'):
                cal = get_current_calibration()
                cal['max_right'] = min(2.0, cal['max_right'] + 0.05)
                print(f"↗️  max_right: {cal['max_right']:.3f}")

            elif key == ord('s'):
                cal = get_current_calibration()
                cal['max_right'] = max(0.5, cal['max_right'] - 0.05)
                print(f"↘️  max_right: {cal['max_right']:.3f}")

            elif key == ord('e'):
                cal = get_current_calibration()
                cal['bias'] = min(0.5, cal['bias'] + 0.01)
                print(f"↗️  bias: {cal['bias']:+.3f}")

            elif key == ord('d'):
                cal = get_current_calibration()
                cal['bias'] = max(-0.5, cal['bias'] - 0.01)
                print(f"↘️  bias: {cal['bias']:+.3f}")

            # Ajuste fino - ULTRA FINO (0.001)
            elif key == ord('1'):
                cal = get_current_calibration()
                cal['max_left'] = min(2.0, cal['max_left'] + 0.001)
                print(f"↗️  max_left FINO: {cal['max_left']:.4f}")

            elif key == ord('2'):
                cal = get_current_calibration()
                cal['max_left'] = max(0.5, cal['max_left'] - 0.001)
                print(f"↘️  max_left FINO: {cal['max_left']:.4f}")

            elif key == ord('3'):
                cal = get_current_calibration()
                cal['max_right'] = min(2.0, cal['max_right'] + 0.001)
                print(f"↗️  max_right FINO: {cal['max_right']:.4f}")

            elif key == ord('4'):
                cal = get_current_calibration()
                cal['max_right'] = max(0.5, cal['max_right'] - 0.001)
                print(f"↘️  max_right FINO: {cal['max_right']:.4f}")

            elif key == ord('5'):
                cal = get_current_calibration()
                cal['bias'] = min(0.5, cal['bias'] + 0.001)
                print(f"↗️  bias FINO: {cal['bias']:+.4f}")

            elif key == ord('6'):
                cal = get_current_calibration()
                cal['bias'] = max(-0.5, cal['bias'] - 0.001)
                print(f"↘️  bias FINO: {cal['bias']:+.4f}")

            # Ajustar dead-zone
            elif key == ord('z'):
                deadzone_left = min(40, deadzone_left + 1)
                print(f"🔧 Dead-zone izq: {deadzone_left} PWM")

            elif key == ord('x'):
                deadzone_left = max(0, deadzone_left - 1)
                print(f"🔧 Dead-zone izq: {deadzone_left} PWM")

            elif key == ord('c'):
                deadzone_right = min(40, deadzone_right + 1)
                print(f"🔧 Dead-zone der: {deadzone_right} PWM")

            elif key == ord('v'):
                deadzone_right = max(0, deadzone_right - 1)
                print(f"🔧 Dead-zone der: {deadzone_right} PWM")

            # Ajustar duración del movimiento - GRUESO
            elif key == ord('['):
                movement_duration = max(0.05, movement_duration - 0.05)
                print(f"⏱️  Duración: {movement_duration:.2f}s")

            elif key == ord(']'):
                movement_duration = min(5.0, movement_duration + 0.05)
                print(f"⏱️  Duración: {movement_duration:.2f}s")

            # Ajustar duración del movimiento - FINO
            elif key == ord('-'):
                movement_duration = max(0.05, movement_duration - 0.01)
                print(f"⏱️  Duración FINO: {movement_duration:.3f}s")

            elif key == ord('='):
                movement_duration = min(5.0, movement_duration + 0.01)
                print(f"⏱️  Duración FINO: {movement_duration:.3f}s")

            # Reset punto actual
            elif key == ord('r'):
                cal = get_current_calibration()
                cal['max_left'] = 1.0
                cal['max_right'] = 1.0
                cal['bias'] = 0.0
                print(f"🔄 Reset punto {current_point_index + 1} a valores neutros")

            # Controles específicos del modo
            if calibration_mode == "MULTIPOINT":
                # Flechas - Movimiento usando velocidad del punto actual
                # Nota: speed puede ser negativo, usar abs() para magnitud
                speed = calibration_points[current_point_index]
                speed_magnitude = abs(speed)

                if key in [82, 0]:  # Flecha arriba - siempre usa magnitud positiva
                    current_left_speed = speed_magnitude
                    current_right_speed = speed_magnitude
                    movement_active = True
                    movement_start_time = time.time()
                    send_motor_command(speed_magnitude, speed_magnitude)
                    print(f"⬆️  Adelante @ {speed_magnitude} PWM")

                elif key in [84, 1]:  # Flecha abajo - siempre usa magnitud negativa
                    current_left_speed = -speed_magnitude
                    current_right_speed = -speed_magnitude
                    movement_active = True
                    movement_start_time = time.time()
                    send_motor_command(-speed_magnitude, -speed_magnitude)
                    print(f"⬇️  Atrás @ {speed_magnitude} PWM")

                elif key in [81, 2]:  # Flecha izquierda
                    turn_speed = int(speed_magnitude * 0.8)
                    current_left_speed = -turn_speed
                    current_right_speed = turn_speed
                    movement_active = True
                    movement_start_time = time.time()
                    send_motor_command(-turn_speed, turn_speed)
                    print(f"⬅️  Girar izquierda @ {turn_speed} PWM")

                elif key in [83, 3]:  # Flecha derecha
                    turn_speed = int(speed_magnitude * 0.8)
                    current_left_speed = turn_speed
                    current_right_speed = -turn_speed
                    movement_active = True
                    movement_start_time = time.time()
                    send_motor_command(turn_speed, -turn_speed)
                    print(f"➡️  Girar derecha @ {turn_speed} PWM")

            elif calibration_mode == "DEADZONE":
                # Controles directos de PWM para encontrar dead-zone
                if key == ord('7'):
                    deadzone_test_left_pwm = min(127, deadzone_test_left_pwm + 1)
                    if rf_controller:
                        firmware_id = robot_id + 1
                        rf_controller.set_motors(firmware_id, deadzone_test_left_pwm, deadzone_test_right_pwm)
                    print(f"⬆️  Motor IZQ: {deadzone_test_left_pwm} PWM")

                elif key == ord('8'):
                    deadzone_test_left_pwm = max(-127, deadzone_test_left_pwm - 1)
                    if rf_controller:
                        firmware_id = robot_id + 1
                        rf_controller.set_motors(firmware_id, deadzone_test_left_pwm, deadzone_test_right_pwm)
                    print(f"⬇️  Motor IZQ: {deadzone_test_left_pwm} PWM")

                elif key == ord('9'):
                    deadzone_test_right_pwm = min(127, deadzone_test_right_pwm + 1)
                    if rf_controller:
                        firmware_id = robot_id + 1
                        rf_controller.set_motors(firmware_id, deadzone_test_left_pwm, deadzone_test_right_pwm)
                    print(f"⬆️  Motor DER: {deadzone_test_right_pwm} PWM")

                elif key == ord('0'):
                    deadzone_test_right_pwm = max(-127, deadzone_test_right_pwm - 1)
                    if rf_controller:
                        firmware_id = robot_id + 1
                        rf_controller.set_motors(firmware_id, deadzone_test_left_pwm, deadzone_test_right_pwm)
                    print(f"⬇️  Motor DER: {deadzone_test_right_pwm} PWM")

            # Control de movimiento temporal y frenado suave
            current_time = time.time()

            if braking_active:
                braking_elapsed = current_time - braking_start_time

                if braking_elapsed < braking_duration:
                    decel_factor = 1.0 - (braking_elapsed / braking_duration)
                    current_left_speed = int(braking_initial_left * decel_factor)
                    current_right_speed = int(braking_initial_right * decel_factor)
                    send_motor_command(current_left_speed, current_right_speed)
                else:
                    braking_active = False
                    current_left_speed = 0
                    current_right_speed = 0
                    if rf_controller:
                        firmware_id = robot_id + 1
                        rf_controller.set_motors(firmware_id, 0, 0)
                    print("🛑 Frenado completado")

            elif movement_active:
                time_elapsed = current_time - movement_start_time

                if time_elapsed < movement_duration:
                    send_motor_command(current_left_speed, current_right_speed)
                else:
                    stop_robot_smooth()
                    print("⏱️  Tiempo completado - Frenando...")

            time.sleep(0.001)

    except KeyboardInterrupt:
        log.info("⏹️  Proceso de control detenido por usuario")
    except Exception as e:
        log.error(f"❌ Error en proceso de control: {e}", exc_info=True)
    finally:
        stop_robot_immediate()
        if rf_controller:
            rf_controller.shutdown()
        cv2.destroyAllWindows()
        log.info("🔌 Proceso de control finalizado")
