"""Proceso de control para calibración de bias (deriva en línea recta).

Mueve el robot hacia adelante a velocidad media y permite ajustar
el bias hasta que vaya recto. La visualización está en otro proceso.

Este proceso:
- Recibe datos de posición desde percepción
- Recibe comandos de teclado desde visualización
- Envía estado a visualización para mostrar
- Controla los motores via RF
"""

import logging
import sys
import time
from pathlib import Path

import numpy as np

ROOT_DIR = Path(__file__).parent.parent.parent
sys.path.insert(0, str(ROOT_DIR / "src"))

from robot_soccer.communication.rf_controller import RFController
from robot_soccer.controllers.robot_calibration_multipoint import RobotCalibrationMultipoint

log = logging.getLogger(__name__)


def control_loop_bias(robot_positions_pipe, control_state_pipe, keyboard_pipe,
                    control_to_perception_pipe, robot_id, serial_port):
    """Bucle principal para calibración de bias (separado de UI).

    Args:
        robot_positions_pipe: Pipe para recibir datos de posición desde percepción
        control_state_pipe: Pipe para enviar estado a visualización
        keyboard_pipe: Pipe para recibir comandos de teclado desde visualización
        control_to_perception_pipe: Pipe para enviar señales a percepción (reset stats)
        robot_id: ID del robot (0-3)
        serial_port: Puerto serial
    """
    log.info("Proceso de calibracion bias iniciado para Robot %d", robot_id)

    # Cargar calibración
    calibration = RobotCalibrationMultipoint()
    pwm_min, pwm_max = calibration.get_pwm_range(robot_id)
    pwm_mid = (pwm_min + pwm_max) // 2
    current_bias = calibration.get_bias(robot_id)
    test_pwm = pwm_mid

    log.info("Robot %d: rango [%d-%d], PWM prueba=%d, bias actual=%.4f",
             robot_id, pwm_min, pwm_max, test_pwm, current_bias)

    # RF controller (sin calibración - aplicamos bias manualmente para ver efecto)
    rf = None
    try:
        rf = RFController(port=serial_port, enable_calibration=False,
                          min_command_interval=0.005)
        if rf.initialize():
            log.info("RF conectado")
        else:
            log.warning("No se pudo conectar RF")
    except Exception as e:
        log.warning("Error RF: %s", e)

    # Estado
    moving = False
    saved = False
    max_trail = 500
    trail = []  # Lista de (x, y) para dibujar trayectoria

    def send_cmd(left, right):
        if rf:
            rf.set_motors(robot_id + 1, int(left), int(right))

    def stop():
        send_cmd(0, 0)

    def send_state_to_viz(current_time):
        """Envía estado actual a visualización con rate limiting (~25 Hz).

        IMPORTANTE: Sin rate limiting, el pipe se satura y pipe.send() BLOQUEA
        hasta que haya espacio, causando pausas de ~0.6-0.7s en el loop de control.
        """
        nonlocal last_state_send_time
        if current_time - last_state_send_time >= 0.04:  # 40ms = 25 Hz
            try:
                control_state_pipe.send({
                    'current_bias': current_bias,
                    'test_pwm': test_pwm,
                    'moving': moving,
                    'trail': trail[-50:] if len(trail) > 50 else trail.copy(),
                    'robot_detected': detected,
                    'robot_data': {'x': robot_x, 'y': robot_y, 'angulo': robot_angle} if detected else None,
                    'saved': saved,
                    'robot_id': robot_id,
                    'timestamp': current_time
                })
                last_state_send_time = current_time
            except Exception as e:
                log.warning(f"Error enviando estado a viz: {e}")

    # Flags de control
    exit_requested = False
    last_state_send_time = 0.0
    robot_x, robot_y = None, None
    robot_angle = None
    detected = False

    try:
        while not exit_requested:
            current_time = time.time()

            # ===== PROCESAR COMANDOS DE TECLADO (desde visualización) =====
            while keyboard_pipe.poll():
                try:
                    cmd = keyboard_pipe.recv()
                    command = cmd.get('command', '')
                    param = cmd.get('param')
                    delta = cmd.get('delta')

                    if command == 'exit':
                        exit_requested = True
                        log.info("Comando exit recibido")
                        stop()
                        break

                    elif command == 'toggle_movement':
                        if not moving:
                            moving = True
                            trail.clear()
                            saved = False
                            log.info(f"Moviendo: PWM={test_pwm}, bias={current_bias:+.4f}")
                        else:
                            moving = False
                            stop()
                            log.info("Detenido")

                    elif command == 'stop_movement':
                        moving = False
                        stop()
                        log.info("Movimiento detenido")

                    elif command == 'clear_trail':
                        trail.clear()
                        log.info("Trail limpiado")

                    elif command == 'adjust_bias':
                        if param == 'bias':
                            delta_val = delta if delta else 0
                            current_bias += delta_val
                            current_bias = max(-0.5, min(0.5, current_bias))
                            saved = False
                            log.info(f"Bias: {current_bias:+.4f}")

                    elif command == 'adjust_pwm':
                        if param == 'pwm':
                            delta_val = delta if delta else 0
                            test_pwm = max(pwm_min, min(pwm_max, test_pwm + delta_val))
                            log.info(f"PWM: {test_pwm}")

                    elif command == 'save_bias':
                        calibration.set_bias(robot_id, current_bias)
                        calibration.save()
                        saved = True
                        log.info(f"Bias {current_bias:+.4f} guardado para Robot {robot_id}")

                except Exception as e:
                    log.warning(f"Error procesando comando: {e}")

            if exit_requested:
                break

            # ===== ENVIAR COMANDOS DE MOTOR =====
            if moving:
                bias_pwm = current_bias * 127
                left = test_pwm + bias_pwm
                right = test_pwm - bias_pwm
                left = max(-127, min(127, int(left)))
                right = max(-127, min(127, int(right)))
                send_cmd(left, right)

            # ===== RECIBIR POSICIÓN DEL ROBOT =====
            robot_x, robot_y = None, None
            robot_angle = None
            detected = False
            while robot_positions_pipe.poll():
                data = robot_positions_pipe.recv()
                if data.get('robot_detected'):
                    rd = data['robot_data']
                    robot_x = rd.get('x')
                    robot_y = rd.get('y')
                    robot_angle = rd.get('angulo')
                    detected = True

            # ===== ACTUALIZAR TRAIL =====
            if moving and detected and robot_x is not None and robot_y is not None:
                trail.append((int(robot_x), int(robot_y)))
                if len(trail) > max_trail:
                    trail.pop(0)

            # ===== ENVIAR ESTADO A VISUALIZACIÓN (rate limited ~25 Hz) =====
            send_state_to_viz(current_time)

            time.sleep(0.001)

    except KeyboardInterrupt:
        log.info("Interrumpido por usuario")
    except Exception as e:
        log.error("Error: %s", e, exc_info=True)
    finally:
        stop()
        if rf:
            rf.shutdown()
        print(f"\nBias final: {current_bias:+.4f} {'(GUARDADO)' if saved else '(NO guardado)'}")
