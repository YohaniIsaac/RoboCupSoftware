"""Proceso de control para calibración de bias (deriva en línea recta).

Mueve el robot hacia adelante a velocidad media y permite ajustar
el bias hasta que vaya recto. Muestra la trayectoria en pantalla.
"""

import logging
import sys
import time
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

WINDOW_NAME = 'Calibracion de Bias'


def control_loop_bias(robot_positions_pipe, _frame_pipe, robot_id, serial_port):
    """Bucle principal para calibración de bias."""
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
    trail = []  # Lista de (x, y) para dibujar trayectoria
    max_trail = 500
    saved = False

    def send_cmd(left, right):
        if rf:
            rf.set_motors(robot_id + 1, int(left), int(right))

    def stop():
        send_cmd(0, 0)

    try:
        while True:
            now = time.time()

            # Enviar comandos continuos si está moviendo
            if moving:
                bias_pwm = current_bias * 127
                left = test_pwm + bias_pwm
                right = test_pwm - bias_pwm
                left = max(-127, min(127, int(left)))
                right = max(-127, min(127, int(right)))
                send_cmd(left, right)

            # Recibir posición del robot
            robot_x, robot_y = None, None
            robot_angle = None
            detected = False
            while robot_positions_pipe.poll():
                data = robot_positions_pipe.recv()
                if data.get('robot_detected'):
                    rd = data['robot_data']
                    robot_x, robot_y = rd['x'], rd['y']
                    robot_angle = rd['angle']
                    detected = True

            # Agregar al trail si se está moviendo y hay detección
            if moving and detected:
                trail.append((robot_x, robot_y))
                if len(trail) > max_trail:
                    trail.pop(0)

            # === DIBUJAR ===
            frame = np.zeros((480, 640, 3), dtype=np.uint8)

            # Trail (trayectoria)
            if len(trail) > 1:
                for i in range(1, len(trail)):
                    # Color gradiente: rojo viejo → verde reciente
                    t = i / len(trail)
                    color = (0, int(255 * t), int(255 * (1 - t)))
                    cv2.line(frame, trail[i - 1], trail[i], color, 2)

            # Robot actual
            if detected:
                cv2.circle(frame, (robot_x, robot_y), 8, (0, 255, 0), -1)
                angle_rad = np.radians(robot_angle)
                end_x = int(robot_x + 30 * np.cos(angle_rad))
                end_y = int(robot_y + 30 * np.sin(angle_rad))
                cv2.line(frame, (robot_x, robot_y), (end_x, end_y), (0, 255, 0), 2)
            else:
                cv2.putText(frame, "Robot no detectado", (200, 240),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)

            # === PANEL SUPERIOR ===
            panel = np.zeros((200, 640, 3), dtype=np.uint8)

            # Estado movimiento
            if moving:
                bias_pwm = current_bias * 127
                l_pwm = int(test_pwm + bias_pwm)
                r_pwm = int(test_pwm - bias_pwm)
                status = f"MOVIENDO: L={l_pwm} R={r_pwm} (PWM base={test_pwm})"
                cv2.putText(panel, status, (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 255), 2)
            else:
                cv2.putText(panel, "DETENIDO - ESPACIO para mover, ESC para salir",
                            (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (180, 180, 180), 1)

            # Bias actual
            bias_color = (0, 255, 0) if abs(current_bias) < 0.001 else (255, 255, 255)
            cv2.putText(panel, f"Bias: {current_bias:+.4f}", (10, 65),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, bias_color, 2)

            # Efecto del bias
            bias_pwm_val = current_bias * 127
            cv2.putText(panel, f"  Efecto: L{bias_pwm_val:+.1f} PWM, R{-bias_pwm_val:+.1f} PWM",
                        (280, 65), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (180, 180, 180), 1)

            # Controles
            cv2.putText(panel, "ESPACIO=mover  x=parar  c=limpiar trail", (10, 100),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (150, 150, 150), 1)
            cv2.putText(panel,
                        "LEFT/RIGHT: bias +/-0.002 | a/d: +/-0.01 | UP/DOWN: PWM +/-1",
                        (10, 125), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (150, 150, 150), 1)
            cv2.putText(panel, "g=guardar  ESC=salir", (10, 150),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (150, 150, 150), 1)

            # Indicador drift
            if len(trail) >= 20:
                # Comparar posición X del inicio vs final del trail
                x_start = trail[0][0]
                x_end = trail[-1][0]
                y_travel = abs(trail[-1][1] - trail[0][1])
                if y_travel > 30:  # Solo si hubo movimiento significativo
                    x_drift = x_end - x_start
                    if abs(x_drift) > 5:
                        direction = "DERECHA" if x_drift > 0 else "IZQUIERDA"
                        drift_color = (0, 165, 255)
                        cv2.putText(panel, f"Drift: {x_drift:+.0f}px hacia {direction}",
                                    (10, 180), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                                    drift_color, 2)
                    else:
                        cv2.putText(panel, "Drift: RECTO", (10, 180),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

            # Estado guardado
            if saved:
                cv2.putText(panel, "GUARDADO", (540, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

            combined = np.vstack([panel, frame])
            cv2.imshow(WINDOW_NAME, combined)

            # === TECLAS ===
            key = cv2.waitKey(1) & 0xFF

            if key == 27:  # ESC
                break

            elif key == ord(' '):  # ESPACIO - mover/parar toggle
                if not moving:
                    moving = True
                    trail.clear()
                    saved = False
                    print(f"Moviendo: PWM={test_pwm}, bias={current_bias:+.4f}")
                else:
                    moving = False
                    stop()
                    print("Detenido")

            elif key == ord('x'):  # Parar
                moving = False
                stop()

            elif key == ord('c'):  # Limpiar trail
                trail.clear()

            elif key == 81:  # LEFT - bias -0.002
                current_bias -= 0.002
                current_bias = max(-0.5, current_bias)
                saved = False
                print(f"Bias: {current_bias:+.4f}")

            elif key == 83:  # RIGHT - bias +0.002
                current_bias += 0.002
                current_bias = min(0.5, current_bias)
                saved = False
                print(f"Bias: {current_bias:+.4f}")

            elif key == ord('a'):  # bias -0.01
                current_bias -= 0.01
                current_bias = max(-0.5, current_bias)
                saved = False
                print(f"Bias: {current_bias:+.4f}")

            elif key == ord('d'):  # bias +0.01
                current_bias += 0.01
                current_bias = min(0.5, current_bias)
                saved = False
                print(f"Bias: {current_bias:+.4f}")

            elif key == 82:  # UP - PWM +1
                test_pwm = min(pwm_max, test_pwm + 1)
                print(f"PWM: {test_pwm}")

            elif key == 84:  # DOWN - PWM -1
                test_pwm = max(pwm_min, test_pwm - 1)
                print(f"PWM: {test_pwm}")

            elif key == ord('g'):  # Guardar
                calibration.set_bias(robot_id, current_bias)
                calibration.save()
                saved = True
                print(f"Bias {current_bias:+.4f} guardado para Robot {robot_id}")

            time.sleep(0.001)

    except KeyboardInterrupt:
        log.info("Interrumpido por usuario")
    except Exception as e:
        log.error("Error: %s", e, exc_info=True)
    finally:
        stop()
        if rf:
            rf.shutdown()
        cv2.destroyAllWindows()
        print(f"\nBias final: {current_bias:+.4f} {'(GUARDADO)' if saved else '(NO guardado)'}")
