"""Proceso de visualización para calibración de bias (Arquitectura de 3 procesos).

Este proceso maneja TODA la interfaz de usuario:
- Lee frames desde shared memory
- Recibe metadata de percepción y estado de control
- Dibuja overlays (robot detectado, trayectoria)
- Muestra panel de información con bias actual, drift, controles
- Captura teclas y envía comandos al proceso de control

Diseñado para NO bloquear el proceso de control.
FPS objetivo: 28-40
"""

import logging
import sys
import time
from multiprocessing import shared_memory
from pathlib import Path

import cv2
import numpy as np

ROOT_DIR = Path(__file__).parent.parent.parent
sys.path.insert(0, str(ROOT_DIR / "src"))

from robot_soccer.config import CAMERA_PERSPECTIVE_HEIGHT, CAMERA_PERSPECTIVE_WIDTH

log = logging.getLogger(__name__)


def visualization_loop_bias(frame_pipe, control_state_pipe, keyboard_pipe,
                            shm_name: str = None, frame_counter=None):
    """Bucle principal del proceso de visualización para calibración de bias.

    Args:
        frame_pipe: Pipe para recibir metadata desde percepción
        control_state_pipe: Pipe para recibir estado desde control (bias, stats)
        keyboard_pipe: Pipe para enviar comandos de teclado a control
        shm_name: Nombre de la shared memory para leer frames
        frame_counter: multiprocessing.Value('i') contador atómico de frames

    Recibe por frame_pipe (metadata desde Percepción):
        {
            'robot_detected': bool,
            'robot_data': {'x': int, 'y': int, 'angulo': float} or None,
            'stats': {'fps': float, 'detection_rate': float, ...},
            'timestamp': float
        }

    Recibe por control_state_pipe (desde Control):
        {
            'current_bias': float,
            'test_pwm': int,
            'moving': bool,
            'trail': [(x, y), ...],
            'robot_detected': bool,
            'robot_data': {'x': int, 'y': int, 'angulo': float},
            'saved': bool,
            'robot_id': int,
            'timestamp': float
        }

    Envía por keyboard_pipe (a Control):
        {
            'command': str,  # 'start_movement', 'stop_movement', 'adjust_bias', 'adjust_pwm', 'save', 'clear_trail', 'exit'
            'param': str or None,  # 'bias', 'pwm'
            'delta': float or int or None,
            'timestamp': float
        }
    """
    log.info("🖥️  Proceso de visualización bias iniciado (3 procesos)")
    log.info("   Responsabilidades:")
    log.info("     ✓ Leer frames de shared memory")
    log.info("     ✓ Recibir metadata de percepción (pipe)")
    log.info("     ✓ Recibir estado de control (bias, trail)")
    log.info("     ✓ Mostrar UI con cv2.imshow()")
    log.info("     ✓ Capturar teclas y enviar comandos a control")

    # Conectar a shared memory para leer frames
    frame_shape = (CAMERA_PERSPECTIVE_HEIGHT, CAMERA_PERSPECTIVE_WIDTH, 3)
    try:
        shm = shared_memory.SharedMemory(name=shm_name)
        shared_array = np.ndarray(frame_shape, dtype=np.uint8, buffer=shm.buf)
        log.info(f"📦 Conectado a shared memory: {shm_name} ({frame_shape})")
    except Exception as e:
        log.error(f"❌ Error conectando a shared memory: {e}")
        log.error("   Asegúrate de que el proceso de percepción esté corriendo")
        log.error("   shm_name: %s", shm_name)
        return

    last_frame_counter = 0
    window_name = 'Calibración de Bias'

    # Crear ventana ANTES del loop principal
    cv2.namedWindow(window_name)
    log.info("✅ Ventana de visualización creada")

    # Estado local (última información recibida)
    last_frame = None
    last_robot_detected = False
    last_robot_data = None
    last_perception_stats = {}

    last_bias_state = {
        'current_bias': 0.0,
        'test_pwm': 30,
        'moving': False,
        'trail': [],
        'robot_detected': False,
        'robot_data': None,
        'saved': False,
        'robot_id': 0
    }

    # Configuración de ventana
    panel_height = 200
    max_trail_display = 500  # Máximo puntos a dibujar

    log.info("✅ Visualización iniciada - Esperando datos...")

    try:
        while True:
            current_time = time.time()
            new_frame_available = False

            # ===== RECIBIR METADATA DE PERCEPCIÓN =====
            if frame_pipe.poll():
                try:
                    perception_data = frame_pipe.recv()
                    last_robot_detected = perception_data.get('robot_detected', False)
                    last_robot_data = perception_data.get('robot_data', None)
                    last_perception_stats = perception_data.get('stats', {})
                    new_frame_available = True
                except Exception as e:
                    log.warning(f"⚠️  Error recibiendo datos de percepción: {e}")

            # ===== RECIBIR ESTADO DE CONTROL =====
            if control_state_pipe.poll():
                try:
                    control_data = control_state_pipe.recv()
                    last_bias_state.update(control_data)
                except Exception as e:
                    log.warning(f"⚠️  Error recibiendo estado de control: {e}")

            # ===== LEER FRAME DE SHARED MEMORY =====
            if frame_counter is not None:
                with frame_counter.get_lock():
                    current_counter = frame_counter.value

                if current_counter != last_frame_counter:
                    try:
                        frame_shape_check = shared_array.shape
                        if frame_shape_check == (CAMERA_PERSPECTIVE_HEIGHT, CAMERA_PERSPECTIVE_WIDTH, 3):
                            last_frame = np.copy(shared_array)
                            last_frame_counter = current_counter
                            new_frame_available = True
                        else:
                            log.warning(f"⚠️  Tamaño de frame incorrecto: {frame_shape_check}")
                    except Exception as e:
                        log.warning(f"⚠️  Error leyendo frame de shared memory: {e}")

            # ===== CREAR FRAME DE VISUALIZACIÓN =====
            if last_frame is not None:
                frame_display = last_frame.copy()
            else:
                frame_display = np.zeros((480, 640, 3), dtype=np.uint8)

            # ===== DIBUJAR TRAYECTORIA (TRAIL) =====
            trail = last_bias_state.get('trail', [])
            if len(trail) > 1:
                for i in range(1, min(len(trail), max_trail_display)):
                    t = i / min(len(trail), max_trail_display)
                    color = (0, int(255 * t), int(255 * (1 - t)))
                    cv2.line(frame_display, trail[i - 1], trail[i], color, 2)

            # ===== DIBUJAR ROBOT =====
            robot_detected = last_bias_state.get('robot_detected', False) or last_robot_detected
            robot_data = last_bias_state.get('robot_data') or last_robot_data

            if robot_detected and robot_data:
                x = robot_data.get('x', 0)
                y = robot_data.get('y', 0)
                angle = robot_data.get('angulo')
                if angle is not None:
                    angle_rad = np.radians(angle)
                else:
                    angle_rad = 0

                # Círculo del robot
                color = (0, 255, 0) if last_bias_state.get('moving', False) else (100, 255, 100)
                cv2.circle(frame_display, (int(x), int(y)), 8, color, -1)

                # Línea de orientación
                end_x = int(x + 30 * np.cos(angle_rad))
                end_y = int(y + 30 * np.sin(angle_rad))
                cv2.line(frame_display, (int(x), int(y)), (end_x, end_y), color, 2)
            elif not robot_detected:
                cv2.putText(frame_display, "Robot no detectado", (200, 240),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)

            # ===== CREAR PANEL DE INFORMACIÓN =====
            panel = np.zeros((panel_height, frame_display.shape[1], 3), dtype=np.uint8)

            # Estado movimiento
            if last_bias_state.get('moving', False):
                current_bias = last_bias_state.get('current_bias', 0.0)
                test_pwm = last_bias_state.get('test_pwm', 30)
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
            current_bias = last_bias_state.get('current_bias', 0.0)
            bias_color = (0, 255, 0) if abs(current_bias) < 0.001 else (255, 255, 255)
            cv2.putText(panel, f"Bias: {current_bias:+.4f}", (10, 65),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, bias_color, 2)

            # Efecto del bias
            bias_pwm_val = current_bias * 127
            cv2.putText(panel, f"  Efecto: L{bias_pwm_val:+.1f} PWM, R{-bias_pwm_val:+.1f} PWM",
                        (280, 65), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (180, 180, 180), 1)

            # Indicador drift
            if len(trail) >= 20:
                x_start = trail[0][0]
                x_end = trail[-1][0]
                y_travel = abs(trail[-1][1] - trail[0][1])
                if y_travel > 30:
                    x_drift = x_end - x_start
                    if abs(x_drift) > 5:
                        direction = "DERECHA" if x_drift > 0 else "IZQUIERDA"
                        drift_color = (0, 165, 255)
                        cv2.putText(panel, f"Drift: {x_drift:+.0f}px hacia {direction}",
                                    (10, 95), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                                    drift_color, 2)
                    else:
                        cv2.putText(panel, "Drift: RECTO", (10, 95),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

            # Controles
            cv2.putText(panel, "ESPACIO=mover  x=parar  c=limpiar trail", (10, 125),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (150, 150, 150), 1)
            cv2.putText(panel,
                        "LEFT/RIGHT: bias +/-0.002 | a/d: +/-0.01 | UP/DOWN: PWM +/-1",
                        (10, 150), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (150, 150, 150), 1)
            cv2.putText(panel, "g=guardar  ESC=salir", (10, 175),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (150, 150, 150), 1)

            # Estado guardado
            if last_bias_state.get('saved', False):
                cv2.putText(panel, "GUARDADO", (540, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

            # Combinar panel y frame
            combined = np.vstack([panel, frame_display])
            cv2.imshow(window_name, combined)

            # ===== CAPTURAR TECLAS Y ENVIAR COMANDOS =====
            key = cv2.waitKey(1) & 0xFF

            if key == 27:  # ESC
                log.info("ESC presionado - Enviando comando de salida...")
                keyboard_pipe.send({
                    'command': 'exit',
                    'timestamp': time.time()
                })
                break

            # Construir comando según la tecla
            command_data = None

            if key == ord(' '):  # ESPACIO - mover/parar toggle
                command_data = {
                    'command': 'toggle_movement',
                    'timestamp': time.time()
                }
            elif key == ord('x'):  # X - detener
                command_data = {
                    'command': 'stop_movement',
                    'timestamp': time.time()
                }
            elif key == ord('c'):  # Limpiar trail
                command_data = {
                    'command': 'clear_trail',
                    'timestamp': time.time()
                }
            elif key == 81:  # LEFT - bias -0.002
                command_data = {
                    'command': 'adjust_bias',
                    'param': 'bias',
                    'delta': -0.002,
                    'timestamp': time.time()
                }
            elif key == 83:  # RIGHT - bias +0.002
                command_data = {
                    'command': 'adjust_bias',
                    'param': 'bias',
                    'delta': 0.002,
                    'timestamp': time.time()
                }
            elif key == ord('a'):  # bias -0.01
                command_data = {
                    'command': 'adjust_bias',
                    'param': 'bias',
                    'delta': -0.01,
                    'timestamp': time.time()
                }
            elif key == ord('d'):  # bias +0.01
                command_data = {
                    'command': 'adjust_bias',
                    'param': 'bias',
                    'delta': 0.01,
                    'timestamp': time.time()
                }
            elif key == 82:  # UP - PWM +1
                command_data = {
                    'command': 'adjust_pwm',
                    'param': 'pwm',
                    'delta': 1,
                    'timestamp': time.time()
                }
            elif key == 84:  # DOWN - PWM -1
                command_data = {
                    'command': 'adjust_pwm',
                    'param': 'pwm',
                    'delta': -1,
                    'timestamp': time.time()
                }
            elif key == ord('g'):  # Guardar
                command_data = {
                    'command': 'save_bias',
                    'timestamp': time.time()
                }

            # Enviar comando si se presionó alguna tecla relevante
            if command_data:
                try:
                    keyboard_pipe.send(command_data)
                except Exception as e:
                    log.warning(f"⚠️  Error enviando comando: {e}")

            time.sleep(0.001)

    except KeyboardInterrupt:
        log.info("⏹️  Visualización detenida por usuario")
    except Exception as e:
        log.error(f"❌ Error en visualización: %s", e, exc_info=True)
    finally:
        cv2.destroyAllWindows()
        shm.close()
        log.info("🔌 Visualización cerrada")
