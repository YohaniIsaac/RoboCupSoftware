"""Proceso de visualización para calibración PWM Range (Arquitectura de 3 procesos).

Este proceso maneja TODA la interfaz de usuario:
- Lee frames desde shared memory
- Recibe metadata de percepción y estado de control
- Dibuja overlays (robot detectado, información)
- Muestra panel de información con PWM actual, detección, FPS
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

# Agregar src al path
ROOT_DIR = Path(__file__).parent.parent.parent
sys.path.insert(0, str(ROOT_DIR / "src"))

from robot_soccer.config import CAMERA_PERSPECTIVE_HEIGHT, CAMERA_PERSPECTIVE_WIDTH

log = logging.getLogger(__name__)


def visualization_loop_pwm_range(frame_pipe, control_state_pipe, keyboard_pipe,
                                  shm_name: str = None, frame_counter=None):
    """Bucle principal del proceso de visualización para calibración PWM Range.

    Args:
        frame_pipe: Pipe para recibir metadata desde percepción
        control_state_pipe: Pipe para recibir estado desde control (PWM params, stats)
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
            'current_pwm': int,
            'movement_active': bool,
            'movement_direction': int,  # 1=adelante, -1=atrás
            'movement_duration': float,
            'pwm_min': int,
            'pwm_max': int,
            'session_stats': {
                'frames_analyzed': int,
                'frames_detected': int,
                'detection_rate': float
            },
            'total_detections': int,
            'robot_id': int,
            'last_speed_px_s': float or None,  # velocidad medida en última sesión
            'last_speed_n_samples': int,
            'last_speed_distance_px': float,
            'timestamp': float
        }

    Envía por keyboard_pipe (a Control):
        {
            'command': str,  # 'adjust_pwm', 'start_movement', 'stop_movement', 'set_range', 'save', 'exit'
            'param': str or None,  # 'pwm', 'pwm_min', 'pwm_max'
            'delta': int or None,  # ±1, ±5, etc.
            'value': int or float or None,
            'timestamp': float
        }
    """
    log.info("🖥️  Proceso de visualización PWM Range iniciado (3 procesos)")
    log.info("   Responsabilidades:")
    log.info("     ✓ Leer frames de shared memory")
    log.info("     ✓ Recibir metadata de percepción (pipe)")
    log.info("     ✓ Recibir estado de control")
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
    window_name = 'Calibracion PWM Range'

    # Crear ventana ANTES del loop principal
    cv2.namedWindow(window_name)
    log.info("✅ Ventana de visualización creada")

    # Estado local (última información recibida)
    last_frame = None
    last_robot_detected = False
    last_robot_data = None
    last_perception_stats = {}

    last_pwm_state = {
        'current_pwm': 30,
        'movement_active': False,
        'movement_direction': 1,
        'movement_duration': 1.0,
        'pwm_min': 20,
        'pwm_max': 80,
        'session_stats': {
            'frames_analyzed': 0,
            'frames_detected': 0,
            'detection_rate': 0.0
        },
        'total_detections': 0,
        'robot_id': 0,
        'last_speed_px_s': None,
        'last_speed_n_samples': 0,
        'last_speed_distance_px': 0.0
    }

    # Configuración de ventana
    panel_height = 325

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
                    last_pwm_state.update(control_data)
                except Exception as e:
                    log.warning(f"⚠️  Error recibiendo estado de control: {e}")

            # ===== LEER FRAME DE SHARED MEMORY =====
            if frame_counter is not None:
                with frame_counter.get_lock():
                    current_counter = frame_counter.value

                if current_counter != last_frame_counter:
                    # Validar tamaño del frame antes de copiar
                    try:
                        frame_shape = shared_array.shape
                        if frame_shape == (CAMERA_PERSPECTIVE_HEIGHT, CAMERA_PERSPECTIVE_WIDTH, 3):
                            last_frame = np.copy(shared_array)
                            last_frame_counter = current_counter
                            new_frame_available = True
                        else:
                            log.warning(f"⚠️  Tamaño de frame incorrecto: {frame_shape}, esperado: ({CAMERA_PERSPECTIVE_HEIGHT}, {CAMERA_PERSPECTIVE_WIDTH}, 3)")
                    except Exception as e:
                        log.warning(f"⚠️  Error leyendo frame de shared memory: {e}")

            # ===== CREAR FRAME DE VISUALIZACIÓN =====
            if last_frame is not None:
                frame_display = last_frame.copy()
            else:
                frame_height = 480
                frame_width = 640
                frame_display = np.zeros((frame_height, frame_width, 3), dtype=np.uint8)

            # Dibujar robot si detectado
            if last_robot_detected and last_robot_data:
                x = last_robot_data.get('x', 0)
                y = last_robot_data.get('y', 0)
                angle = last_robot_data.get('angulo', 0)
                angle_rad = np.radians(angle)

                # Rectángulo del robot
                size = 40
                color = (0, 255, 0) if last_pwm_state.get('movement_active', False) else (100, 255, 100)
                cv2.rectangle(frame_display, (x-size//2, y-size//2),
                             (x+size//2, y+size//2), color, 2)

                # Línea de orientación
                end_x = int(x + 50 * np.cos(angle_rad))
                end_y = int(y + 50 * np.sin(angle_rad))
                cv2.line(frame_display, (x, y), (end_x, end_y), color, 2)

                # ID del robot
                cv2.putText(frame_display, f"Robot {last_pwm_state.get('robot_id', 0)}", (x+15, y-15),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
            elif not last_robot_detected:
                # Mensaje de no detectado
                cv2.putText(frame_display, "Robot no detectado",
                           (frame_display.shape[1]//2 - 100, frame_display.shape[0]//2),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

            # ===== CREAR PANEL DE INFORMACIÓN =====
            panel = np.zeros((panel_height, frame_display.shape[1], 3), dtype=np.uint8)

            # Información del robot
            robot_id = last_pwm_state.get('robot_id', 0)
            if last_robot_detected:
                robot_info = f"Robot {robot_id}: DETECTADO"
                color = (0, 255, 0)
            else:
                robot_info = f"Robot {robot_id}: NO DETECTADO"
                color = (0, 0, 255)
            cv2.putText(panel, robot_info, (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)

            # Estado del movimiento
            if last_pwm_state.get('movement_active', False):
                direction_text = "ADELANTE" if last_pwm_state.get('movement_direction', 1) > 0 else "ATRAS"
                status = f"MOVIENDO {direction_text}"
                status_color = (0, 255, 255)
            else:
                status = "DETENIDO"
                status_color = (200, 200, 200)
            cv2.putText(panel, status, (10, 65),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, status_color, 2)

            # PWM actual
            current_pwm = last_pwm_state.get('current_pwm', 30)
            pwm_text = f"PWM actual: {current_pwm}  (↑/↓: ±5, w/s: ±1)"
            cv2.putText(panel, pwm_text, (10, 100),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

            # Duración
            duration = last_pwm_state.get('movement_duration', 1.0)
            dur_text = f"Duracion: {duration:.1f}s  (+/-: ±0.5s, [/]: ±0.1s)"
            cv2.putText(panel, dur_text, (10, 130),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

            # Rango PWM
            pwm_min = last_pwm_state.get('pwm_min', 20)
            pwm_max = last_pwm_state.get('pwm_max', 80)
            range_text = f"Rango: [{pwm_min}, {pwm_max}]  (n/m: PWM_min, ,/.: PWM_max)"
            cv2.putText(panel, range_text, (10, 160),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (100, 255, 100), 2)

            # FPS de percepción
            fps = last_perception_stats.get('fps', 0.0)
            fps_text = f"FPS: {fps:.1f} (objetivo: 28-40)"
            fps_color = (0, 255, 0) if fps >= 28 else (0, 165, 255)
            cv2.putText(panel, fps_text, (10, 190),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, fps_color, 2)

            # Estadísticas de sesión
            session_stats = last_pwm_state.get('session_stats', {})
            frames_analyzed = session_stats.get('frames_analyzed', 0)
            frames_detected = session_stats.get('frames_detected', 0)
            detection_rate = session_stats.get('detection_rate', 0.0)

            if frames_analyzed > 0:
                session_text = (
                    f"SESION: {frames_detected}/{frames_analyzed} frames "
                    f"({detection_rate*100:.1f}% deteccion)"
                )
                session_color = (0, 255, 255) if detection_rate >= 0.8 else (0, 165, 255)
            else:
                session_text = "Presiona ESPACIO para iniciar sesion"
                session_color = (150, 150, 150)
            cv2.putText(panel, session_text, (10, 220),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, session_color, 2)

            # Velocidad medida en la última sesión completada
            speed_val = last_pwm_state.get('last_speed_px_s', None)
            speed_n = last_pwm_state.get('last_speed_n_samples', 0)
            speed_dist = last_pwm_state.get('last_speed_distance_px', 0.0)
            if speed_val is not None:
                speed_text = f"Velocidad: {speed_val:.1f} px/s  (n={speed_n}, dist={speed_dist:.0f} px)"
                speed_color = (0, 255, 255)
            else:
                speed_text = "Velocidad: N/A (sin detecciones)"
                speed_color = (150, 150, 150)
            cv2.putText(panel, speed_text, (10, 250),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, speed_color, 2)

            # Total detecciones
            total_det = last_pwm_state.get('total_detections', 0)
            total_text = f"Total detecciones: {total_det}  |  g=Guardar rango  |  r=sugerencia"
            cv2.putText(panel, total_text, (10, 280),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (150, 150, 150), 1)

            # Controles
            controls_text = "ESPACIO=adelante | BACKSPACE=atras | x=detener | ESC=salir"
            cv2.putText(panel, controls_text, (10, 310),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (150, 150, 150), 1)

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

            if key == ord(' '):  # ESPACIO - adelante
                command_data = {
                    'command': 'start_movement',
                    'value': 1,  # adelante
                    'timestamp': time.time()
                }
            elif key == 8:  # BACKSPACE - atrás
                command_data = {
                    'command': 'start_movement',
                    'value': -1,  # atrás
                    'timestamp': time.time()
                }
            elif key == ord('x'):  # X - detener
                command_data = {
                    'command': 'stop_movement',
                    'timestamp': time.time()
                }
            elif key == 82:  # ↑
                command_data = {
                    'command': 'adjust_pwm',
                    'param': 'pwm',
                    'delta': 5,
                    'timestamp': time.time()
                }
            elif key == 84:  # ↓
                command_data = {
                    'command': 'adjust_pwm',
                    'param': 'pwm',
                    'delta': -5,
                    'timestamp': time.time()
                }
            elif key == ord('w'):
                command_data = {
                    'command': 'adjust_pwm',
                    'param': 'pwm',
                    'delta': 1,
                    'timestamp': time.time()
                }
            elif key == ord('s'):
                command_data = {
                    'command': 'adjust_pwm',
                    'param': 'pwm',
                    'delta': -1,
                    'timestamp': time.time()
                }
            elif key == ord('=') or key == ord('+'):
                command_data = {
                    'command': 'adjust_duration',
                    'delta': 0.5,
                    'timestamp': time.time()
                }
            elif key == ord('-') or key == ord('_'):
                command_data = {
                    'command': 'adjust_duration',
                    'delta': -0.5,
                    'timestamp': time.time()
                }
            elif key == ord(']'):
                command_data = {
                    'command': 'adjust_duration',
                    'delta': 0.1,
                    'timestamp': time.time()
                }
            elif key == ord('['):
                command_data = {
                    'command': 'adjust_duration',
                    'delta': -0.1,
                    'timestamp': time.time()
                }
            elif key == ord('n'):  # PWM_min -1
                command_data = {
                    'command': 'set_range',
                    'param': 'pwm_min',
                    'delta': -1,
                    'timestamp': time.time()
                }
            elif key == ord('m'):  # PWM_min +1
                command_data = {
                    'command': 'set_range',
                    'param': 'pwm_min',
                    'delta': 1,
                    'timestamp': time.time()
                }
            elif key == ord(','):  # PWM_max -1
                command_data = {
                    'command': 'set_range',
                    'param': 'pwm_max',
                    'delta': -1,
                    'timestamp': time.time()
                }
            elif key == ord('.'):  # PWM_max +1
                command_data = {
                    'command': 'set_range',
                    'param': 'pwm_max',
                    'delta': 1,
                    'timestamp': time.time()
                }
            elif key == ord('r'):  # Sugerencia basada en PWM actual
                command_data = {
                    'command': 'suggest_range',
                    'timestamp': time.time()
                }
            elif key == ord('g'):  # Guardar rango
                command_data = {
                    'command': 'save_range',
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
        log.error(f"❌ Error en visualización: {e}", exc_info=True)
    finally:
        cv2.destroyAllWindows()
        shm.close()
        log.info("🔌 Visualización cerrada")
