"""Proceso de visualización para calibración PID (Arquitectura de 3 procesos).

Este proceso maneja TODA la interfaz de usuario:
- Recibe frames procesados desde percepción
- Recibe estado del controlador PID desde control
- Dibuja overlays (robot, waypoint, orientación)
- Muestra panel de información con parámetros PID
- Captura teclas y mouse
- Envía comandos al proceso de control

Diseñado para NO bloquear el proceso de control PID crítico.
FPS objetivo: 28-40 (limitado por percepción y cv2.imshow)
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

log = logging.getLogger(__name__)


def visualization_loop_pid(frame_pipe, control_state_pipe, keyboard_pipe):
    """Bucle principal del proceso de visualización para calibración PID.

    Args:
        frame_pipe: Pipe para recibir frames procesados desde percepción
        control_state_pipe: Pipe para recibir estado desde control (PID params, waypoint, etc.)
        keyboard_pipe: Pipe bidireccional para enviar comandos de teclado/mouse a control

    Recibe por frame_pipe (desde Percepción):
        {
            'frame': np.ndarray (BGR, con transformación de perspectiva),
            'robot_detected': bool,
            'robot_data': {'x': int, 'y': int, 'angulo': float} or None,
            'stats': {'fps': float, 'detection_rate': float, ...},
            'timestamp': float
        }

    Recibe por control_state_pipe (desde Control):
        {
            'pid_params': {
                'kp_pos': float, 'ki_pos': float, 'kd_pos': float,
                'kp_angle': float, 'ki_angle': float, 'kd_angle': float
            },
            'target_waypoint': [x, y] or None,
            'movement_active': bool,
            'robot_id': int,
            'timestamp': float
        }

    Envía por keyboard_pipe (a Control):
        {
            'command': str,  # 'adjust_pid', 'set_waypoint', 'toggle_movement', 'exit', etc.
            'param': str or None,  # 'kp_pos', 'ki_angle', etc.
            'delta': float or None,  # ±0.001, ±0.01, etc.
            'waypoint': [x, y] or None,
            'timestamp': float
        }
    """
    log.info("🖥️  Proceso de visualización iniciado (PID - 3 procesos)")
    log.info("   Responsabilidades:")
    log.info("     ✓ Recibir frames de percepción")
    log.info("     ✓ Recibir estado de control")
    log.info("     ✓ Mostrar UI con cv2.imshow()")
    log.info("     ✓ Capturar teclas y mouse")
    log.info("     ✓ Enviar comandos a control")
    log.info("     → FPS: 28-40 (limitado por percepción)")

    # Estado local (última información recibida)
    last_frame = None
    last_robot_detected = False
    last_robot_data = None
    last_perception_stats = {}

    last_pid_params = {
        'kp_pos': 0.0, 'ki_pos': 0.0, 'kd_pos': 0.0,
        'kp_angle': 0.0, 'ki_angle': 0.0, 'kd_angle': 0.0
    }
    last_target_waypoint = None
    last_movement_active = False
    last_robot_id = 0
    last_robot_predicted = False

    # Configuración de ventana
    window_name = 'Calibracion PID (3 Procesos)'
    panel_height = 350

    # Callback de mouse para establecer waypoints
    def mouse_callback(event, x, y, flags, param):
        """Callback de mouse para establecer waypoint con click."""
        if event == cv2.EVENT_LBUTTONDOWN:
            # Ajustar coordenadas (restar altura del panel)
            if y > panel_height:
                actual_y = y - panel_height
                # Enviar comando al proceso de control
                try:
                    if keyboard_pipe.poll():
                        _ = keyboard_pipe.recv()  # Vaciar pipe
                    keyboard_pipe.send({
                        'command': 'set_waypoint',
                        'waypoint': [x, actual_y],
                        'timestamp': time.time()
                    })
                    log.info(f"🎯 Waypoint establecido: ({x}, {actual_y})")
                except Exception as e:
                    log.warning(f"⚠️  Error enviando waypoint: {e}")

    cv2.namedWindow(window_name)
    cv2.setMouseCallback(window_name, mouse_callback)

    log.info("✅ Ventana de visualización creada")

    try:
        while True:
            # ===== RECIBIR FRAMES DESDE PERCEPCIÓN =====
            if frame_pipe.poll():
                try:
                    perception_data = frame_pipe.recv()
                    last_frame = perception_data.get('frame')
                    last_robot_detected = perception_data.get('robot_detected', False)
                    last_robot_data = perception_data.get('robot_data')
                    last_perception_stats = perception_data.get('stats', {})
                except Exception as e:
                    log.warning(f"⚠️  Error recibiendo frame: {e}")

            # ===== RECIBIR ESTADO DESDE CONTROL =====
            if control_state_pipe.poll():
                try:
                    control_data = control_state_pipe.recv()
                    last_pid_params = control_data.get('pid_params', last_pid_params)
                    last_target_waypoint = control_data.get('target_waypoint')
                    last_movement_active = control_data.get('movement_active', False)
                    last_robot_id = control_data.get('robot_id', 0)
                    last_robot_predicted = control_data.get('robot_predicted', False)
                except Exception as e:
                    log.warning(f"⚠️  Error recibiendo estado de control: {e}")

            # ===== GENERAR FRAME DE VISUALIZACIÓN =====
            if last_frame is not None:
                frame_display = last_frame.copy()
                frame_height, frame_width = frame_display.shape[:2]

                # Dibujar robot si detectado
                if last_robot_detected and last_robot_data:
                    x = last_robot_data['x']
                    y = last_robot_data['y']
                    angle_rad = np.radians(last_robot_data['angulo'])

                    # Rectángulo del robot (40x40)
                    size = 40
                    color_robot = (0, 255, 0) if last_movement_active else (100, 255, 100)
                    cv2.rectangle(frame_display,
                                 (x - size // 2, y - size // 2),
                                 (x + size // 2, y + size // 2),
                                 color_robot, 2)

                    # Línea de orientación
                    end_x = int(x + 50 * np.cos(angle_rad))
                    end_y = int(y + 50 * np.sin(angle_rad))
                    cv2.line(frame_display, (x, y), (end_x, end_y), color_robot, 2)

                    # ID del robot
                    cv2.putText(frame_display, f"Robot {last_robot_id}",
                               (x + 15, y - 15),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.5, color_robot, 1)

                # Dibujar waypoint objetivo si existe
                if last_target_waypoint:
                    cv2.circle(frame_display, tuple(last_target_waypoint), 15, (0, 255, 255), 2)
                    cv2.circle(frame_display, tuple(last_target_waypoint), 3, (0, 255, 255), -1)

                    # Línea desde robot a waypoint (si robot detectado)
                    if last_robot_detected and last_robot_data:
                        cv2.line(frame_display,
                                (last_robot_data['x'], last_robot_data['y']),
                                tuple(last_target_waypoint),
                                (0, 255, 255), 1, cv2.LINE_AA)

                # ===== GENERAR PANEL DE INFORMACIÓN =====
                panel = np.zeros((panel_height, frame_width, 3), dtype=np.uint8)

                # Línea 1: Estado del robot
                robot_status = f"Robot {last_robot_id}: "
                if last_robot_detected and last_robot_data:
                    robot_status += f"DETECTADO | Pos: ({last_robot_data['x']}, {last_robot_data['y']}) | Angulo: {last_robot_data['angulo']:.1f}°"
                    status_color = (0, 255, 0)
                elif last_robot_predicted:
                    robot_status += "PREDICCION LINEAL"
                    status_color = (0, 165, 255)  # Naranja
                else:
                    robot_status += "NO DETECTADO"
                    status_color = (0, 0, 255)
                cv2.putText(panel, robot_status, (10, 30),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, status_color, 2)

                # Línea 2: Estado de movimiento
                if last_movement_active and last_target_waypoint:
                    movement_text = f"▶ MOVIENDO a waypoint ({last_target_waypoint[0]}, {last_target_waypoint[1]})"
                    movement_color = (0, 255, 0)
                else:
                    movement_text = "⏸ DETENIDO (Click en frame para establecer waypoint)"
                    movement_color = (150, 150, 150)
                cv2.putText(panel, movement_text, (10, 60),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, movement_color, 2)

                # Línea 3-8: Parámetros PID (3 de posición + 3 de ángulo)
                y_offset = 90
                pid_texts = [
                    f"PID Posicion KP: {last_pid_params['kp_pos']:.4f}  (1/q: ±0.001)",
                    f"PID Posicion KI: {last_pid_params['ki_pos']:.5f}  (2/w: ±0.0001)",
                    f"PID Posicion KD: {last_pid_params['kd_pos']:.3f}  (3/e: ±0.01)",
                    f"PID Angulo KP:   {last_pid_params['kp_angle']:.3f}  (4/r: ±0.01)",
                    f"PID Angulo KI:   {last_pid_params['ki_angle']:.4f}  (5/t: ±0.001)",
                    f"PID Angulo KD:   {last_pid_params['kd_angle']:.3f}  (6/y: ±0.01)",
                ]

                for i, text in enumerate(pid_texts):
                    cv2.putText(panel, text, (10, y_offset + i * 30),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.55, (200, 200, 255), 1)

                # Línea 9: FPS de percepción
                fps_value = last_perception_stats.get('fps', 0.0)
                fps_text = f"FPS Percepcion: {fps_value:.1f} (objetivo: 28-40)"
                fps_color = (0, 255, 0) if fps_value >= 28 else (0, 165, 255)
                cv2.putText(panel, fps_text, (10, 280),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, fps_color, 2)

                # Línea 10: Estadísticas de detección
                detection_rate = last_perception_stats.get('detection_rate', 0.0)
                det_text = f"Deteccion: {detection_rate * 100:.1f}% | Frames: {last_perception_stats.get('frames_analyzed', 0)}"
                cv2.putText(panel, det_text, (10, 310),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (100, 255, 255), 1)

                # Línea 11: Controles
                controls_text = "ESPACIO=Mover | g=Guardar PID | z=Reset PID | ESC=Salir"
                cv2.putText(panel, controls_text, (10, 340),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (150, 150, 150), 1)

                # Combinar panel + frame
                combined = np.vstack([panel, frame_display])

                # Mostrar ventana
                cv2.imshow(window_name, combined)

            else:
                # Sin frame disponible, mostrar mensaje de espera
                waiting_frame = np.zeros((480, 640, 3), dtype=np.uint8)
                cv2.putText(waiting_frame, "Esperando frames de percepcion...",
                           (100, 240),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
                cv2.imshow(window_name, waiting_frame)

            # ===== PROCESAR TECLAS =====
            key = cv2.waitKey(1) & 0xFF

            command = None
            param = None
            delta = None

            # Ajuste PID Posición KP (teclas 1/q)
            if key == ord('1'):
                command = 'adjust_pid'
                param = 'kp_pos'
                delta = 0.001
            elif key == ord('q'):
                command = 'adjust_pid'
                param = 'kp_pos'
                delta = -0.001

            # Ajuste PID Posición KI (teclas 2/w)
            elif key == ord('2'):
                command = 'adjust_pid'
                param = 'ki_pos'
                delta = 0.0001
            elif key == ord('w'):
                command = 'adjust_pid'
                param = 'ki_pos'
                delta = -0.0001

            # Ajuste PID Posición KD (teclas 3/e)
            elif key == ord('3'):
                command = 'adjust_pid'
                param = 'kd_pos'
                delta = 0.01
            elif key == ord('e'):
                command = 'adjust_pid'
                param = 'kd_pos'
                delta = -0.01

            # Ajuste PID Ángulo KP (teclas 4/r)
            elif key == ord('4'):
                command = 'adjust_pid'
                param = 'kp_angle'
                delta = 0.01
            elif key == ord('r'):
                command = 'adjust_pid'
                param = 'kp_angle'
                delta = -0.01

            # Ajuste PID Ángulo KI (teclas 5/t)
            elif key == ord('5'):
                command = 'adjust_pid'
                param = 'ki_angle'
                delta = 0.001
            elif key == ord('t'):
                command = 'adjust_pid'
                param = 'ki_angle'
                delta = -0.001

            # Ajuste PID Ángulo KD (teclas 6/y)
            elif key == ord('6'):
                command = 'adjust_pid'
                param = 'kd_angle'
                delta = 0.01
            elif key == ord('y'):
                command = 'adjust_pid'
                param = 'kd_angle'
                delta = -0.01

            # Guardar parámetros PID (tecla 'g')
            elif key == ord('g'):
                command = 'save_pid'

            # Reset PID a valores por defecto (tecla 'z')
            elif key == ord('z'):
                command = 'reset_pid'

            # Toggle movimiento (tecla ESPACIO)
            elif key == ord(' '):
                command = 'toggle_movement'

            # Salir (tecla ESC)
            elif key == 27:
                command = 'exit'
                log.info("🛑 ESC presionado - Enviando comando de salida")

            # Enviar comando al proceso de control
            if command:
                try:
                    if keyboard_pipe.poll():
                        _ = keyboard_pipe.recv()  # Vaciar pipe
                    keyboard_pipe.send({
                        'command': command,
                        'param': param,
                        'delta': delta,
                        'timestamp': time.time()
                    })
                except Exception as e:
                    log.warning(f"⚠️  Error enviando comando: {e}")

                # Salir del bucle si comando es 'exit'
                if command == 'exit':
                    break

            # Pausa pequeña para no saturar CPU
            time.sleep(0.001)

    except KeyboardInterrupt:
        log.info("⏹️  Proceso de visualización detenido por usuario")
    except Exception as e:
        log.error(f"❌ Error en proceso de visualización: {e}", exc_info=True)
    finally:
        cv2.destroyAllWindows()
        log.info("🔌 Ventana cerrada")
