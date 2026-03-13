"""Proceso de control para encontrar rango PWM útil.

Este proceso maneja la UI y control RF para determinar el PWM máximo
que la cámara puede detectar consistentemente.
"""

import logging
import sys
import time
from io import BytesIO
from pathlib import Path

import cv2
import matplotlib
import matplotlib.pyplot as plt
import numpy as np

matplotlib.use('Agg')  # Backend sin GUI

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

    # Estadísticas de SESIÓN (reseteadas al presionar ESPACIO)
    session_stats = {
        'active': False,
        'frames_analyzed': 0,
        'frames_detected': 0,
        'detection_rate': 0.0,
        'start_time': 0,
        'detection_timeline': []  # Lista de timestamps cuando se detectó
    }

    # Estadísticas GLOBALES (acumuladas desde inicio)
    global_stats = {
        'total_detections': 0,
        'last_detection_time': 0,
        'avg_fps': 0.0
    }

    def send_motor_command(left_speed, right_speed):
        """Envía comando de motor."""
        if not rf_controller:
            return
        firmware_id = robot_id + 1
        rf_controller.set_motors(firmware_id, left_speed, right_speed)

    def show_detection_graph(detections, total_duration):
        """Genera gráfico matplotlib y lo muestra con OpenCV.

        Usa backend Agg (sin GUI) + BytesIO + cv2.imshow para evitar conflictos.

        Args:
            detections: Lista de timestamps (en segundos desde inicio)
            total_duration: Duración total del movimiento (segundos)
        """
        # Crear figura con matplotlib
        fig, ax = plt.subplots(figsize=(10, 5))

        if total_duration <= 0 or not detections:
            # Sin detecciones
            ax.text(0.5, 0.5, 'Sin detecciones', ha='center', va='center',
                   fontsize=14, color='red')
            ax.set_xlim(0, 1)
            ax.set_ylim(0, 1)
            ax.axis('off')
        else:
            # Crear timeline continuo de detecciones
            # Cada punto representa un frame detectado
            detections_sorted = sorted(detections)

            # Marcar cada detección como punto en y=1
            ax.scatter(detections_sorted, [1]*len(detections_sorted),
                      c='green', s=50, alpha=0.6, marker='|', linewidths=2)

            # Calcular densidad de detección con ventana deslizante
            window_size = 0.1  # 100ms de ventana
            time_points = np.linspace(0, total_duration, 500)
            density = []

            for t in time_points:
                # Contar detecciones en ventana [t-window/2, t+window/2]
                count = sum(1 for d in detections_sorted
                           if t - window_size/2 <= d <= t + window_size/2)
                # Normalizar por tamaño de ventana para obtener tasa
                rate = count / window_size if window_size > 0 else 0
                density.append(rate)

            # Crear segundo eje para la curva de densidad
            ax2 = ax.twinx()
            ax2.plot(time_points, density, color='blue', linewidth=2, alpha=0.7, label='Densidad')
            ax2.fill_between(time_points, density, alpha=0.2, color='blue')
            ax2.set_ylabel('Detecciones/segundo', fontsize=10, color='blue')
            ax2.tick_params(axis='y', labelcolor='blue')

            # Análisis temporal (tercios)
            tercio = total_duration / 3
            inicio = sum(1 for t in detections if t < tercio)
            medio = sum(1 for t in detections if tercio <= t < 2*tercio)
            final = sum(1 for t in detections if t >= 2*tercio)

            # Líneas de división de tercios
            ax.axvline(tercio, color='red', linestyle='--', alpha=0.4, linewidth=1.5)
            ax.axvline(2*tercio, color='red', linestyle='--', alpha=0.4, linewidth=1.5)

            # Etiquetas de tercios (en la parte superior)
            ax.text(tercio/2, 0.95, f'INICIO: {inicio} det.',
                   ha='center', va='top', fontsize=9,
                   bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.7),
                   transform=ax.get_xaxis_transform())
            ax.text(tercio + tercio/2, 0.95, f'MEDIO: {medio} det.',
                   ha='center', va='top', fontsize=9,
                   bbox=dict(boxstyle='round', facecolor='lightblue', alpha=0.7),
                   transform=ax.get_xaxis_transform())
            ax.text(2*tercio + tercio/2, 0.95, f'FINAL: {final} det.',
                   ha='center', va='top', fontsize=9,
                   bbox=dict(boxstyle='round', facecolor='lightgreen', alpha=0.7),
                   transform=ax.get_xaxis_transform())

            # Configurar ejes
            ax.set_xlabel('Tiempo (segundos)', fontsize=10)
            ax.set_ylabel('Detección (puntos verdes)', fontsize=10, color='green')
            det_rate = len(detections)/total_duration
            title = (f'Timeline de Detecciones - {len(detections)} detecciones en '
                    f'{total_duration:.2f}s ({det_rate:.1f} det/s promedio)')
            ax.set_title(title, fontsize=11, fontweight='bold')
            ax.set_xlim(0, total_duration)
            ax.set_ylim(0, 1.2)
            ax.set_yticks([])  # Ocultar ticks del eje Y principal
            ax.grid(True, alpha=0.3, axis='x')
            ax.tick_params(axis='y', labelcolor='green')

        fig.tight_layout()

        # Guardar a buffer BytesIO (en memoria)
        buf = BytesIO()
        fig.savefig(buf, format='png', dpi=100)
        buf.seek(0)

        # Convertir a numpy array y decodificar con OpenCV
        img_arr = np.frombuffer(buf.getvalue(), dtype=np.uint8)
        img = cv2.imdecode(img_arr, cv2.IMREAD_COLOR)

        # Cerrar figura para liberar memoria
        plt.close(fig)
        buf.close()

        # Mostrar con OpenCV (que ya funciona bien)
        cv2.imshow('Timeline de Detecciones', img)
        cv2.waitKey(0)
        cv2.destroyWindow('Timeline de Detecciones')

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

                    # Mostrar resumen de sesión
                    if session_stats['active']:
                        print(f"\n⏹️  Movimiento completado ({time_elapsed:.2f}s)")
                        print("=" * 70)
                        print("📊 RESUMEN DE SESIÓN")
                        print("=" * 70)
                        print(f"   Frames analizados: {session_stats['frames_analyzed']}")
                        print(f"   Frames detectados: {session_stats['frames_detected']}")
                        print(f"   Tasa detección: {session_stats['detection_rate']*100:.1f}%")
                        print(f"   FPS promedio: {global_stats['avg_fps']:.1f}")
                        print("=" * 70 + "\n")

                        # Generar y mostrar gráfico de timeline
                        if len(session_stats['detection_timeline']) > 0:
                            show_detection_graph(
                                session_stats['detection_timeline'],
                                time_elapsed
                            )
                            print("📊 Gráfico de timeline generado - cierra la ventana para continuar")
                        else:
                            print("⚠️  Sin detecciones para graficar\n")

                        # Marcar sesión como inactiva
                        session_stats['active'] = False
                    else:
                        print(f"⏹️  Movimiento completado ({time_elapsed:.2f}s)")

            # ===== RECIBIR DATOS DE PERCEPCIÓN =====
            perception_data = None
            robot_detected = False
            robot_data = None
            perception_stats = None

            # Recibir datos de percepción (non-blocking)
            if robot_positions_pipe.poll():
                perception_data = robot_positions_pipe.recv()
                robot_detected = perception_data.get('robot_detected', False)
                robot_data = perception_data.get('robot_data', None)
                perception_stats = perception_data.get('stats', {})

                # Actualizar estadísticas globales
                global_stats['avg_fps'] = perception_stats.get('fps', 0.0)
                if robot_detected:
                    global_stats['total_detections'] += 1
                    global_stats['last_detection_time'] = current_time

                # Actualizar estadísticas de sesión (si hay movimiento activo)
                if session_stats['active']:
                    session_stats['frames_analyzed'] = perception_stats.get('frames_analyzed', 0)
                    session_stats['frames_detected'] = perception_stats.get('frames_detected', 0)
                    session_stats['detection_rate'] = perception_stats.get('detection_rate', 0.0)

                    # Guardar timestamp de detección para timeline
                    if robot_detected:
                        elapsed = current_time - session_stats['start_time']
                        session_stats['detection_timeline'].append(elapsed)

            # ===== CREAR FRAME DE VISUALIZACIÓN =====
            # Crear frame simple si no hay frame_pipe (modo ultra-rápido)
            if frame_pipe is None:
                frame_height = 480
                frame_width = 640
                frame_display = np.zeros((frame_height, frame_width, 3), dtype=np.uint8)

                # Dibujar robot si detectado (solo un rectángulo simple)
                if robot_detected and robot_data:
                    x = robot_data['x']
                    y = robot_data['y']
                    angle_rad = np.radians(robot_data['angle'])

                    # Rectángulo simple del robot
                    size = 40
                    color_robot = (0, 255, 0) if movement_active else (100, 255, 100)
                    cv2.rectangle(frame_display, (x-size//2, y-size//2),
                                (x+size//2, y+size//2), color_robot, 2)

                    # Línea de orientación
                    end_x = int(x + 50 * np.cos(angle_rad))
                    end_y = int(y + 50 * np.sin(angle_rad))
                    cv2.line(frame_display, (x, y), (end_x, end_y), color_robot, 2)

                    # ID del robot
                    cv2.putText(frame_display, f"Robot {robot_id}", (x+15, y-15),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.5, color_robot, 1)
                else:
                    # No detectado - mostrar mensaje
                    cv2.putText(frame_display, "Robot no detectado",
                               (frame_width//2 - 100, frame_height//2),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            else:
                # Modo estándar: recibir frame del proceso de percepción
                if frame_pipe.poll():
                    frame_display = frame_pipe.recv()
                else:
                    # Si no hay frame, esperar
                    time.sleep(0.001)
                    continue

            # ===== INFO DEL ROBOT =====
            if robot_detected and robot_data:
                robot_info = (
                    f"Robot {robot_id}: DETECTADO | "
                    f"Pos: ({robot_data['x']:.0f}, {robot_data['y']:.0f}) | "
                    f"Ángulo: {robot_data['angle']:.1f}°"
                )
                color = (0, 255, 0)  # Verde
            else:
                robot_info = f"Robot {robot_id}: NO DETECTADO"
                color = (0, 0, 255)  # Rojo

            # ===== PANEL DE INFORMACIÓN =====
            panel_height = 280  # Aumentado de 220 para nuevas estadísticas
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

            # FPS de percepción (NUEVO)
            fps_text = f"FPS Percepcion: {global_stats['avg_fps']:.1f} (objetivo: 28-40)"
            fps_color = (0, 255, 0) if global_stats['avg_fps'] >= 28 else (0, 165, 255)
            cv2.putText(panel, fps_text, (10, 190),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, fps_color, 2)

            # Estadísticas de sesión (NUEVO)
            if session_stats['active'] and session_stats['frames_analyzed'] > 0:
                session_text = (
                    f"SESION ACTUAL: {session_stats['frames_detected']}/{session_stats['frames_analyzed']} frames "
                    f"({session_stats['detection_rate']*100:.1f}% deteccion)"
                )
                cv2.putText(panel, session_text, (10, 220),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
            else:
                cv2.putText(panel, "Presiona ESPACIO para iniciar sesion", (10, 220),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (150, 150, 150), 1)

            # Estadísticas globales y controles
            time_since_detection = (current_time - global_stats['last_detection_time']
                                  if global_stats['last_detection_time'] > 0 else 999)
            stats = (f"Total: {global_stats['total_detections']} det. | "
                    f"Ultima: {time_since_detection:.1f}s | g=Guardar")
            cv2.putText(panel, stats, (10, 250),
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

                # Resetear estadísticas de sesión
                session_stats['active'] = True
                session_stats['frames_analyzed'] = 0
                session_stats['frames_detected'] = 0
                session_stats['detection_rate'] = 0.0
                session_stats['start_time'] = time.time()
                session_stats['detection_timeline'] = []
                print("📊 Estadísticas de sesión reseteadas")

            elif key == 8:  # BACKSPACE - atrás
                movement_active = True
                movement_direction = -1
                movement_start_time = time.time()
                current_left_speed = -current_pwm
                current_right_speed = -current_pwm
                send_motor_command(current_left_speed, current_right_speed)
                print(f"◀️  Movimiento ATRAS | PWM: {current_pwm} | Duración: {movement_duration}s")

                # Resetear estadísticas de sesión
                session_stats['active'] = True
                session_stats['frames_analyzed'] = 0
                session_stats['frames_detected'] = 0
                session_stats['detection_rate'] = 0.0
                session_stats['start_time'] = time.time()
                session_stats['detection_timeline'] = []
                print("📊 Estadísticas de sesión reseteadas")

            elif key == ord('x'):  # X - Detener
                if movement_active:
                    send_motor_command(0, 0)
                    movement_active = False
                    print("🛑 Movimiento detenido manualmente")

                    # Finalizar sesión si estaba activa
                    if session_stats['active']:
                        elapsed = current_time - session_stats['start_time']
                        print(f"\n🛑 Sesión interrumpida ({elapsed:.2f}s)")
                        print("=" * 70)
                        print("📊 RESUMEN DE SESIÓN (interrumpida)")
                        print("=" * 70)
                        print(f"   Frames analizados: {session_stats['frames_analyzed']}")
                        print(f"   Frames detectados: {session_stats['frames_detected']}")
                        print(f"   Tasa detección: {session_stats['detection_rate']*100:.1f}%")
                        print("=" * 70 + "\n")

                        # Generar y mostrar gráfico de timeline
                        if len(session_stats['detection_timeline']) > 0:
                            show_detection_graph(
                                session_stats['detection_timeline'],
                                elapsed
                            )
                            print("📊 Gráfico de timeline generado - cierra la ventana para continuar")
                        else:
                            print("⚠️  Sin detecciones para graficar\n")

                        session_stats['active'] = False

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
                    print("⚠️  Los puntos de calibración fueron regenerados con valores neutros")
                    print(f"   Ejecuta 'calibrate_robot_bias.py --robot-id {robot_id}' para calibrar bias\n")

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
        print(f"Total de detecciones: {global_stats['total_detections']}")
        print(f"Rango temporal: [{pwm_min_temp}, {pwm_max_temp}]")

        # Verificar si el rango fue guardado
        current_saved_min, current_saved_max = calibration.get_pwm_range(robot_id)
        if (current_saved_min, current_saved_max) == (pwm_min_temp, pwm_max_temp):
            print("✅ Rango GUARDADO en JSON")
        else:
            print("⚠️  Rango NO guardado (presiona 'g' para guardar)")

        print("\n💡 Proceso sugerido:")
        print("   1. Encuentra PWM_min: PWM más bajo donde el robot se mueve consistentemente")
        print("   2. Encuentra PWM_max: PWM más alto donde la cámara detecta al robot")
        print("   3. Ajusta rango con n/m (min) y ,/. (max)")
        print("   4. Presiona 'g' para guardar")
        print("   5. Calibra bias: calibrate_robot_bias.py --robot-id " + str(robot_id))
        print("=" * 70 + "\n")
