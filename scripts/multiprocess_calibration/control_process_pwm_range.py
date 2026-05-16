"""Proceso de control para encontrar rango PWM útil.

Este proceso maneja la lógica de calibración PWM y comandos RF.
La UI y captura de teclado están separadas en visualization_process_pwm_range.

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

# Agregar src al path
ROOT_DIR = Path(__file__).parent.parent.parent
sys.path.insert(0, str(ROOT_DIR / "src"))

from robot_soccer.communication.rf_controller import RFController
from robot_soccer.controllers.robot_calibration_multipoint import RobotCalibrationMultipoint

log = logging.getLogger(__name__)


def compute_avg_speed(samples):
    """Calcula velocidad promedio en px/s a partir de muestras (timestamp, x, y).

    Aplica trim simétrico del 10% si hay >=10 muestras para mitigar outliers
    (saltos del tracker, frames con detección imprecisa).
    """
    if len(samples) < 2:
        return None, 0, 0.0
    velocities = []
    total_dist = 0.0
    for (t0, x0, y0), (t1, x1, y1) in zip(samples, samples[1:]):
        dt = t1 - t0
        if dt < 0.005:
            continue
        dist = ((x1 - x0) ** 2 + (y1 - y0) ** 2) ** 0.5
        total_dist += dist
        velocities.append(dist / dt)
    if not velocities:
        return None, 0, total_dist
    velocities.sort()
    n = len(velocities)
    if n >= 10:
        cut = max(1, n // 10)
        trimmed = velocities[cut:n - cut]
        if len(trimmed) < 3:
            trimmed = velocities
    else:
        trimmed = velocities
    avg = sum(trimmed) / len(trimmed)
    return avg, len(trimmed), total_dist


def control_loop_pwm_range(robot_positions_pipe, control_state_pipe, keyboard_pipe,
                          control_to_perception_pipe, robot_id, serial_port):
    """Bucle principal del proceso de control para PWM range.

    Args:
        robot_positions_pipe: Pipe para recibir datos de posición desde percepción
        control_state_pipe: Pipe para enviar estado a visualización
        keyboard_pipe: Pipe para recibir comandos de teclado desde visualización
        control_to_perception_pipe: Pipe para enviar señales a percepción (reset stats)
        robot_id: ID del robot a probar (0-3)
        serial_port: Puerto serial para comunicación RF
    """
    log.info(f"🎮 Proceso de control PWM Range iniciado para Robot ID {robot_id}")

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

    # Rango PWM temporal (para determinar antes de guardar)
    pwm_min_temp = pwm_min_saved
    pwm_max_temp = pwm_max_saved

    # Estadísticas de SESIÓN (reseteadas al iniciar movimiento)
    session_stats = {
        'frames_analyzed': 0,
        'frames_detected': 0,
        'detection_rate': 0.0
    }

    # Estadísticas GLOBALES
    global_stats = {
        'total_detections': 0,
        'last_detection_time': 0,
        'avg_fps': 0.0
    }

    # Buffer para medición de velocidad (durante el movimiento activo)
    position_samples = []  # lista de (timestamp, x, y)
    last_speed_px_s = None
    last_speed_n_samples = 0
    last_speed_distance_px = 0.0

    def send_motor_command(left_speed, right_speed):
        """Envía comando de motor."""
        if not rf_controller:
            return
        firmware_id = robot_id + 1
        rf_controller.set_motors(firmware_id, left_speed, right_speed)

    def send_state_to_viz(current_time):
        """Envía estado actual a visualización con rate limiting (~25 Hz).
        
        IMPORTANTE: Sin rate limiting, el pipe se satura y pipe.send() BLOQUEA
        hasta que haya espacio, causando pausas de ~0.6-0.7s en el loop de control.
        Esto hace que el robot se mueva en "saltos" en lugar de fluido.
        Ver: https://github.com/anomalyco/opencode/issues (problema similar)
        """
        nonlocal last_state_send_time
        # Rate limiting: enviar solo cada 40ms (~25 Hz) para evitar saturar el pipe
        if current_time - last_state_send_time >= 0.04:
            try:
                control_state_pipe.send({
                    'current_pwm': current_pwm,
                    'movement_active': movement_active,
                    'movement_direction': movement_direction,
                    'movement_duration': movement_duration,
                    'pwm_min': pwm_min_temp,
                    'pwm_max': pwm_max_temp,
                    'session_stats': session_stats.copy(),
                    'total_detections': global_stats['total_detections'],
                    'robot_id': robot_id,
                    'last_speed_px_s': last_speed_px_s,
                    'last_speed_n_samples': last_speed_n_samples,
                    'last_speed_distance_px': last_speed_distance_px,
                    'timestamp': current_time
                })
                last_state_send_time = current_time
            except Exception as e:
                log.warning(f"⚠️  Error enviando estado a viz: {e}")

    def reset_session():
        """Resetea estadísticas de sesión."""
        session_stats['frames_analyzed'] = 0
        session_stats['frames_detected'] = 0
        session_stats['detection_rate'] = 0.0

    # Flags de control
    exit_requested = False
    last_state_send_time = 0.0

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
                    value = cmd.get('value')

                    if command == 'exit':
                        exit_requested = True
                        log.info("Comando exit recibido")
                        send_motor_command(0, 0)
                        break

                    elif command == 'start_movement':
                        movement_active = True
                        movement_direction = value if value else 1
                        movement_start_time = current_time
                        reset_session()
                        position_samples.clear()
                        
                        # Enviar señal de reset a percepción para empezar a contar desde 0
                        try:
                            control_to_perception_pipe.send({'command': 'reset_stats'})
                        except Exception:
                            pass
                        
                        if movement_direction > 0:
                            send_motor_command(current_pwm, current_pwm)
                            log.info(f"▶️  Movimiento ADELANTE | PWM: {current_pwm}")
                        else:
                            send_motor_command(-current_pwm, -current_pwm)
                            log.info(f"◀️  Movimiento ATRAS | PWM: {current_pwm}")

                    elif command == 'stop_movement':
                        movement_active = False
                        send_motor_command(0, 0)
                        log.info("🛑 Movimiento detenido")

                    elif command == 'adjust_pwm':
                        if param == 'pwm':
                            delta_val = delta if delta else 0
                            current_pwm = max(5, min(127, current_pwm + delta_val))
                            log.info(f"PWM: {current_pwm}")
                            # Si hay movimiento activo, actualizar comando
                            if movement_active:
                                if movement_direction > 0:
                                    send_motor_command(current_pwm, current_pwm)
                                else:
                                    send_motor_command(-current_pwm, -current_pwm)

                    elif command == 'adjust_duration':
                        delta_val = delta if delta else 0
                        movement_duration = max(0.1, min(10.0, movement_duration + delta_val))
                        log.info(f"Duración: {movement_duration}s")

                    elif command == 'set_range':
                        if param == 'pwm_min':
                            delta_val = delta if delta else 0
                            pwm_min_temp = max(5, min(pwm_max_temp - 1, pwm_min_temp + delta_val))
                            log.info(f"PWM_min: {pwm_min_temp}")
                        elif param == 'pwm_max':
                            delta_val = delta if delta else 0
                            pwm_max_temp = max(pwm_min_temp + 1, min(127, pwm_max_temp + delta_val))
                            log.info(f"PWM_max: {pwm_max_temp}")

                    elif command == 'suggest_range':
                        log.info(f"💡 Sugerencia basada en PWM actual ({current_pwm}):")
                        log.info(f"   PWM_min sugerido: {max(5, current_pwm - 50)}")
                        log.info(f"   PWM_max sugerido: {min(127, current_pwm + 50)}")

                    elif command == 'save_range':
                        if pwm_min_temp >= pwm_max_temp:
                            log.error(f"❌ Error: PWM_min ({pwm_min_temp}) debe ser menor que PWM_max ({pwm_max_temp})")
                        else:
                            log.info(f"💾 Guardando rango PWM: [{pwm_min_temp}, {pwm_max_temp}]")
                            calibration.set_pwm_range(robot_id, pwm_min_temp, pwm_max_temp)
                            calibration.save()
                            log.info(f"✅ Rango guardado para Robot {robot_id}")

                except Exception as e:
                    log.warning(f"⚠️  Error procesando comando: {e}")

            if exit_requested:
                break

            # ===== CONTROL DE MOVIMIENTO =====
            if movement_active:
                time_elapsed = current_time - movement_start_time

                if time_elapsed < movement_duration:
                    # Enviar comando CONTINUAMENTE durante el movimiento
                    if movement_direction > 0:
                        send_motor_command(current_pwm, current_pwm)
                    else:
                        send_motor_command(-current_pwm, -current_pwm)
                else:
                    # Detener
                    send_motor_command(0, 0)
                    movement_active = False
                    log.info(f"⏹️  Movimiento completado ({time_elapsed:.2f}s)")

                    # Calcular velocidad de la sesión
                    avg_speed, n_used, total_dist = compute_avg_speed(position_samples)
                    last_speed_px_s = avg_speed
                    last_speed_n_samples = n_used
                    last_speed_distance_px = total_dist

                    # Resumen de sesión
                    log.info("=" * 50)
                    log.info("📊 RESUMEN DE SESIÓN")
                    log.info("=" * 50)
                    log.info(f"   Frames analizados: {session_stats['frames_analyzed']}")
                    log.info(f"   Frames detectados: {session_stats['frames_detected']}")
                    log.info(f"   Tasa detección: {session_stats['detection_rate']*100:.1f}%")
                    log.info(f"   FPS promedio: {global_stats['avg_fps']:.1f}")
                    if avg_speed is not None:
                        log.info("   Velocidad: %.1f px/s | %d muestras | distancia=%.1f px",
                                 avg_speed, n_used, total_dist)
                    else:
                        log.info("   Velocidad: N/A (sin detecciones suficientes)")
                    log.info("=" * 50)

            # ===== RECIBIR DATOS DE PERCEPCIÓN =====
            robot_detected = False
            robot_data = None
            perception_stats = {}

            if robot_positions_pipe.poll():
                try:
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
                    if movement_active:
                        session_stats['frames_analyzed'] = perception_stats.get('frames_analyzed', 0)
                        session_stats['frames_detected'] = perception_stats.get('frames_detected', 0)
                        session_stats['detection_rate'] = perception_stats.get('detection_rate', 0.0)

                        # Acumular muestras (timestamp, x, y) para medir velocidad
                        if robot_detected and robot_data is not None:
                            sample_t = perception_data.get('timestamp', current_time)
                            position_samples.append((
                                sample_t,
                                robot_data.get('x', 0),
                                robot_data.get('y', 0),
                            ))

                except Exception as e:
                    log.warning(f"⚠️  Error recibiendo datos de percepción: {e}")

            # ===== ENVIAR ESTADO A VISUALIZACIÓN (rate limited ~25 Hz) =====
            send_state_to_viz(current_time)

            time.sleep(0.001)

    except KeyboardInterrupt:
        log.info("⏹️  Proceso de control detenido por usuario")
    except Exception as e:
        log.error(f"❌ Error en proceso de control: {e}", exc_info=True)
    finally:
        # Detener robot
        if rf_controller:
            send_motor_command(0, 0)
            rf_controller.shutdown()

        log.info("\n" + "=" * 50)
        log.info("RESUMEN FINAL")
        log.info("=" * 50)
        log.info(f"Último PWM probado: {current_pwm}")
        log.info(f"Total de detecciones: {global_stats['total_detections']}")
        log.info(f"Rango temporal: [{pwm_min_temp}, {pwm_max_temp}]")

        # Verificar si el rango fue guardado
        current_saved_min, current_saved_max = calibration.get_pwm_range(robot_id)
        if (current_saved_min, current_saved_max) == (pwm_min_temp, pwm_max_temp):
            log.info("✅ Rango GUARDADO en JSON")
        else:
            log.info("⚠️  Rango NO guardado (presiona 'g' para guardar)")

        log.info("\n💡 Proceso sugerido:")
        log.info("   1. Encuentra PWM_min: PWM más bajo donde el robot se mueve consistentemente")
        log.info("   2. Encuentra PWM_max: PWM más alto donde la cámara detecta al robot")
        log.info("   3. Ajusta rango con n/m (min) y ,/. (max)")
        log.info("   4. Presiona 'g' para guardar")
        log.info("   5. Calibra bias: calibrate_robot_bias.py --robot-id " + str(robot_id))
        log.info("=" * 50)
