"""Proceso de decisión autónoma para robot soccer.

Instancia BehaviorManager y ejecuta el árbol de comportamiento en tiempo real,
recibiendo posiciones de percepción y enviando comandos RF a los robots.

Uso desde scripts de integración:
    from robot_soccer.core.process.decision_process import decision_process

Interfaz de pipes:
    perception_pipe  ← recv: {'robot_detected': bool, 'robot_data': {'x','y','angulo'},
                               'ball_detected': bool, 'ball_pos': (bx, by)}
    viz_state_pipe   → send: estado actual para visualización
    keyboard_pipe    ← recv: {'command': 'exit'|'toggle'}
"""
import math
import time
import logging
import numpy as np

from robot_soccer.entities.player import Player
from robot_soccer.entities.ball import Ball
from robot_soccer.ai.behavior_tree.manager import BehaviorManager
from robot_soccer.utils.robot_logger import robot_status_logger
from robot_soccer.config import (
    ROL_ATACANTE,
    FIELD_CAM,
    CAPTURE_CONFIRM_DISTANCE_PX,
    DRIBBLER_HOLD_POWER,
    DRIBBLER_PULSE_ON_MS,
    DRIBBLER_PULSE_OFF_MS,
)

log = logging.getLogger(__name__)


def decision_process(
    perception_pipe,
    viz_state_pipe,
    keyboard_pipe,
    robot_id: int,
    serial_port: str,
    team: str = 'red',
):
    """Ejecuta BehaviorManager en bucle, recibiendo percepción y enviando comandos RF.

    Args:
        perception_pipe: Pipe de entrada con detecciones de robot y pelota.
        viz_state_pipe: Pipe de salida con estado para visualización.
        keyboard_pipe: Pipe de entrada con comandos de teclado ('exit', 'toggle').
        robot_id: ID del robot (0-3).
        serial_port: Puerto serial para RF (ej. '/dev/ttyUSB0').
        team: Equipo del robot ('red' o 'blue').
    """
    log.info("Decision iniciada para robot %d", robot_id)

    player = Player(robot_id, 0, 0, 0.0, team=team)
    player.set_rol(ROL_ATACANTE)
    ball = Ball(0, 0)

    robot_available = False
    try:
        behavior_manager = BehaviorManager(
            players=[player],
            ball=ball,
            team=team,
            use_real_robots=True,
            serial_port=serial_port,
            field=FIELD_CAM
        )
        robot_available = (behavior_manager.command_manager.rf_controller is not None
                           and behavior_manager.command_manager.use_real_robots)
        if robot_available:
            log.info("Robot %d disponible via RF", robot_id)
        else:
            log.warning("Robot %d NO disponible via RF — modo simulacion", robot_id)
    except Exception as e:
        log.error("Error inicializando BehaviorManager: %s", e)
        return

    player_initialized = False
    ball_initialized = False
    behavior_active = False
    running = True

    last_viz_time = 0.0
    VIZ_INTERVAL = 0.04           # ~25 Hz

    last_bt_tick = 0.0
    BT_TICK_INTERVAL = 0.1        # BT decide a 10 Hz

    last_dribbler_keepalive = 0.0
    DRIBBLER_KEEPALIVE = 0.08     # 80ms < timeout firmware (100ms)
    dribbler_pulse_phase = 'on'
    dribbler_pulse_timer = 0.0
    prev_has_ball = False

    prev_bt_action = None
    last_status_log = 0.0         # throttle de status (0.5 Hz → cada 2s)
    phase_start_time = {}

    OPPONENT_GOAL_POS = FIELD_CAM.goal_right_center  # arco rival para equipo 'red'

    angle_to_ball_deg = None
    angle_to_goal_deg = None
    err_to_ball_deg   = None
    err_to_goal_deg   = None

    log.info("BehaviorManager listo. Presiona ESPACIO para activar comportamiento.")

    try:
        while running:
            now = time.time()

            # --- Recibir datos de percepcion ---
            if perception_pipe.poll():
                try:
                    data = perception_pipe.recv()

                    if data.get('robot_detected') and data.get('robot_data'):
                        rd = data['robot_data']
                        player.x = rd['x']
                        player.y = rd['y']
                        # player.angle se mantiene en GRADOS para el BT
                        # (capture_ball, is_shot_possible usan operaciones en grados)
                        # La conversión a radianes se hace puntualmente en execute_commands()
                        player.angle = rd['angulo']
                        if not player_initialized:
                            log.info("Robot detectado en (%d, %d)", rd['x'], rd['y'])
                        player_initialized = True

                    if data.get('ball_detected') and data.get('ball_pos'):
                        bx, by = data['ball_pos']
                        ball.set_position(bx, by)
                        if not ball_initialized:
                            log.info("Pelota detectada en (%d, %d)", bx, by)
                        ball_initialized = True
                    # Si la pelota no es visible, mantener última posición conocida

                except Exception:
                    pass

            # --- Recibir comandos de teclado ---
            if keyboard_pipe.poll():
                try:
                    cmd = keyboard_pipe.recv()
                    command = cmd.get('command', '')
                    if command == 'exit':
                        running = False
                    elif command == 'toggle':
                        behavior_active = not behavior_active
                        log.info("Comportamiento %s",
                                 "ACTIVADO" if behavior_active else "PAUSADO")
                        if not behavior_active and robot_available:
                            firmware_id = robot_id + 1
                            rf = behavior_manager.command_manager.rf_controller
                            rf.set_motors(firmware_id, 0, 0)
                            rf.set_dribbler(firmware_id, 0)
                            player._has_ball = False
                            ctrl = behavior_manager.command_manager.controllers.get(robot_id)
                            if ctrl:
                                ctrl.max_linear_pwm_override = None
                            behavior_manager.command_manager.actions_in_progress.pop(robot_id, None)
                except Exception:
                    pass

            # --- Actualizar has_ball por proximidad + alineación ---
            # Requiere que el robot esté cerca Y mirando la pelota.
            # Sin verificar alineación, el robot "tiene" la pelota por estar
            # cerca pero apuntando al lado opuesto, causando kicks espurios.
            # Umbral captura: CAPTURE_CONFIRM_DISTANCE_PX + heading < 30°
            # Umbral liberación: 2x distancia — permite rotación sin perder posesión
            distance = None
            if player_initialized and ball_initialized:
                distance = float(player.distance_to_ball(ball))
                if distance < CAPTURE_CONFIRM_DISTANCE_PX:
                    _ang = math.degrees(math.atan2(
                        ball.y - player.y, ball.x - player.x))
                    _err = abs((_ang - player.angle + 180) % 360 - 180)
                    if _err < 30:
                        player._has_ball = True
                elif distance > CAPTURE_CONFIRM_DISTANCE_PX * 2:
                    player._has_ball = False
                # Zona de histéresis: mantener valor actual

            # --- Calcular errores angulares robot↔pelota y robot↔arco ---
            angle_to_ball_deg = None
            angle_to_goal_deg = None
            err_to_ball_deg   = None
            err_to_goal_deg   = None
            if player_initialized and ball_initialized:
                angle_to_ball_deg = math.degrees(math.atan2(
                    ball.y - player.y, ball.x - player.x))
                angle_to_goal_deg = math.degrees(math.atan2(
                    OPPONENT_GOAL_POS[1] - player.y,
                    OPPONENT_GOAL_POS[0] - player.x))
                # Error normalizado a (-180, 180]: positivo = girar CCW, negativo = CW
                err_to_ball_deg = (angle_to_ball_deg - player.angle + 180) % 360 - 180
                err_to_goal_deg = (angle_to_goal_deg - player.angle + 180) % 360 - 180

            # --- Actualizar contexto del juego ---
            if player_initialized and ball_initialized:
                posesion = 0.0 if player.has_ball() else 0.5
                proximidad = 0.5   # siempre "cerca" con 1 robot sin rivales
                zona = (ball.x / FIELD_CAM.width) * 2.0
                zona = max(0.0, min(2.0, zona))
                behavior_manager.update_game_context((posesion, proximidad, zona))

            # --- Dribbler keepalive con pulso intermitente ---
            if behavior_active and player._has_ball and robot_available:
                firmware_id = robot_id + 1
                rf = behavior_manager.command_manager.rf_controller

                # Resetear ciclo de pulso al capturar la pelota (flanco de subida)
                if player._has_ball and not prev_has_ball:
                    dribbler_pulse_phase = 'on'
                    dribbler_pulse_timer = now

                pulse_on_s  = DRIBBLER_PULSE_ON_MS  / 1000.0
                pulse_off_s = DRIBBLER_PULSE_OFF_MS / 1000.0

                if pulse_off_s <= 0:
                    # Modo continuo
                    if now - last_dribbler_keepalive >= DRIBBLER_KEEPALIVE:
                        rf.set_dribbler(firmware_id, DRIBBLER_HOLD_POWER)
                        last_dribbler_keepalive = now
                else:
                    phase_elapsed = now - dribbler_pulse_timer
                    if dribbler_pulse_phase == 'on':
                        if now - last_dribbler_keepalive >= DRIBBLER_KEEPALIVE:
                            rf.set_dribbler(firmware_id, DRIBBLER_HOLD_POWER)
                            last_dribbler_keepalive = now
                        if phase_elapsed >= pulse_on_s:
                            rf.set_dribbler(firmware_id, 0)
                            dribbler_pulse_phase = 'off'
                            dribbler_pulse_timer = now
                    else:  # 'off'
                        if phase_elapsed >= pulse_off_s:
                            rf.set_dribbler(firmware_id, DRIBBLER_HOLD_POWER)
                            last_dribbler_keepalive = now
                            dribbler_pulse_phase = 'on'
                            dribbler_pulse_timer = now

            prev_has_ball = player._has_ball

            # --- Ejecutar árbol de comportamiento ---
            if behavior_active and player_initialized and ball_initialized:
                try:
                    # BT decide a 10 Hz: setea el target (move_robot_to)
                    # BT usa player.angle en GRADOS (capture_ball, is_shot_possible)
                    if now - last_bt_tick >= BT_TICK_INTERVAL:
                        behavior_manager.update()
                        last_bt_tick = now

                    # execute_commands corre a ~100 Hz: envía comandos RF cada loop.
                    # DifferentialDriveController espera robot.angle en RADIANES.
                    # Conversión temporal: grados → radianes → ejecutar → restaurar
                    angle_deg = player.angle
                    player.angle = math.radians(angle_deg)
                    behavior_manager.command_manager.execute_commands()
                    player.angle = angle_deg
                except Exception as e:
                    log.error("Error en BehaviorManager: %s", e)

            # --- Detectar transiciones de fase del BT ---
            try:
                blackboard = behavior_manager.blackboards.get(robot_id)
                bt_action = blackboard.last_action if blackboard else None
                if bt_action != prev_bt_action:
                    if behavior_active:
                        if prev_bt_action is not None and prev_bt_action in phase_start_time:
                            elapsed = now - phase_start_time[prev_bt_action]
                            robot_status_logger.emit_event(
                                robot_id,
                                f"FASE FIN:  {prev_bt_action:<24s} duracion={elapsed:.2f}s"
                            )
                        d_str  = f" dist={distance:.0f}px"        if distance is not None else ""
                        eb_str = f" Dball={err_to_ball_deg:+.1f}°" if err_to_ball_deg is not None else ""
                        eg_str = f" Dgoal={err_to_goal_deg:+.1f}°" if err_to_goal_deg is not None else ""
                        robot_status_logger.emit_event(
                            robot_id,
                            f"FASE INIT: {(bt_action or '?'):<24s}{d_str}{eb_str}{eg_str}"
                        )
                        if bt_action:
                            phase_start_time[bt_action] = now
                    prev_bt_action = bt_action
            except Exception:
                pass

            # --- Status periódico unificado (2 Hz cuando BT activo) ---
            if behavior_active and player_initialized and ball_initialized:
                if now - last_status_log >= 0.5:
                    action_info = behavior_manager.command_manager.actions_in_progress.get(robot_id)
                    tgt_pos_v = tgt_ang_v = dist_v = None
                    if action_info:
                        if action_info['type'] == 'move':
                            tp = action_info['target_pos']
                            tgt_pos_v = (int(tp[0]), int(tp[1]))
                            dist_v = float(np.linalg.norm(
                                np.array([player.x, player.y]) - np.array(tp[:2])
                            ))
                        elif action_info['type'] == 'rotate':
                            tgt_ang_v = action_info['target_angle']
                    robot_status_logger.update(
                        robot_id,
                        state=prev_bt_action or "unknown",
                        ang=player.angle,
                        pos=(int(player.x), int(player.y)),
                        tgt_pos=tgt_pos_v,
                        tgt_ang=tgt_ang_v,
                        ball_err=err_to_ball_deg,
                        goal_err=err_to_goal_deg,
                        dist=dist_v,
                    )
                    robot_status_logger.emit(robot_id)
                    last_status_log = now

            # --- Enviar estado a visualización ---
            if now - last_viz_time >= VIZ_INTERVAL:
                try:
                    blackboard = behavior_manager.blackboards.get(robot_id)
                    last_action = blackboard.last_action if blackboard else None

                    current_target = None
                    action_type = None
                    action_info = behavior_manager.command_manager.actions_in_progress.get(robot_id)
                    if action_info:
                        action_type = action_info.get('type')
                        if 'target_pos' in action_info:
                            tp = action_info['target_pos']
                            try:
                                current_target = (int(tp[0]), int(tp[1]))
                            except Exception:
                                pass

                    viz_state_pipe.send({
                        'player_pos': (player.x, player.y) if player_initialized else None,
                        'robot_angle_deg': player.angle if player_initialized else None,
                        'ball_pos': (int(ball.x), int(ball.y)) if ball_initialized else None,
                        'behavior_active': behavior_active,
                        'has_ball': player.has_ball(),
                        'last_action': str(last_action) if last_action else None,
                        'current_target': current_target,
                        'action_type': action_type,
                        'distance': distance,
                        'robot_available': robot_available,
                        'angle_to_ball': angle_to_ball_deg,
                        'angle_to_goal': angle_to_goal_deg,
                        'err_to_ball': err_to_ball_deg,
                        'err_to_goal': err_to_goal_deg,
                        'timestamp': now
                    })
                    last_viz_time = now
                except Exception:
                    pass

            time.sleep(0.01)  # ~100 Hz

    finally:
        try:
            behavior_manager.shutdown()
        except Exception:
            pass
        log.info("Decision finalizada")
