"""Proceso de decisión autónoma para robot soccer.

Instancia BehaviorManager y ejecuta el árbol de comportamiento en tiempo real,
recibiendo posiciones de percepción y enviando comandos RF a los robots.

Uso desde scripts de integración:
    from robot_soccer.core.process.decision_process import decision_process

Interfaz de pipes:
    perception_pipe  ← recv: {'robots': {id: {'x','y','angulo'}, ...},
                               'ball_detected': bool, 'ball_pos': (bx, by)}
                     (retrocompatible con {'robot_detected': bool, 'robot_data': {...}, ...})
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
from multiprocessing import Process, Pipe, Queue as MPQueue
from robot_soccer.core.process.planning_worker import planning_worker
from robot_soccer.config import (
    ROL_ATACANTE,
    ROL_DEFENSIVO,
    FIELD_CAM,
    CAPTURE_CONFIRM_DISTANCE_PX,
    DRIBBLER_HOLD_POWER,
    DRIBBLER_PULSE_ON_MS,
    DRIBBLER_PULSE_OFF_MS,
    PATH_PLANNING_OBSTACLE_CLEARANCE,
)

log = logging.getLogger(__name__)


def decision_process(
    perception_pipe,
    viz_state_pipe,
    keyboard_pipe,
    robot_ids,
    serial_port: str,
    team: str = 'red',
):
    """Ejecuta BehaviorManager en bucle, recibiendo percepción y enviando comandos RF.

    Args:
        perception_pipe: Pipe de entrada con detecciones de robots y pelota.
            Formato multi-robot: {'robots': {id: {'x','y','angulo'}, ...},
                                   'ball_detected': bool, 'ball_pos': (bx, by)}
            Formato legacy (1 robot): {'robot_detected': bool, 'robot_data': dict, ...}
        viz_state_pipe: Pipe de salida con estado para visualización.
        keyboard_pipe: Pipe de entrada con comandos de teclado ('exit', 'toggle').
        robot_ids: int o list[int] — ID(s) del robot (0-3). Si es int, se envuelve en lista.
        serial_port: Puerto serial para RF (ej. '/dev/ttyUSB0').
        team: Equipo del robot ('red' o 'blue').
    """
    # Retrocompatibilidad: si se pasa un int, envolver en lista
    if isinstance(robot_ids, int):
        robot_ids = [robot_ids]
    # robot_id primario para logging y campos legacy de viz
    robot_id = robot_ids[0]

    log.info("Decision iniciada para robots %s", robot_ids)

    players = [Player(rid, 0, 0, 0.0, team=team) for rid in robot_ids]
    player_map = {p.id: p for p in players}
    # Roles iniciales: primer robot atacante, resto defensivos
    players[0].set_rol(ROL_ATACANTE)
    for p in players[1:]:
        p.set_rol(ROL_DEFENSIVO)
    ball = Ball(0, 0)

    robot_available = False
    try:
        behavior_manager = BehaviorManager(
            players=players,
            ball=ball,
            team=team,
            use_real_robots=True,
            serial_port=serial_port,
            field=FIELD_CAM
        )
        robot_available = (behavior_manager.command_manager.rf_controller is not None
                           and behavior_manager.command_manager.use_real_robots)
        ids_str = ",".join(str(r) for r in robot_ids)
        if robot_available:
            log.info("Robots [%s] disponibles via RF", ids_str)
        else:
            log.warning("Robots [%s] NO disponibles via RF — modo simulacion", ids_str)
    except Exception as e:
        log.error("Error inicializando BehaviorManager: %s", e)
        return

    # --- Inicializar planners RRT* (un subprocess por robot) ---
    _plan_pipes    = {}   # player_id → pipe padre
    _path_queues   = {}   # player_id → Queue con el último path calculado
    _planner_procs = {}   # player_id → Process

    for pid in robot_ids:
        parent_conn, child_conn = Pipe()
        pq = MPQueue(maxsize=1)
        proc = Process(
            target=planning_worker,
            args=(child_conn, pq, PATH_PLANNING_OBSTACLE_CLEARANCE),
            daemon=True,
            name=f"Planner-R{pid}",
        )
        proc.start()
        _plan_pipes[pid]    = parent_conn
        _path_queues[pid]   = pq
        _planner_procs[pid] = proc
        log.info("Planner RRT* iniciado para robot %d (PID=%d)", pid, proc.pid)

    behavior_manager.command_manager.set_planning_channels(_plan_pipes, _path_queues)

    players_initialized = set()   # IDs de robots detectados al menos una vez
    ball_initialized = False
    behavior_active = False
    running = True

    last_viz_time = 0.0
    VIZ_INTERVAL = 0.04           # ~25 Hz

    last_bt_tick = 0.0
    BT_TICK_INTERVAL = 0.1        # BT decide a 10 Hz

    # Dribbler keepalive: estado independiente por robot
    last_dribbler_keepalive = {rid: 0.0 for rid in robot_ids}
    DRIBBLER_KEEPALIVE = 0.08     # 80ms < timeout firmware (100ms)
    dribbler_pulse_phase = {rid: 'on' for rid in robot_ids}
    dribbler_pulse_timer = {rid: 0.0 for rid in robot_ids}
    prev_has_ball = {rid: False for rid in robot_ids}

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

                    robots_dict = data.get('robots')
                    if robots_dict is not None:
                        # Formato multi-robot: {'robots': {id: {x,y,angulo}}, ...}
                        for rid, rd in robots_dict.items():
                            if rid in player_map:
                                player_map[rid].x = rd['x']
                                player_map[rid].y = rd['y']
                                player_map[rid].angle = rd['angulo']
                                if rid not in players_initialized:
                                    log.info("Robot %d detectado en (%d, %d)",
                                             rid, rd['x'], rd['y'])
                                    players_initialized.add(rid)
                    else:
                        # Formato legacy (1 robot) — retrocompatible
                        if data.get('robot_detected') and data.get('robot_data'):
                            rd = data['robot_data']
                            if robot_id in player_map:
                                player_map[robot_id].x = rd['x']
                                player_map[robot_id].y = rd['y']
                                player_map[robot_id].angle = rd['angulo']
                                if robot_id not in players_initialized:
                                    log.info("Robot detectado en (%d, %d)",
                                             rd['x'], rd['y'])
                                    players_initialized.add(robot_id)

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
                            rf = behavior_manager.command_manager.rf_controller
                            for p in players:
                                fid = p.id + 1
                                rf.set_motors(fid, 0, 0)
                                rf.set_dribbler(fid, 0)
                                p._has_ball = False
                                ctrl = behavior_manager.command_manager.controllers.get(p.id)
                                if ctrl:
                                    ctrl.max_linear_pwm_override = None
                                behavior_manager.command_manager.actions_in_progress.pop(p.id, None)
                except Exception:
                    pass

            # --- Actualizar has_ball por proximidad + alineación (por robot) ---
            # Umbral captura: CAPTURE_CONFIRM_DISTANCE_PX + heading < 30°
            # Umbral liberación: 2x distancia — permite rotación sin perder posesión
            distances = {}
            if ball_initialized:
                for p in players:
                    if p.id not in players_initialized:
                        continue
                    dist = float(p.distance_to_ball(ball))
                    distances[p.id] = dist
                    if dist < CAPTURE_CONFIRM_DISTANCE_PX:
                        _ang = math.degrees(math.atan2(
                            ball.y - p.y, ball.x - p.x))
                        _err = abs((_ang - p.angle + 180) % 360 - 180)
                        if _err < 30:
                            p._has_ball = True
                    elif dist > CAPTURE_CONFIRM_DISTANCE_PX * 2:
                        p._has_ball = False
                    # Zona de histéresis: mantener valor actual
            # distance para logging = robot primario
            distance = distances.get(robot_id)

            # --- Calcular errores angulares robot primario ↔ pelota y ↔ arco ---
            angle_to_ball_deg = None
            angle_to_goal_deg = None
            err_to_ball_deg   = None
            err_to_goal_deg   = None
            primary = player_map[robot_id]
            if robot_id in players_initialized and ball_initialized:
                angle_to_ball_deg = math.degrees(math.atan2(
                    ball.y - primary.y, ball.x - primary.x))
                angle_to_goal_deg = math.degrees(math.atan2(
                    OPPONENT_GOAL_POS[1] - primary.y,
                    OPPONENT_GOAL_POS[0] - primary.x))
                # Error normalizado a (-180, 180]: positivo = girar CCW, negativo = CW
                err_to_ball_deg = (angle_to_ball_deg - primary.angle + 180) % 360 - 180
                err_to_goal_deg = (angle_to_goal_deg - primary.angle + 180) % 360 - 180

            # --- Actualizar contexto del juego ---
            if players_initialized and ball_initialized:
                posesion = 0.0 if any(p.has_ball() for p in players) else 0.5
                proximidad = 0.5   # siempre "cerca" con robots del mismo equipo sin rivales
                zona = (ball.x / FIELD_CAM.width) * 2.0
                zona = max(0.0, min(2.0, zona))
                behavior_manager.update_game_context((posesion, proximidad, zona))

            # --- Dribbler keepalive con pulso intermitente (por robot) ---
            if behavior_active and robot_available:
                rf = behavior_manager.command_manager.rf_controller
                for p in behavior_manager.team_players:
                    rid = p.id
                    firmware_id = rid + 1

                    # Resetear ciclo de pulso al capturar la pelota (flanco de subida)
                    if p._has_ball and not prev_has_ball.get(rid, False):
                        dribbler_pulse_phase[rid] = 'on'
                        dribbler_pulse_timer[rid] = now

                    if p._has_ball:
                        pulse_on_s  = DRIBBLER_PULSE_ON_MS  / 1000.0
                        pulse_off_s = DRIBBLER_PULSE_OFF_MS / 1000.0

                        if pulse_off_s <= 0:
                            # Modo continuo
                            if now - last_dribbler_keepalive[rid] >= DRIBBLER_KEEPALIVE:
                                rf.set_dribbler(firmware_id, DRIBBLER_HOLD_POWER)
                                last_dribbler_keepalive[rid] = now
                        else:
                            phase_elapsed = now - dribbler_pulse_timer[rid]
                            if dribbler_pulse_phase[rid] == 'on':
                                if now - last_dribbler_keepalive[rid] >= DRIBBLER_KEEPALIVE:
                                    rf.set_dribbler(firmware_id, DRIBBLER_HOLD_POWER)
                                    last_dribbler_keepalive[rid] = now
                                if phase_elapsed >= pulse_on_s:
                                    rf.set_dribbler(firmware_id, 0)
                                    dribbler_pulse_phase[rid] = 'off'
                                    dribbler_pulse_timer[rid] = now
                            else:  # 'off'
                                if phase_elapsed >= pulse_off_s:
                                    rf.set_dribbler(firmware_id, DRIBBLER_HOLD_POWER)
                                    last_dribbler_keepalive[rid] = now
                                    dribbler_pulse_phase[rid] = 'on'
                                    dribbler_pulse_timer[rid] = now

            for p in players:
                prev_has_ball[p.id] = p._has_ball

            # --- Actualizar posiciones de todos los robots para el planner ---
            _all_robot_data = [
                {'id': p.id, 'x': p.x, 'y': p.y}
                for p in players
                if p.id in players_initialized
            ]
            behavior_manager.command_manager.update_robot_data(_all_robot_data)
            if ball_initialized:
                behavior_manager.command_manager.update_ball_data(ball.x, ball.y)

            # --- Ejecutar árbol de comportamiento ---
            # Requiere que TODOS los robots hayan sido detectados al menos una vez
            all_initialized = players_initialized.issuperset(set(robot_ids))
            if behavior_active and all_initialized and ball_initialized:
                try:
                    # BT decide a 10 Hz: setea el target (move_robot_to)
                    # BT usa player.angle en GRADOS (capture_ball, is_shot_possible)
                    if now - last_bt_tick >= BT_TICK_INTERVAL:
                        behavior_manager.update()
                        last_bt_tick = now

                    # execute_commands corre a ~100 Hz: envía comandos RF cada loop.
                    # DifferentialDriveController espera robot.angle en RADIANES.
                    # Conversión temporal: grados → radianes → ejecutar → restaurar
                    angle_degs = {p.id: p.angle for p in players}
                    for p in players:
                        p.angle = math.radians(p.angle)
                    behavior_manager.command_manager.execute_commands()
                    for p in players:
                        p.angle = angle_degs[p.id]
                except Exception as e:
                    log.error("Error en BehaviorManager: %s", e)

            # --- Detectar transiciones de fase del BT (robot primario) ---
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

            # --- Status periódico unificado (2 Hz cuando BT activo, por robot) ---
            if behavior_active and players_initialized and ball_initialized:
                if now - last_status_log >= 0.5:
                    for p in players:
                        if p.id not in players_initialized:
                            continue
                        action_info = behavior_manager.command_manager.actions_in_progress.get(p.id)
                        tgt_pos_v = tgt_ang_v = dist_v = None
                        if action_info:
                            if action_info['type'] == 'move':
                                tp = action_info['target_pos']
                                tgt_pos_v = (int(tp[0]), int(tp[1]))
                                dist_v = float(np.linalg.norm(
                                    np.array([p.x, p.y]) - np.array(tp[:2])
                                ))
                            elif action_info['type'] == 'rotate':
                                tgt_ang_v = action_info['target_angle']
                        blackboard_p = behavior_manager.blackboards.get(p.id)
                        bt_action_p = blackboard_p.last_action if blackboard_p else None
                        robot_status_logger.update(
                            p.id,
                            state=bt_action_p or "unknown",
                            ang=p.angle,
                            pos=(int(p.x), int(p.y)),
                            tgt_pos=tgt_pos_v,
                            tgt_ang=tgt_ang_v,
                            ball_err=err_to_ball_deg if p.id == robot_id else None,
                            goal_err=err_to_goal_deg if p.id == robot_id else None,
                            dist=dist_v,
                        )
                        robot_status_logger.emit(p.id)
                    last_status_log = now

            # --- Enviar estado a visualización ---
            if now - last_viz_time >= VIZ_INTERVAL:
                try:
                    players_viz = {}
                    for p in players:
                        blackboard_p = behavior_manager.blackboards.get(p.id)
                        last_action_p = blackboard_p.last_action if blackboard_p else None
                        action_info_p = behavior_manager.command_manager.actions_in_progress.get(p.id)
                        action_type_p = None
                        current_target_p = None
                        if action_info_p:
                            action_type_p = action_info_p.get('type')
                            if 'target_pos' in action_info_p:
                                tp = action_info_p['target_pos']
                                try:
                                    current_target_p = (int(tp[0]), int(tp[1]))
                                except Exception:
                                    pass
                        players_viz[p.id] = {
                            'pos': (p.x, p.y) if p.id in players_initialized else None,
                            'angle_deg': p.angle if p.id in players_initialized else None,
                            'rol': 'atacante' if p.rol == ROL_ATACANTE else 'defensor',
                            'has_ball': p.has_ball(),
                            'last_action': str(last_action_p) if last_action_p else None,
                            'target': current_target_p,
                            'action_type': action_type_p,
                        }

                    # Robot primario para campos legacy (backward compat con test_behavior_1robot.py)
                    prim = player_map[robot_id]
                    prim_viz = players_viz.get(robot_id, {})

                    viz_state_pipe.send({
                        'players': players_viz,
                        'ball_pos': (int(ball.x), int(ball.y)) if ball_initialized else None,
                        'behavior_active': behavior_active,
                        'robot_available': robot_available,
                        # Campos legacy para backward compat
                        'player_pos': (prim.x, prim.y) if robot_id in players_initialized else None,
                        'robot_angle_deg': prim.angle if robot_id in players_initialized else None,
                        'has_ball': prim.has_ball(),
                        'last_action': prim_viz.get('last_action'),
                        'current_target': prim_viz.get('target'),
                        'action_type': prim_viz.get('action_type'),
                        'distance': distance,
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
        for proc in _planner_procs.values():
            proc.terminate()
            proc.join(timeout=1.0)
        try:
            behavior_manager.shutdown()
        except Exception:
            pass
        log.info("Decision finalizada")
