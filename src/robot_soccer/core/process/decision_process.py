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
    RESET_POS,
    RESET_MOVE_FACTOR,
    RESET_ANGLE,
)

log = logging.getLogger(__name__)


def decision_process(
    perception_pipe,
    viz_state_pipe,
    keyboard_pipe,
    robot_ids,
    serial_port: str,
    team: str = 'red',
    opponent_ids=None,
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
        opponent_ids: list[int] — IDs de robots rivales para modelar como obstáculos en RRT*.
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

    # Jugadores rivales — actualizados desde percepción, usados solo como obstáculos
    opponent_team = 'blue' if team == 'red' else 'red'
    opponent_players = [Player(rid, 0, 0, 0.0, team=opponent_team)
                        for rid in (opponent_ids or [])]
    opponent_map = {p.id: p for p in opponent_players}
    all_players = players + opponent_players

    ball = Ball(0, 0)

    robot_available = False
    try:
        behavior_manager = BehaviorManager(
            players=all_players,
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
                            elif rid in opponent_map:
                                opponent_map[rid].x = rd['x']
                                opponent_map[rid].y = rd['y']
                                opponent_map[rid].angle = rd['angulo']
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
                    elif command == 'tablero':
                        cmd_num = cmd.get('cmd')
                        if cmd_num and robot_available:
                            rf = behavior_manager.command_manager.rf_controller
                            if rf:
                                rf.send_tablero(cmd_num)
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
                for p in all_players
                if p.id in players_initialized or p.id in opponent_map
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
                        path_p = behavior_manager.command_manager._current_paths.get(p.id, [])
                        wp_idx_p = behavior_manager.command_manager._current_wp_idx.get(p.id, 0)
                        rrt_len_p = max(0, len(path_p) - wp_idx_p) if path_p else 0
                        n_obs_p = len([r for r in _all_robot_data if r['id'] != p.id])
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
                            rrt_len=rrt_len_p,
                            n_obs=n_obs_p,
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
        # Resetear tablero antes de cerrar la conexión RF
        if robot_available:
            try:
                rf = behavior_manager.command_manager.rf_controller
                if rf and rf.serial_manager.is_connected:
                    rf.send_tablero(4)   # reset goles → 0:0
                    rf.send_tablero(5)   # reset tiempo → minutos preestablecidos
                    time.sleep(0.15)     # dar tiempo al worker serial para enviar
                    log.info("Tablero: goles y tiempo reseteados al cerrar")
            except Exception:
                pass
        for proc in _planner_procs.values():
            proc.terminate()
            proc.join(timeout=1.0)
        try:
            behavior_manager.shutdown()
        except Exception:
            pass
        log.info("Decision finalizada")


def _assign_reset(team_players, pos_dict):
    """Asigna posiciones de reset minimizando distancia total de viaje (2! permutaciones).

    Evita el escenario donde un robot apunta a una posición ocupada por su compañero,
    que hace que RRT* falle y el robot caiga a PID directo sin evasión de obstáculos.
    """
    from itertools import permutations as _perms
    ids = [p.id for p in team_players]
    positions = [pos_dict[pid] for pid in ids]
    best = {pid: pos for pid, pos in zip(ids, positions)}
    best_cost = float('inf')
    for perm in _perms(range(len(ids))):
        cost = sum(
            math.hypot(team_players[i].x - positions[perm[i]][0],
                       team_players[i].y - positions[perm[i]][1])
            for i in range(len(ids))
        )
        if cost < best_cost:
            best_cost = cost
            best = {ids[i]: positions[perm[i]] for i in range(len(ids))}
    return best


def decision_process_2v2(
    perception_pipe,
    viz_state_pipe,
    keyboard_pipe,
    team_red_ids,
    team_blue_ids,
    serial_port: str,
):
    """Proceso de decisión para partido 2v2: dos BehaviorManagers, un solo RFController.

    Ambos equipos comparten el mismo puerto serial. Cada BehaviorManager controla
    únicamente los robots de su equipo (filtrado por p.team en BehaviorManager.__init__),
    pero recibe posiciones de todos para modelar obstáculos en RRT*.

    Args:
        perception_pipe: Pipe con {'robots': {id: {x,y,angulo}}, 'ball_detected', 'ball_pos'}.
        viz_state_pipe: Pipe de salida con estado de los 4 robots para visualización.
        keyboard_pipe: Pipe con {'command': 'exit'|'toggle'|'toggle_red'|'toggle_blue'}.
        team_red_ids: list[int] — IDs Python del equipo rojo (ej. [0, 1]).
        team_blue_ids: list[int] — IDs Python del equipo azul (ej. [2, 3]).
        serial_port: Puerto serial compartido por ambos equipos.
    """
    from robot_soccer.communication.rf_controller import RFController as _RFController

    all_ids = team_red_ids + team_blue_ids
    log.info("Decision 2v2 iniciada | rojo=%s azul=%s", team_red_ids, team_blue_ids)

    # --- Un solo RFController para ambos equipos ---
    rf = _RFController(port=serial_port)
    robot_available = rf.initialize()
    if not robot_available:
        log.error("No se pudo inicializar RF en %s — abortando Decision 2v2", serial_port)
        return
    log.info("RF compartido OK en %s", serial_port)

    # --- Crear jugadores ---
    players_red  = [Player(rid, 0, 0, 0.0, team='red')  for rid in team_red_ids]
    players_blue = [Player(rid, 0, 0, 0.0, team='blue') for rid in team_blue_ids]
    all_players  = players_red + players_blue
    player_map   = {p.id: p for p in all_players}

    players_red[0].set_rol(ROL_ATACANTE)
    for p in players_red[1:]:
        p.set_rol(ROL_DEFENSIVO)
    players_blue[0].set_rol(ROL_ATACANTE)
    for p in players_blue[1:]:
        p.set_rol(ROL_DEFENSIVO)

    ball = Ball(0, 0)

    # --- BehaviorManagers: comparten rf, reciben all_players ---
    try:
        bm_red = BehaviorManager(
            players=all_players, ball=ball, team='red',
            use_real_robots=True, field=FIELD_CAM, rf_controller=rf,
        )
        bm_blue = BehaviorManager(
            players=all_players, ball=ball, team='blue',
            use_real_robots=True, field=FIELD_CAM, rf_controller=rf,
        )
    except Exception as e:
        log.error("Error inicializando BehaviorManagers: %s", e)
        rf.shutdown()
        return

    # --- Planners RRT* (uno por robot) ---
    _plan_pipes_red  = {}
    _path_queues_red = {}
    _plan_pipes_blue  = {}
    _path_queues_blue = {}
    _planner_procs   = {}

    for pid in team_red_ids:
        parent_conn, child_conn = Pipe()
        pq = MPQueue(maxsize=1)
        proc = Process(target=planning_worker,
                       args=(child_conn, pq, PATH_PLANNING_OBSTACLE_CLEARANCE),
                       daemon=True, name=f"Planner-R{pid}")
        proc.start()
        _plan_pipes_red[pid]  = parent_conn
        _path_queues_red[pid] = pq
        _planner_procs[pid]   = proc
        log.info("Planner RRT* R%d (rojo) PID=%d", pid, proc.pid)

    for pid in team_blue_ids:
        parent_conn, child_conn = Pipe()
        pq = MPQueue(maxsize=1)
        proc = Process(target=planning_worker,
                       args=(child_conn, pq, PATH_PLANNING_OBSTACLE_CLEARANCE),
                       daemon=True, name=f"Planner-R{pid}")
        proc.start()
        _plan_pipes_blue[pid]  = parent_conn
        _path_queues_blue[pid] = pq
        _planner_procs[pid]    = proc
        log.info("Planner RRT* R%d (azul)  PID=%d", pid, proc.pid)

    bm_red.command_manager.set_planning_channels(_plan_pipes_red, _path_queues_red)
    bm_blue.command_manager.set_planning_channels(_plan_pipes_blue, _path_queues_blue)

    # --- Estado ---
    players_initialized = set()
    ball_initialized    = False
    active_red          = False
    active_blue         = False
    running             = True
    ball_out_active     = False   # pelota fuera — robots congelados, BT pausado
    ball_out_frames     = 0       # frames consecutivos con pelota fuera (histéresis)
    goal_reset_active   = False   # pausa post-gol — robots navegan a reset positions
    reset_orient_issued = False   # orientación inicial emitida (fase 1 del reset post-gol)
    reset_orient_done   = False   # orientación completada → emitir movimiento (fase 2)
    init_phase          = True    # pre-juego: robots se ordenan antes del primer saque
    init_orient_issued  = False   # orientación emitida (fase 1)
    init_orient_done    = False   # orientación completada → emitir movimiento (fase 2)

    VIZ_INTERVAL    = 0.04
    BT_TICK_INTERVAL = 0.1
    DRIBBLER_KEEPALIVE = 0.08
    OPPONENT_GOAL_RED  = FIELD_CAM.goal_right_center
    OPPONENT_GOAL_BLUE = FIELD_CAM.goal_left_center

    last_viz_time   = 0.0
    last_bt_tick    = 0.0
    last_status_log = 0.0

    last_dribbler_keepalive = {rid: 0.0 for rid in all_ids}
    dribbler_pulse_phase    = {rid: 'on' for rid in all_ids}
    dribbler_pulse_timer    = {rid: 0.0 for rid in all_ids}
    prev_has_ball           = {rid: False for rid in all_ids}

    log.info("Decision 2v2 lista. ESPACIO=ambos, R=rojo, B=azul.")

    try:
        while running:
            now = time.time()

            # --- Percepción ---
            if perception_pipe.poll():
                try:
                    data = perception_pipe.recv()
                    robots_dict = data.get('robots', {})
                    for rid, rd in (robots_dict or {}).items():
                        if rid in player_map:
                            player_map[rid].x     = rd['x']
                            player_map[rid].y     = rd['y']
                            player_map[rid].angle = rd['angulo']
                            if rid not in players_initialized:
                                log.info("Robot %d (%s) detectado en (%d, %d)",
                                         rid, player_map[rid].team, rd['x'], rd['y'])
                                players_initialized.add(rid)
                    if data.get('ball_detected') and data.get('ball_pos'):
                        bx, by = data['ball_pos']
                        ball.set_position(bx, by)
                        if not ball_initialized:
                            log.info("Pelota detectada en (%d, %d)", bx, by)
                        ball_initialized = True

                    # Pelota fuera: congelar robots cuando sale de límites.
                    # Durante init_phase o goal_reset_active el juego no está RUNNING:
                    # ignorar ball_out igual que en SSL (árbitro maneja la pelota).
                    ball_out_new = data.get('ball_out', False)
                    game_running = not init_phase and not goal_reset_active
                    # Histéresis: requerir 3 frames consecutivos para activar ball_out.
                    # Previene oscilaciones cuando la pelota roza el borde o parpadea.
                    if ball_out_new and game_running:
                        ball_out_frames += 1
                    else:
                        ball_out_frames = 0
                    if ball_out_frames >= 3 and not ball_out_active and game_running:
                        ball_out_active = True
                        log.info("PELOTA FUERA: robots congelados")
                        for fid in [p.id + 1 for p in all_players]:
                            rf.set_motors(fid, 0, 0)
                        for bm in (bm_red, bm_blue):
                            for p in bm.team_players:
                                bm.command_manager.actions_in_progress.pop(p.id, None)
                    elif not ball_out_new and ball_out_active:
                        ball_out_frames = 0
                        ball_out_active = False
                        log.info("PELOTA EN JUEGO: reanudando")
                    elif ball_out_active and not game_running:
                        # Transición a init/reset mientras ball_out estaba activo → limpiar
                        ball_out_frames = 0
                        ball_out_active = False
                except Exception:
                    pass

            # --- Teclado ---
            if keyboard_pipe.poll():
                try:
                    cmd = keyboard_pipe.recv()
                    command = cmd.get('command', '')
                    if command == 'exit':
                        running = False
                    elif command == 'goal_scored':
                        goal_reset_active   = True
                        reset_orient_issued = False
                        reset_orient_done   = False
                        active_red = active_blue = False
                        for bm in (bm_red, bm_blue):
                            for p in bm.team_players:
                                rf.set_motors(p.id + 1, 0, 0)
                                rf.set_dribbler(p.id + 1, 0)
                                p._has_ball = False
                                ctrl = bm.command_manager.controllers.get(p.id)
                                if ctrl:
                                    ctrl.max_linear_pwm_override = None
                                bm.command_manager.actions_in_progress.pop(p.id, None)
                        log.info("GOL: iniciando reset a posiciones iniciales")
                    elif command == 'toggle':
                        if init_phase:
                            init_phase = False
                            init_orient_issued = init_orient_done = False
                            active_red = active_blue = True
                            for bm in (bm_red, bm_blue):
                                for p in bm.team_players:
                                    ctrl = bm.command_manager.controllers.get(p.id)
                                    if ctrl:
                                        ctrl.max_linear_pwm_override = None
                                    bm.command_manager.actions_in_progress.pop(p.id, None)
                            log.info("PARTIDO INICIADO desde posiciones de inicio")
                        elif goal_reset_active:
                            goal_reset_active = False
                            active_red = active_blue = True
                            for bm in (bm_red, bm_blue):
                                for p in bm.team_players:
                                    ctrl = bm.command_manager.controllers.get(p.id)
                                    if ctrl:
                                        ctrl.max_linear_pwm_override = None
                                    bm.command_manager.actions_in_progress.pop(p.id, None)
                            log.info("Reanudando partido tras gol")
                        else:
                            new_state = not (active_red or active_blue)
                            active_red = active_blue = new_state
                            log.info("Ambos equipos: %s", "ACTIVOS" if new_state else "PAUSADOS")
                            if not new_state:
                                for bm in (bm_red, bm_blue):
                                    for p in bm.team_players:
                                        rf.set_motors(p.id + 1, 0, 0)
                                        rf.set_dribbler(p.id + 1, 0)
                                        p._has_ball = False
                                        ctrl = bm.command_manager.controllers.get(p.id)
                                        if ctrl:
                                            ctrl.max_linear_pwm_override = None
                                        bm.command_manager.actions_in_progress.pop(p.id, None)
                    elif command == 'toggle_red':
                        active_red = not active_red
                        log.info("Equipo rojo: %s", "ACTIVO" if active_red else "PAUSADO")
                        if not active_red:
                            for p in bm_red.team_players:
                                rf.set_motors(p.id + 1, 0, 0)
                                rf.set_dribbler(p.id + 1, 0)
                                p._has_ball = False
                                bm_red.command_manager.actions_in_progress.pop(p.id, None)
                    elif command == 'toggle_blue':
                        active_blue = not active_blue
                        log.info("Equipo azul: %s", "ACTIVO" if active_blue else "PAUSADO")
                        if not active_blue:
                            for p in bm_blue.team_players:
                                rf.set_motors(p.id + 1, 0, 0)
                                rf.set_dribbler(p.id + 1, 0)
                                p._has_ball = False
                                bm_blue.command_manager.actions_in_progress.pop(p.id, None)
                    elif command == 'tablero':
                        cmd_num = cmd.get('cmd')
                        if cmd_num:
                            rf.send_tablero(cmd_num)
                except Exception:
                    pass

            # --- has_ball: solo el robot más cercano puede tener la pelota (estilo SSL) ---
            # Si dos robots están dentro del umbral, solo el MÁS CERCANO recibe has_ball=True.
            # Evita deadlock donde ambos atacantes de equipos opuestos "tienen" la pelota
            # y tratan de disparar en direcciones contrarias simultáneamente.
            if ball_initialized:
                best_pid  = None
                best_dist = float('inf')
                for p in all_players:
                    if p.id not in players_initialized:
                        continue
                    dist = float(p.distance_to_ball(ball))
                    if dist < CAPTURE_CONFIRM_DISTANCE_PX:
                        ang = math.degrees(math.atan2(ball.y - p.y, ball.x - p.x))
                        err = abs((ang - p.angle + 180) % 360 - 180)
                        if err < 30 and dist < best_dist:
                            best_dist = dist
                            best_pid  = p.id
                for p in all_players:
                    if p.id not in players_initialized:
                        continue
                    dist = float(p.distance_to_ball(ball))
                    if p.id == best_pid:
                        p._has_ball = True
                    elif dist < CAPTURE_CONFIRM_DISTANCE_PX:
                        p._has_ball = False   # en rango pero no el más cercano → cede
                    elif dist > CAPTURE_CONFIRM_DISTANCE_PX * 2:
                        p._has_ball = False

            # --- Dribbler keepalive (robots activos de cada equipo) ---
            for bm, active in ((bm_red, active_red), (bm_blue, active_blue)):
                if not active:
                    continue
                for p in bm.team_players:
                    rid = p.id
                    if p._has_ball and not prev_has_ball.get(rid, False):
                        dribbler_pulse_phase[rid] = 'on'
                        dribbler_pulse_timer[rid] = now
                    if p._has_ball:
                        pulse_on_s  = DRIBBLER_PULSE_ON_MS  / 1000.0
                        pulse_off_s = DRIBBLER_PULSE_OFF_MS / 1000.0
                        if pulse_off_s <= 0:
                            if now - last_dribbler_keepalive[rid] >= DRIBBLER_KEEPALIVE:
                                rf.set_dribbler(rid + 1, DRIBBLER_HOLD_POWER)
                                last_dribbler_keepalive[rid] = now
                        else:
                            elapsed = now - dribbler_pulse_timer[rid]
                            if dribbler_pulse_phase[rid] == 'on':
                                if now - last_dribbler_keepalive[rid] >= DRIBBLER_KEEPALIVE:
                                    rf.set_dribbler(rid + 1, DRIBBLER_HOLD_POWER)
                                    last_dribbler_keepalive[rid] = now
                                if elapsed >= pulse_on_s:
                                    rf.set_dribbler(rid + 1, 0)
                                    dribbler_pulse_phase[rid] = 'off'
                                    dribbler_pulse_timer[rid] = now
                            else:
                                if elapsed >= pulse_off_s:
                                    rf.set_dribbler(rid + 1, DRIBBLER_HOLD_POWER)
                                    last_dribbler_keepalive[rid] = now
                                    dribbler_pulse_phase[rid] = 'on'
                                    dribbler_pulse_timer[rid] = now

            for p in all_players:
                prev_has_ball[p.id] = p._has_ball

            # --- Obstáculos comunes: todos los robots detectados ---
            _all_robot_data = [
                {'id': p.id, 'x': p.x, 'y': p.y}
                for p in all_players
                if p.id in players_initialized
            ]
            bm_red.command_manager.update_robot_data(_all_robot_data)
            bm_blue.command_manager.update_robot_data(_all_robot_data)
            if ball_initialized:
                bm_red.command_manager.update_ball_data(ball.x, ball.y)
                bm_blue.command_manager.update_ball_data(ball.x, ball.y)

            # --- Game context ---
            if players_initialized and ball_initialized:
                zona = max(0.0, min(2.0, (ball.x / FIELD_CAM.width) * 2.0))
                posesion_red  = 0.0 if any(p.has_ball() for p in bm_red.team_players)  else 0.5
                posesion_blue = 0.0 if any(p.has_ball() for p in bm_blue.team_players) else 0.5
                bm_red.update_game_context((posesion_red,  0.5, zona))
                bm_blue.update_game_context((posesion_blue, 0.5, 2.0 - zona))

            all_initialized = players_initialized.issuperset(set(all_ids))

            # --- Fase inicial: orientar → mover a posiciones de inicio ---
            if init_phase and all_initialized:
                # Subfase 1: todos giran al ángulo de ataque canónico
                if not init_orient_issued:
                    init_orient_issued = True
                    for bm in (bm_red, bm_blue):
                        ang = RESET_ANGLE[bm.team]
                        for p in bm.team_players:
                            bm.command_manager.rotate_robot_to(p.id, ang)
                    log.info("Fase inicial: orientando robots (rojo→%.0f°, azul→%.0f°)",
                             RESET_ANGLE['red'], RESET_ANGLE['blue'])

                # Subfase 2: cuando todos terminaron de girar → emitir movimiento
                if init_orient_issued and not init_orient_done:
                    orient_pending = any(
                        bm.command_manager.actions_in_progress.get(p.id, {}).get('type') == 'rotate'
                        for bm in (bm_red, bm_blue) for p in bm.team_players
                    )
                    if not orient_pending:
                        init_orient_done = True
                        for bm in (bm_red, bm_blue):
                            assignment = _assign_reset(bm.team_players, RESET_POS)
                            for p in bm.team_players:
                                rpos = assignment.get(p.id)
                                if rpos:
                                    ctrl = bm.command_manager.controllers.get(p.id)
                                    if ctrl:
                                        ctrl.max_linear_pwm_override = ctrl.compute_reset_pwm(p.id, RESET_MOVE_FACTOR)
                                    bm.command_manager.move_robot_to(p.id, rpos)
                        log.info("Fase inicial: orientación lista → moviendo a posiciones de inicio (RRT*)")

                # Ejecutar la subfase activa (rotación o movimiento)
                for bm in (bm_red, bm_blue):
                    angle_degs = {p.id: p.angle for p in bm.team_players}
                    for p in bm.team_players:
                        p.angle = math.radians(p.angle)
                    try:
                        bm.command_manager.execute_commands()
                    except Exception as e:
                        log.error("init execute_commands [%s]: %s", bm.team, e)
                    for p in bm.team_players:
                        p.angle = angle_degs[p.id]

            # --- Reset post-gol: orientar → mover a posiciones iniciales ---
            if goal_reset_active and all_initialized:
                # Subfase 1: todos giran al ángulo de ataque canónico
                if not reset_orient_issued:
                    reset_orient_issued = True
                    for bm in (bm_red, bm_blue):
                        ang = RESET_ANGLE[bm.team]
                        for p in bm.team_players:
                            bm.command_manager.rotate_robot_to(p.id, ang)
                    log.info("Reset: orientando robots (rojo→%.0f°, azul→%.0f°)",
                             RESET_ANGLE['red'], RESET_ANGLE['blue'])

                # Subfase 2: cuando todos terminaron de girar → emitir movimiento
                if reset_orient_issued and not reset_orient_done:
                    orient_pending = any(
                        bm.command_manager.actions_in_progress.get(p.id, {}).get('type') == 'rotate'
                        for bm in (bm_red, bm_blue) for p in bm.team_players
                    )
                    if not orient_pending:
                        reset_orient_done = True
                        for bm in (bm_red, bm_blue):
                            assignment = _assign_reset(bm.team_players, RESET_POS)
                            for p in bm.team_players:
                                rpos = assignment.get(p.id)
                                if rpos:
                                    ctrl = bm.command_manager.controllers.get(p.id)
                                    if ctrl:
                                        ctrl.max_linear_pwm_override = ctrl.compute_reset_pwm(p.id, RESET_MOVE_FACTOR)
                                    bm.command_manager.move_robot_to(p.id, rpos)
                        log.info("Reset: orientación lista → moviendo (%.0f%% pwm_max por robot, RRT* activo)",
                                 RESET_MOVE_FACTOR * 100)

                # Ejecutar la subfase activa (rotación o movimiento)
                for bm in (bm_red, bm_blue):
                    angle_degs = {p.id: p.angle for p in bm.team_players}
                    for p in bm.team_players:
                        p.angle = math.radians(p.angle)
                    try:
                        bm.command_manager.execute_commands()
                    except Exception as e:
                        log.error("reset execute_commands [%s]: %s", bm.team, e)
                    for p in bm.team_players:
                        p.angle = angle_degs[p.id]

            # --- BT tick y execute_commands (solo cuando el juego está activo) ---
            if all_initialized and ball_initialized and not ball_out_active and not goal_reset_active and not init_phase:
                if now - last_bt_tick >= BT_TICK_INTERVAL:
                    if active_red:
                        bm_red.update()
                    if active_blue:
                        bm_blue.update()
                    last_bt_tick = now

                for bm, active in ((bm_red, active_red), (bm_blue, active_blue)):
                    if not active:
                        continue
                    angle_degs = {p.id: p.angle for p in bm.team_players}
                    for p in bm.team_players:
                        p.angle = math.radians(p.angle)
                    try:
                        bm.command_manager.execute_commands()
                    except Exception as e:
                        log.error("execute_commands [%s]: %s", bm.team, e)
                    for p in bm.team_players:
                        p.angle = angle_degs[p.id]

            # --- Status periódico (2 Hz) ---
            if all_initialized and ball_initialized and now - last_status_log >= 0.5:
                for bm, opp_goal in ((bm_red, OPPONENT_GOAL_RED), (bm_blue, OPPONENT_GOAL_BLUE)):
                    for p in bm.team_players:
                        if p.id not in players_initialized:
                            continue
                        action_info = bm.command_manager.actions_in_progress.get(p.id)
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
                        bb = bm.blackboards.get(p.id)
                        bt_action = bb.last_action if bb else None
                        path_p  = bm.command_manager._current_paths.get(p.id, [])
                        wp_p    = bm.command_manager._current_wp_idx.get(p.id, 0)
                        rrt_len = max(0, len(path_p) - wp_p) if path_p else 0
                        n_obs   = len([r for r in _all_robot_data if r['id'] != p.id])
                        ang_ball = math.degrees(math.atan2(ball.y - p.y, ball.x - p.x))
                        err_ball = (ang_ball - p.angle + 180) % 360 - 180
                        ang_goal = math.degrees(math.atan2(
                            opp_goal[1] - p.y, opp_goal[0] - p.x))
                        err_goal = (ang_goal - p.angle + 180) % 360 - 180
                        robot_status_logger.update(
                            p.id,
                            state=bt_action or "unknown",
                            ang=p.angle,
                            pos=(int(p.x), int(p.y)),
                            tgt_pos=tgt_pos_v,
                            tgt_ang=tgt_ang_v,
                            dist=dist_v,
                            ball_err=err_ball,
                            goal_err=err_goal,
                            rrt_len=rrt_len,
                            n_obs=n_obs,
                        )
                        robot_status_logger.emit(p.id)
                last_status_log = now

            # --- Viz state ---
            if now - last_viz_time >= VIZ_INTERVAL:
                try:
                    players_viz = {}
                    for bm in (bm_red, bm_blue):
                        for p in bm.team_players:
                            bb = bm.blackboards.get(p.id)
                            last_action = bb.last_action if bb else None
                            action_info = bm.command_manager.actions_in_progress.get(p.id)
                            current_target = None
                            if action_info and 'target_pos' in action_info:
                                tp = action_info['target_pos']
                                try:
                                    current_target = (int(tp[0]), int(tp[1]))
                                except Exception:
                                    pass
                            players_viz[p.id] = {
                                'pos': (p.x, p.y) if p.id in players_initialized else None,
                                'angle_deg': p.angle if p.id in players_initialized else None,
                                'rol': 'atacante' if p.rol == ROL_ATACANTE else 'defensor',
                                'has_ball': p.has_ball(),
                                'last_action': str(last_action) if last_action else None,
                                'target': current_target,
                                'team': p.team,
                            }
                    viz_state_pipe.send({
                        'players': players_viz,
                        'ball_pos': (int(ball.x), int(ball.y)) if ball_initialized else None,
                        'active_red': active_red,
                        'active_blue': active_blue,
                        'robot_available': robot_available,
                        'ball_out': ball_out_active,
                        'goal_reset': goal_reset_active,
                        'init_phase': init_phase,
                        'timestamp': now,
                    })
                    last_viz_time = now
                except Exception:
                    pass

            time.sleep(0.01)

    finally:
        log.info("Decision 2v2 cerrando...")
        try:
            if rf.serial_manager.is_connected:
                for fid in [p.id + 1 for p in all_players]:
                    rf.stop_robot(fid)
                    rf.set_dribbler(fid, 0)
                rf.send_tablero(4)
                rf.send_tablero(5)
                time.sleep(0.15)
        except Exception:
            pass
        for proc in _planner_procs.values():
            proc.terminate()
            proc.join(timeout=1.0)
        try:
            bm_red.shutdown()
        except Exception:
            pass
        try:
            bm_blue.command_manager.rf_controller = None
            bm_blue.shutdown()
        except Exception:
            pass
        rf.shutdown()
        log.info("Decision 2v2 finalizada")
