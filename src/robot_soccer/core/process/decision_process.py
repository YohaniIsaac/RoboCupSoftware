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
    DRIBBLER_CAPTURE_POWER,
    DRIBBLER_HOLD_POWER,
    DRIBBLER_FW_ON_MS,
    DRIBBLER_FW_OFF_MS,
    DRIBBLER_FW_WDT_MS,
    PATH_PLANNING_OBSTACLE_CLEARANCE,
    RESET_POS,
    RESET_MOVE_FACTOR,
    RESET_ANGLE,
    KICKOFF_STAGING_OFFSET_PX,
    KICKOFF_BALL_CENTER_TOL_PX,
    BALL_FRESHNESS_TIMEOUT_S,
    BALL_OUT_DEBOUNCE_FRAMES,
    ROBOT_DETECTION_LOST_RAMPDOWN_S,
    KICK_POINT_ANGLE_OFFSET_DEG,
    POST_KICK_COOLDOWN_S,
    is_dribbler_enabled,
)

log = logging.getLogger(__name__)


# last_action del BT (blackboard) en los que el robot está POSICIONÁNDOSE/rotando, no
# capturando: el rodillo debe estar OFF aunque el estimador geométrico haya enclavado
# _has_ball (rodear la pelota con el rodillo encendido la expulsa al estar descentrada;
# ver _ball_cornered). Cubre dos familias:
#   - posicionamiento detrás de la pelota: blackboard.last_action que setea
#     _move_behind_ball en soccer_behaviors.
#   - cesión/intercepción: estados de hold_intercept_position (el robot cede la pelota
#     y se reubica como interceptor, NO la posee). Respaldo del apagado explícito del
#     latch (fix en hold_intercept_position + press del avance): suprime el rodillo
#     durante TODA la intercepción aunque _dribbler_on o _has_ball vengan sucios.
_POSITIONING_ACTIONS = frozenset({
    "move_behind_ball", "circle_ball", "retreating_from_ball",
    "move_to_intercept", "intercept_orienting", "intercept_hold",
})


def _pulse_dribbler(rf, p, now, last_ka_d, prev_d, keepalive_s, positioning=False):
    """Refresca el ESTADO del dribbler de un robot (la oscilación on/off la hace el firmware).

    Se engancha al AGARRAR (player._dribbler_on, durante el avance recto al contacto) o al
    tener posesión (_has_ball): usa DRIBBLER_CAPTURE_POWER agarrando y DRIBBLER_HOLD_POWER
    sosteniendo. Manda set_dribbler(power) a ritmo de keepalive_s mientras está enganchado
    (refresca el watchdog PROPIO del firmware), y un set_dribbler(0) EXPLÍCITO una sola vez al
    desenganchar (apagado inmediato, sin esperar el watchdog). Python ya NO oscila: el firmware
    pulsa el rodillo con el duty configurado (comando 'C' / EEPROM). NO se engancha durante el
    posicionamiento/rotación (_dribbler_on se limpia allí).
    """
    rid = p.id
    fw_id = rid + 1
    if not is_dribbler_enabled(rid):
        # Dribbler averiado: nunca energizar (proteger el componente). El robot captura igual por
        # avance+asentamiento+kick. set_dribbler también lo gatea, esto evita RF/eventos inútiles.
        return
    has_ball = getattr(p, '_has_ball', False)
    grabbing = getattr(p, '_dribbler_on', False) and not has_ball
    # positioning=True: el robot rodea/rota para posicionarse (no captura). Suprimir el rodillo
    # aunque _has_ball esté enclavado: girar con la pelota descentrada en el morro la expulsa.
    engaged  = (has_ball or getattr(p, '_dribbler_on', False)) and not positioning
    power = DRIBBLER_CAPTURE_POWER if grabbing else DRIBBLER_HOLD_POWER

    # Flancos de enganche (baja frecuencia, solo en transiciones): log de diagnóstico + acción
    # RF inmediata. En el flanco de BAJADA se manda set_dribbler(0) EXPLÍCITO — antes el apagado
    # dependía del watchdog, que el tráfico de 'M' del giro mantenía vivo y NO apagaba el rodillo.
    if engaged and not prev_d.get(rid, False):
        robot_status_logger.emit_event(
            rid, f"dribbler ON pwr={power} ({'captura' if grabbing else 'sosten'})")
        rf.set_dribbler(fw_id, power)   # engancha de inmediato
        last_ka_d[rid] = now
    elif not engaged and prev_d.get(rid, False):
        robot_status_logger.emit_event(rid, "dribbler OFF (fuera de enganche)")
        rf.set_dribbler(fw_id, 0)       # apagado EXPLÍCITO e inmediato
    prev_d[rid] = engaged

    if not engaged:
        return

    # Refresco del estado a ritmo de keepalive (alimenta el watchdog del firmware). NO oscila:
    # manda siempre 'power' (el firmware decide la fase on/off). El cambio captura↔sostén viaja
    # en el propio power.
    if now - last_ka_d[rid] >= keepalive_s:
        rf.set_dribbler(fw_id, power)
        last_ka_d[rid] = now


def _dribbler_view(p, positioning=False):
    """Estado del dribbler para el campo drb= del STATUS (snapshot a 2 Hz).

    Nivel ENGANCHADO (la micro-fase on/off de la oscilación la maneja el firmware, Python no
    la conoce): 'C{pwr}' agarrando (captura), 'H{pwr}' sosteniendo (con pelota), None
    desenganchado (rodillo parado), 'avr' averiado. Con positioning=True el rodillo está
    suprimido (mismo gate que _pulse_dribbler) → reportar parado (None).
    """
    if not is_dribbler_enabled(p.id):
        return 'avr'  # dribbler averiado (DRIBBLER_DISABLED_ROBOT_IDS): nunca se energiza
    if positioning:
        return None  # suprimido durante el posicionamiento/rotación (ver _pulse_dribbler)
    has_ball = getattr(p, '_has_ball', False)
    if not (has_ball or getattr(p, '_dribbler_on', False)):
        return None
    return f"H{DRIBBLER_HOLD_POWER}" if has_ball else f"C{DRIBBLER_CAPTURE_POWER}"


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

    # Dribbler: refresco de estado por robot (la oscilación on/off la hace el firmware).
    last_dribbler_keepalive = {rid: 0.0 for rid in robot_ids}
    DRIBBLER_KEEPALIVE = 0.08     # 80ms — refresca el watchdog del firmware (wdt 150ms)
    prev_dribbler_engaged = {rid: False for rid in robot_ids}
    _last_telem_poll = 0.0        # D1: poll de telemetría del firmware (~1Hz)
    TELEM_POLL_S = 1.0

    # Enviar al firmware la config de oscilación del dribbler (persiste en EEPROM del robot).
    _rf_cfg = behavior_manager.command_manager.rf_controller
    if robot_available and _rf_cfg is not None:
        for rid in robot_ids:
            _rf_cfg.set_dribbler_config(
                rid + 1, DRIBBLER_FW_ON_MS, DRIBBLER_FW_OFF_MS, DRIBBLER_FW_WDT_MS)

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
                                player_map[rid].last_seen_t = now
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
                                player_map[robot_id].last_seen_t = now
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
                    if now - getattr(p, '_last_kick_time', 0.0) < POST_KICK_COOLDOWN_S:
                        # Cooldown post-kick: la posesión terminó al disparar. No re-derivar
                        # has_ball=True por proximidad mientras la cámara aún ve la pelota
                        # pegada (1-2 frames de lag) o el dribbler se re-engancha (H{pwr})
                        # sobre una pelota ya disparada. Ver _pulse_dribbler / kick_immediately.
                        p._has_ball = False
                    elif dist < CAPTURE_CONFIRM_DISTANCE_PX:
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
            ball_dist_px      = None
            kick_lat_px       = None
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
                # Diagnóstico de captura: distancia a la pelota y desvío lateral con signo
                # de la pelota respecto al eje de la nariz (solenoide a player.angle +
                # KICK_POINT_ANGLE_OFFSET_DEG). Mismo signo que Dball (+ = pelota CCW de la
                # nariz, − = CW). Un signo persistente delata un sesgo sistemático
                # (cámara/montaje del marcador), no el lazo de alineación.
                ball_dist_px = math.hypot(ball.x - primary.x, ball.y - primary.y)
                _nose_rad = math.radians(primary.angle + KICK_POINT_ANGLE_OFFSET_DEG)
                kick_lat_px = ((ball.x - primary.x) * (-math.sin(_nose_rad))
                               + (ball.y - primary.y) * math.cos(_nose_rad))

            # --- Actualizar contexto del juego ---
            if players_initialized and ball_initialized:
                posesion = 0.0 if any(p.has_ball() for p in players) else 0.5
                proximidad = 0.5   # siempre "cerca" con robots del mismo equipo sin rivales
                zona = (ball.x / FIELD_CAM.width) * 2.0
                zona = max(0.0, min(2.0, zona))
                behavior_manager.update_game_context((posesion, proximidad, zona))

            # --- Dribbler: pulso de captura/sostén (por robot) ---
            if behavior_active and robot_available:
                rf = behavior_manager.command_manager.rf_controller
                for p in behavior_manager.team_players:
                    _bb_p = behavior_manager.blackboards.get(p.id)
                    _positioning = bool(_bb_p and _bb_p.last_action in _POSITIONING_ACTIONS)
                    _pulse_dribbler(rf, p, now, last_dribbler_keepalive,
                                    prev_dribbler_engaged, DRIBBLER_KEEPALIVE,
                                    positioning=_positioning)

                # Telemetría del firmware (D1): leer lo que el robot adjuntó al ACK (observación).
                if now - _last_telem_poll >= TELEM_POLL_S:
                    _last_telem_poll = now
                    _tlm, _terr = rf.poll_telemetry()
                    for _t in _tlm:
                        log.info("[TELEM] R%s cfg=%s/%s/%s eng=%s pwr=%s ev=%s m=%s d=%s",
                                 _t.get('robot'), _t.get('on'), _t.get('off'), _t.get('wdt'),
                                 _t.get('eng'), _t.get('pwr'), _t.get('ev'), _t.get('m'), _t.get('d'))
                    if _terr:
                        log.warning("[TELEM] %d ERROR(es) de entrega RF desde el ultimo poll", _terr)

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
                            ball_dist=ball_dist_px if p.id == robot_id else None,
                            kick_lat=kick_lat_px if p.id == robot_id else None,
                            dribbler=_dribbler_view(
                                p, positioning=bt_action_p in _POSITIONING_ACTIONS),
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
                        path_p, wp_idx_p = behavior_manager.command_manager.get_path_state(p.id)
                        players_viz[p.id] = {
                            'pos': (p.x, p.y) if p.id in players_initialized else None,
                            'angle_deg': p.angle if p.id in players_initialized else None,
                            'rol': 'atacante' if p.rol == ROL_ATACANTE else 'defensor',
                            'has_ball': p.has_ball(),
                            'last_action': str(last_action_p) if last_action_p else None,
                            'target': current_target_p,
                            'path': path_p,
                            'wp_idx': wp_idx_p,
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


def _segments_cross(a1, a2, b1, b2):
    """True si los segmentos a1→a2 y b1→b2 se intersectan (test de orientación CCW)."""
    def _ccw(p, q, r):
        return (r[1] - p[1]) * (q[0] - p[0]) - (q[1] - p[1]) * (r[0] - p[0])
    d1 = _ccw(b1, b2, a1)
    d2 = _ccw(b1, b2, a2)
    d3 = _ccw(a1, a2, b1)
    d4 = _ccw(a1, a2, b2)
    return ((d1 > 0) != (d2 > 0)) and ((d3 > 0) != (d4 > 0))


def _assign_reset(teams, pos_dict):
    """Asigna posiciones de reset a TODOS los robots (ambos equipos) globalmente.

    Cada robot solo recibe posiciones de su propio equipo (pos_dict define a qué
    equipo pertenece cada posición vía el id), pero la permutación se elige
    considerando los dos equipos a la vez: se minimiza primero el número de
    cruces entre las trayectorias rectas robot→posición de todos los robots, y
    como desempate la distancia total de viaje.

    Evita el escenario donde dos robots — incluso de equipos distintos — quedan
    con trayectorias enfrentadas: RRT* falla (el goal está ocupado por el otro
    robot), el control cae a PID directo sin evasión y terminan chocando.

    Args:
        teams: lista de listas de Player, una por equipo.
        pos_dict: {player_id: (x, y)} posiciones de reset.

    Returns:
        dict: {player_id: (x, y)} para todos los robots de todos los equipos.
    """
    from itertools import permutations as _perms, product as _product
    team_options = []
    for players in teams:
        ids = [p.id for p in players]
        positions = [pos_dict[pid] for pid in ids]
        team_options.append([
            {ids[i]: positions[perm[i]] for i in range(len(ids))}
            for perm in _perms(range(len(ids)))
        ])

    all_players = [p for players in teams for p in players]
    best, best_key = None, None
    for combo in _product(*team_options):
        assignment = {}
        for team_part in combo:
            assignment.update(team_part)
        segments = [((p.x, p.y), assignment[p.id]) for p in all_players]
        crossings = sum(
            1
            for i in range(len(segments))
            for j in range(i + 1, len(segments))
            if _segments_cross(segments[i][0], segments[i][1],
                               segments[j][0], segments[j][1])
        )
        total_dist = sum(
            math.hypot(p.x - assignment[p.id][0], p.y - assignment[p.id][1])
            for p in all_players
        )
        key = (crossings, total_dist)
        if best_key is None or key < best_key:
            best_key, best = key, assignment
    if best_key and best_key[0] > 0:
        log.info("Asignación de reset: %d cruce(s) de trayectoria inevitable(s)",
                 best_key[0])
    return best


def _kickoff_targets(scoring_team, team_red_ids, team_blue_ids):
    """Posiciones de saque estilo SSL tras un gol.

    El equipo que RECIBIÓ el gol (conceding) toma el saque, igual que en fútbol:
    uno de sus robots se ubica detrás de la pelota (centro del campo) listo para
    atacar; el otro queda en su media-formación (RESET_POS). El equipo que anotó
    se repliega a RESET_POS (ya en su mitad, fuera del círculo central).

    Qué robot concreto del equipo que saca va al staging lo decide _assign_reset
    (minimiza cruces de trayectoria y distancia).

    Args:
        scoring_team: 'red' | 'blue' — equipo que anotó (None → reset simétrico).
        team_red_ids, team_blue_ids: list[int] — ids de cada equipo.

    Returns:
        (targets, conceding): dict {player_id: (x, y)} para los 4 robots y el
        equipo que toma el saque ('red'|'blue'|None).
    """
    targets = dict(RESET_POS)
    if scoring_team not in ('red', 'blue'):
        return targets, None  # sin info de quién anotó → formación simétrica
    conceding = 'blue' if scoring_team == 'red' else 'red'
    cx, cy = FIELD_CAM.center
    # Sacador detrás de la pelota según la dirección de ataque del que saca:
    # rojo ataca +x (arco derecho) → detrás = lado -x; azul ataca -x → detrás = lado +x.
    if conceding == 'red':
        staging    = (cx - KICKOFF_STAGING_OFFSET_PX, cy)
        kicker_ids = team_red_ids
    else:
        staging    = (cx + KICKOFF_STAGING_OFFSET_PX, cy)
        kicker_ids = team_blue_ids
    if kicker_ids:
        targets[kicker_ids[0]] = staging
    return targets, conceding


def decision_process_2v2(
    perception_pipe,
    viz_state_pipe,
    keyboard_pipe,
    team_red_ids,
    team_blue_ids,
    serial_port: str,
    game_active,
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

    # Config de oscilación del dribbler al firmware (persiste en EEPROM). El robot oscila el
    # rodillo con este duty de forma autónoma; Python ya no oscila.
    for _rid in all_ids:
        rf.set_dribbler_config(
            _rid + 1, DRIBBLER_FW_ON_MS, DRIBBLER_FW_OFF_MS, DRIBBLER_FW_WDT_MS)

    # --- Crear jugadores ---
    players_red  = [Player(rid, 0, 0, 0.0, team='red')  for rid in team_red_ids]
    players_blue = [Player(rid, 0, 0, 0.0, team='blue') for rid in team_blue_ids]
    all_players  = players_red + players_blue
    player_map   = {p.id: p for p in all_players}

    # Atacante inicial: preferir un robot con dribbler sano (los averiados atacan solo si no
    # hay alternativa). Evita arrancar con el robot averiado de atacante y esperar un cooldown
    # de arbitraje para corregirlo. Ver DRIBBLER_DISABLED_ROBOT_IDS.
    for team in (players_red, players_blue):
        attacker = next((p for p in team if is_dribbler_enabled(p.id)), team[0])
        for p in team:
            p.set_rol(ROL_ATACANTE if p is attacker else ROL_DEFENSIVO)

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
    ball_last_seen_t    = 0.0     # timestamp de la última detección de pelota
    active_red          = False
    active_blue         = False
    running             = True
    ball_out_active     = False   # pelota fuera — robots congelados, BT pausado
    ball_out_frames     = 0       # frames consecutivos con pelota fuera (histéresis)
    ball_in_frames      = 0       # frames consecutivos con pelota en juego (debounce)
    goal_reset_active   = False   # pausa post-gol — robots navegan a reset positions
    reset_move_issued   = False   # move emitido (reset post-gol)
    reset_robot_phase   = {}      # id → 'moving' | 'orienting' | 'done'
    scored_team         = None    # equipo que anotó el último gol ('red'|'blue')
    kickoff_conceding   = None    # equipo que toma el saque (recibió el gol)
    kickoff_ball_centered = False # pelota repuesta cerca del centro del campo
    kickoff_ready       = False   # robots detectados en posición + pelota al centro
    pre_init            = True    # esperando ESPACIO1: robots quietos, aún no organizan
    init_phase          = False   # True desde ESPACIO1: robots organizándose
    init_ready          = False   # True cuando robots llegaron y están orientados
    init_move_issued    = False   # move emitido (fase inicial)
    init_robot_phase    = {}      # id → 'moving' | 'orienting' | 'done'

    VIZ_INTERVAL    = 0.04
    BT_TICK_INTERVAL = 0.1
    DRIBBLER_KEEPALIVE = 0.08
    OPPONENT_GOAL_RED  = FIELD_CAM.goal_right_center
    OPPONENT_GOAL_BLUE = FIELD_CAM.goal_left_center

    last_viz_time   = 0.0
    last_bt_tick    = 0.0
    last_status_log = 0.0

    last_dribbler_keepalive = {rid: 0.0 for rid in all_ids}
    prev_dribbler_engaged   = {rid: False for rid in all_ids}
    _last_telem_poll = 0.0        # D1: poll de telemetría del firmware (~1Hz)
    TELEM_POLL_S = 1.0
    _telem_outbox = []            # D1b: telemetría polleada pendiente de mandar al recorder (timeline)

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
                            player_map[rid].last_seen_t = now
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
                        ball_last_seen_t = time.time()

                    # Pelota fuera: congelar robots cuando sale de límites.
                    # Durante init_phase o goal_reset_active el juego no está RUNNING:
                    # ignorar ball_out igual que en SSL (árbitro maneja la pelota).
                    ball_out_new = data.get('ball_out', False)
                    game_running = not init_phase and not goal_reset_active
                    # Histéresis simétrica: BALL_OUT_DEBOUNCE_FRAMES en ambos
                    # sentidos. Antes la activación tenía 3 frames pero la
                    # desactivación era instantánea → flicker constante con
                    # detección al 30%.
                    if game_running:
                        if ball_out_new:
                            ball_out_frames += 1
                            ball_in_frames   = 0
                        else:
                            ball_in_frames += 1
                            ball_out_frames  = 0
                    else:
                        ball_out_frames = ball_in_frames = 0

                    if (ball_out_frames >= BALL_OUT_DEBOUNCE_FRAMES
                            and not ball_out_active and game_running):
                        ball_out_active = True
                        log.info("PELOTA FUERA: robots congelados")
                        for fid in [p.id + 1 for p in all_players]:
                            rf.set_motors(fid, 0, 0)
                        for bm in (bm_red, bm_blue):
                            for p in bm.team_players:
                                bm.command_manager.actions_in_progress.pop(p.id, None)
                    elif (ball_in_frames >= BALL_OUT_DEBOUNCE_FRAMES
                          and ball_out_active):
                        ball_out_active = False
                        log.info("PELOTA EN JUEGO: reanudando")
                        # Reset del estado táctico arrastrado durante la pausa:
                        # un atacante que venía cediendo reanudaría cediendo y
                        # provocaría el deadlock de cesión mutua. Empezar limpio.
                        for bm in (bm_red, bm_blue):
                            for p in bm.team_players:
                                _bb = bm.blackboards.get(p.id)
                                if _bb is not None:
                                    _bb._yielding_to_rival      = False
                                    _bb._yielding_last_change_t = 0.0
                                    _bb._uncontested_since      = None
                    elif ball_out_active and not game_running:
                        # Transición a init/reset mientras ball_out estaba activo → limpiar
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
                        game_active.value   = 0
                        reset_move_issued   = False
                        reset_robot_phase   = {}
                        scored_team         = cmd.get('team')
                        kickoff_conceding   = ('blue' if scored_team == 'red'
                                               else 'red' if scored_team == 'blue'
                                               else None)
                        kickoff_ready       = False
                        kickoff_ball_centered = False
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
                        if pre_init:
                            pre_init = False
                            init_phase = True
                            for bm in (bm_red, bm_blue):
                                bm.command_manager.replan_cooldown_s_override = 2.0
                            log.info("┌─────────────────────────────────────────────────────┐")
                            log.info("│ ► INICIANDO ORGANIZACIÓN DE ROBOTS                   │")
                            log.info("└─────────────────────────────────────────────────────┘")
                        elif init_phase and not init_ready:
                            log.info("Robots aún organizándose, espera que lleguen a sus posiciones...")
                        elif init_phase and init_ready:
                            if not ball_initialized:
                                log.info("Partido no puede iniciar: pelota no detectada — coloca la pelota en el campo")
                            else:
                                init_phase  = False
                                init_ready  = False
                                init_move_issued = False
                                init_robot_phase = {}
                                active_red  = active_blue = True
                                game_active.value = 1
                                for bm in (bm_red, bm_blue):
                                    for p in bm.team_players:
                                        ctrl = bm.command_manager.controllers.get(p.id)
                                        if ctrl:
                                            ctrl.max_linear_pwm_override = None
                                            ctrl.linear_pwm_cap = None
                                        bm.command_manager.actions_in_progress.pop(p.id, None)
                                    bm.command_manager.replan_cooldown_s_override = None
                                log.info("┌─────────────────────────────────────────────────────┐")
                                log.info("│ ► COMIENZA PARTIDO (robots en posición)              │")
                                log.info("└─────────────────────────────────────────────────────┘")
                        elif goal_reset_active:
                            if not kickoff_ready:
                                if not kickoff_ball_centered:
                                    log.info("Saque aún no listo: coloca la pelota en el centro del campo")
                                else:
                                    pendientes = [p.id for bm in (bm_red, bm_blue)
                                                  for p in bm.team_players
                                                  if reset_robot_phase.get(p.id) != 'done']
                                    no_detect = [pid for pid in pendientes
                                                 if now - player_map[pid].last_seen_t
                                                 >= ROBOT_DETECTION_LOST_RAMPDOWN_S]
                                    if no_detect:
                                        log.info("Saque aún no listo: robot(s) %s sin detección — "
                                                 "reubícalos manualmente", no_detect)
                                    else:
                                        log.info("Saque aún no listo: robots posicionándose %s", pendientes)
                            else:
                                goal_reset_active = False
                                game_active.value = 1
                                active_red = active_blue = True
                                for bm in (bm_red, bm_blue):
                                    for p in bm.team_players:
                                        ctrl = bm.command_manager.controllers.get(p.id)
                                        if ctrl:
                                            ctrl.max_linear_pwm_override = None
                                            ctrl.linear_pwm_cap = None
                                        bm.command_manager.actions_in_progress.pop(p.id, None)
                                        # Limpiar estado táctico arrastrado del pre-gol para que
                                        # el sacador no ceda al instante (mismo reset que PELOTA EN JUEGO).
                                        _bb = bm.blackboards.get(p.id)
                                        if _bb is not None:
                                            _bb._yielding_to_rival      = False
                                            _bb._yielding_last_change_t = 0.0
                                            _bb._uncontested_since      = None
                                log.info("Reanudando partido tras gol (saque para %s)",
                                         kickoff_conceding or "?")
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
                    # Cooldown post-kick: un robot que acaba de disparar ya no posee la
                    # pelota aunque la cámara la siga viendo pegada (1-2 frames de lag) → no
                    # compite por best_pid ni re-engancha el dribbler. Ver kick_immediately.
                    if now - getattr(p, '_last_kick_time', 0.0) < POST_KICK_COOLDOWN_S:
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
                    if now - getattr(p, '_last_kick_time', 0.0) < POST_KICK_COOLDOWN_S:
                        p._has_ball = False   # acaba de patear: soltar posesión y dribbler
                    elif p.id == best_pid:
                        p._has_ball = True
                    elif dist < CAPTURE_CONFIRM_DISTANCE_PX:
                        p._has_ball = False   # en rango pero no el más cercano → cede
                    elif dist > CAPTURE_CONFIRM_DISTANCE_PX * 2:
                        p._has_ball = False

            # --- Dribbler: pulso de captura/sostén (robots activos de cada equipo) ---
            for bm, active in ((bm_red, active_red), (bm_blue, active_blue)):
                if not active:
                    continue
                for p in bm.team_players:
                    _bb_p = bm.blackboards.get(p.id)
                    _positioning = bool(_bb_p and _bb_p.last_action in _POSITIONING_ACTIONS)
                    _pulse_dribbler(rf, p, now, last_dribbler_keepalive,
                                    prev_dribbler_engaged, DRIBBLER_KEEPALIVE,
                                    positioning=_positioning)

            # Telemetría del firmware (D1): leer lo que los robots adjuntaron al ACK (observación).
            if now - _last_telem_poll >= TELEM_POLL_S:
                _last_telem_poll = now
                _tlm, _terr = rf.poll_telemetry()
                for _t in _tlm:
                    log.info("[TELEM] R%s cfg=%s/%s/%s eng=%s pwr=%s ev=%s m=%s d=%s",
                             _t.get('robot'), _t.get('on'), _t.get('off'), _t.get('wdt'),
                             _t.get('eng'), _t.get('pwr'), _t.get('ev'), _t.get('m'), _t.get('d'))
                if _tlm:
                    _telem_outbox.extend(_tlm)   # D1b: adjuntar al próximo payload de viz (timeline)
                if _terr:
                    log.warning("[TELEM] %d ERROR(es) de entrega RF desde el ultimo poll", _terr)

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

            # --- Fase inicial: mover directo a posiciones, orientar por robot al llegar ---
            if init_phase and all_initialized:
                # Paso 1: emitir movimiento inmediatamente (sin pre-orientación)
                if not init_move_issued:
                    init_move_issued = True
                    init_robot_phase = {
                        p.id: 'moving'
                        for bm in (bm_red, bm_blue) for p in bm.team_players
                    }
                    assignment = _assign_reset(
                        [bm_red.team_players, bm_blue.team_players], RESET_POS)
                    for bm in (bm_red, bm_blue):
                        for p in bm.team_players:
                            rpos = assignment.get(p.id)
                            if rpos:
                                ctrl = bm.command_manager.controllers.get(p.id)
                                if ctrl:
                                    ctrl.linear_pwm_cap = ctrl.compute_reset_pwm(p.id, RESET_MOVE_FACTOR)
                                bm.command_manager.move_robot_to(p.id, rpos)
                    log.info("Fase inicial: moviendo robots a posiciones de inicio (RRT*)")

                # Paso 2: por robot — cuando llega, orientar hacia la pelota; cuando orienta, marcar listo
                if init_move_issued:
                    ball_fresh = (ball_initialized
                                  and time.time() - ball_last_seen_t < BALL_FRESHNESS_TIMEOUT_S)
                    for bm in (bm_red, bm_blue):
                        for p in bm.team_players:
                            phase = init_robot_phase.get(p.id)
                            action = bm.command_manager.actions_in_progress.get(p.id)
                            if phase == 'moving' and action is None:
                                # Sólo apuntamos a la pelota si la detección es
                                # fresca; con detección stale (e.g. 7%) la
                                # posición puede ser de varios segundos atrás.
                                if ball_fresh:
                                    angle = math.degrees(math.atan2(ball.y - p.y, ball.x - p.x))
                                    src = "pelota"
                                else:
                                    angle = RESET_ANGLE[bm.team]
                                    src = "canónico (pelota stale)"
                                bm.command_manager.rotate_robot_to(p.id, angle)
                                init_robot_phase[p.id] = 'orienting'
                                log.info("Robot %d en posición → orientando a %s (%.1f°)",
                                         p.id, src, angle)
                            elif phase == 'orienting' and action is None:
                                init_robot_phase[p.id] = 'done'
                                log.info("Robot %d listo", p.id)

                # Paso 3: todos listos → init_ready
                if init_move_issued and not init_ready:
                    if all(
                        init_robot_phase.get(p.id) == 'done'
                        for bm in (bm_red, bm_blue) for p in bm.team_players
                    ):
                        init_ready = True
                        log.info("Robots listos en posiciones de inicio. Presiona ESPACIO para comenzar el partido.")

                # Ejecutar subfase activa
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

            # --- Reset post-gol: mover directo a posiciones, orientar por robot al llegar ---
            if goal_reset_active and all_initialized:
                # Paso 1: emitir movimiento inmediatamente (sin pre-orientación)
                if not reset_move_issued:
                    reset_move_issued = True
                    reset_robot_phase = {
                        p.id: 'moving'
                        for bm in (bm_red, bm_blue) for p in bm.team_players
                    }
                    kickoff_targets, kickoff_conceding = _kickoff_targets(
                        scored_team, team_red_ids, team_blue_ids)
                    assignment = _assign_reset(
                        [bm_red.team_players, bm_blue.team_players], kickoff_targets)
                    for bm in (bm_red, bm_blue):
                        for p in bm.team_players:
                            rpos = assignment.get(p.id)
                            if rpos:
                                ctrl = bm.command_manager.controllers.get(p.id)
                                if ctrl:
                                    ctrl.linear_pwm_cap = ctrl.compute_reset_pwm(p.id, RESET_MOVE_FACTOR)
                                bm.command_manager.move_robot_to(p.id, rpos)
                    log.info("Saque tras gol: equipo %s saca desde el centro; %s se repliega "
                             "(%.0f%% pwm_max, RRT*)",
                             kickoff_conceding or "?", scored_team or "?", RESET_MOVE_FACTOR * 100)

                # Paso 2: por robot — al llegar, orientar a su dirección de ataque
                # (RESET_ANGLE). Con la pelota repuesta al centro, el sacador (detrás
                # del centro) queda mirando a la pelota; orientar al ángulo canónico
                # evita apuntar a una pelota que aún esté en la esquina antes del saque.
                if reset_move_issued:
                    for bm in (bm_red, bm_blue):
                        for p in bm.team_players:
                            phase = reset_robot_phase.get(p.id)
                            action = bm.command_manager.actions_in_progress.get(p.id)
                            if phase == 'moving' and action is None:
                                angle = RESET_ANGLE[bm.team]
                                bm.command_manager.rotate_robot_to(p.id, angle)
                                reset_robot_phase[p.id] = 'orienting'
                                log.info("Saque robot %d en posición → orientando (%.1f°)", p.id, angle)
                            elif phase == 'orienting' and action is None:
                                reset_robot_phase[p.id] = 'done'

                # Gate de saque listo: pelota repuesta al centro + todos los robots
                # ACTUALMENTE detectados en posición. Un robot perdido (detección stale)
                # se excluye para no colgar el saque; el operador lo reubica a mano.
                cx, cy = FIELD_CAM.center
                kickoff_ball_centered = (
                    ball_initialized
                    and math.hypot(ball.x - cx, ball.y - cy) <= KICKOFF_BALL_CENTER_TOL_PX
                )
                _present = [p for bm in (bm_red, bm_blue) for p in bm.team_players
                            if now - p.last_seen_t < ROBOT_DETECTION_LOST_RAMPDOWN_S]
                kickoff_ready = bool(_present) and kickoff_ball_centered and all(
                    reset_robot_phase.get(p.id) == 'done' for p in _present)

                # Ejecutar subfase activa
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
                        # Diagnóstico de captura (ver path 1-robot): distancia a la pelota y
                        # desvío lateral con signo respecto a la nariz (mismo signo que Dball).
                        ball_dist = math.hypot(ball.x - p.x, ball.y - p.y)
                        _nose_rad = math.radians(p.angle + KICK_POINT_ANGLE_OFFSET_DEG)
                        kick_lat = ((ball.x - p.x) * (-math.sin(_nose_rad))
                                    + (ball.y - p.y) * math.cos(_nose_rad))
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
                            ball_dist=ball_dist,
                            kick_lat=kick_lat,
                            dribbler=_dribbler_view(
                                p, positioning=bt_action in _POSITIONING_ACTIONS),
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
                            path, wp_idx = bm.command_manager.get_path_state(p.id)
                            ctrl = bm.command_manager.controllers.get(p.id)
                            lp, rp = ctrl._last_pwm.get(p.id, (None, None)) if ctrl else (None, None)
                            players_viz[p.id] = {
                                'pos': (p.x, p.y) if p.id in players_initialized else None,
                                'angle_deg': p.angle if p.id in players_initialized else None,
                                'rol': 'atacante' if p.rol == ROL_ATACANTE else 'defensor',
                                'has_ball': p.has_ball(),
                                'last_action': str(last_action) if last_action else None,
                                'target': current_target,
                                'path': path,
                                'wp_idx': wp_idx,
                                'team': p.team,
                                'left_pwm': lp,
                                'right_pwm': rp,
                                # Estado del rodillo (mismo valor que el campo drb= del STATUS):
                                # 'C{pwr}'/'H{pwr}' enganchado, 'off' enganchado en ventana de
                                # pulso apagado, 'avr' averiado, None desenganchado. Observación
                                # pura: _dribbler_view es O(1) sobre flags ya mantenidos por el
                                # loop de control (no agrega cómputo ni sincronización).
                                'dribbler': _dribbler_view(
                                    p, positioning=bool(last_action in _POSITIONING_ACTIONS)),
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
                        'pre_init': pre_init,
                        'init_ready': init_ready,
                        'ball_detected': ball_initialized,
                        'goal_scored_team': scored_team,
                        'kickoff_conceding': kickoff_conceding,
                        'kickoff_ball_centered': kickoff_ball_centered,
                        'kickoff_ready': kickoff_ready,
                        # D1b: telemetría del firmware polleada desde el último envío (el recorder
                        # la vuelca al timeline). Lista vacía la mayoría de los frames.
                        'telemetry': _telem_outbox,
                        'timestamp': now,
                    })
                    last_viz_time = now
                    _telem_outbox = []   # enviado: vaciar solo tras un send exitoso (sin pérdida)
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
