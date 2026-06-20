#!/usr/bin/env python3
"""Test de integración: partido 2v2 completo con dos equipos autónomos.

PASO 7 del plan de integración. Valida el partido completo 2vs2:
  - Equipo rojo (robots 0, 1): ataca arco derecho
  - Equipo azul (robots 2, 3): ataca arco izquierdo
  - Ambos equipos con BehaviorManager independiente, RFController compartido
  - Detección de gol en ambos arcos con marcador y pausa automática

Pipeline:
  Cámara → Perspectiva → Detección (4 robots ArUco + pelota HSV + zonas de gol)
         → Decision2v2 (BM-rojo + BM-azul, serial único)
         → PID → RF → Motores

Arquitectura: 3 procesos
  1. Percepción:   4 ArUcos + pelota + detección de gol
  2. Decision 2v2: BM-rojo y BM-azul en el mismo proceso, un solo serial
  3. Visualización: campo con 4 robots, marcador, overlays de gol

Uso:
    python scripts/integration/test_behavior_2v2.py
    python scripts/integration/test_behavior_2v2.py --serial-port /dev/ttyUSB0

Controles (ventana OpenCV):
    ESPACIO : Activar/pausar AMBOS equipos
    R       : Activar/pausar solo equipo rojo   (debug: verificar un equipo)
    B       : Activar/pausar solo equipo azul   (debug: verificar un equipo)
    K       : Mostrar/ocultar overlay del kick_point (lectura de config.py)
    ESC     : Salir

Overlay del kick_point:
    Con K se muestra para cada robot el punto desplazado del marker ArUco en
    la dirección del heading, donde se asume que impacta el solenoide
    (KICK_POINT_OFFSET_PX), junto con el círculo de tolerancia
    (KICK_POINT_TOLERANCE_PX). Verde si la pelota cae dentro de la zona,
    amarillo en caso contrario. Útil para verificar el alineamiento durante
    un partido real. La calibración interactiva del valor vive en
    scripts/calibrate_behavior_thresholds.py.

Métricas:
    Al salir (ESC) se guarda LOG/match_2v2_<ts>.json con las métricas del
    partido: tiempo efectivo de juego, goles, embudo de secuencias de ataque
    (posicionamiento → avance → contacto → disparo), recuperaciones del
    defensor, cambios de rol, repertorio de acciones del BT, error de
    seguimiento de ruta, cadencia del lazo de decisión y tasas de detección
    de percepción durante el partido. La captura observa los payloads que ya
    fluyen por los pipes (sin tocar src/).
"""

import sys
import math
import time
import logging
import argparse
import statistics
import multiprocessing
from multiprocessing import Value, shared_memory
from pathlib import Path

ROOT_DIR = Path(__file__).parent.parent.parent
sys.path.insert(0, str(ROOT_DIR / "src"))
sys.path.insert(0, str(ROOT_DIR / "scripts"))

from robot_soccer.config import (
    CAMERA_PERSPECTIVE_HEIGHT, CAMERA_PERSPECTIVE_WIDTH,
    CAMERA_PERSPECTIVE_ENABLED, CAMERA_PERSPECTIVE_SRC_POINTS,
    RANGO_COLOR_NARANJO,
    FIELD_CAM,
    BALL_OUT_MARGIN_PX,
    RESET_POS,
    KICK_POINT_OFFSET_PX,
    KICK_POINT_TOLERANCE_PX,
    FIELD_PHYSICAL_WIDTH_CM,
    FIELD_PHYSICAL_HEIGHT_CM,
)
from robot_soccer.utils.camera_utils import get_camera_index
from robot_soccer.core.process.decision_process import decision_process_2v2
from metrics.metrics_capture import save_metrics

logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s [%(processName)-12s] %(levelname)-8s %(message)s',
    datefmt='%H:%M:%S'
)
log = logging.getLogger(__name__)

SHM_NAME       = "robot_soccer_2v2"
BALL_RADIUS_PX = 15
GOAL_COOLDOWN_S  = 5.0

# Robots por equipo
TEAM_RED_IDS  = [0, 1]
TEAM_BLUE_IDS = [2, 3]
ALL_IDS       = TEAM_RED_IDS + TEAM_BLUE_IDS

# Colores (BGR): rojo oscuro, rojo claro, azul oscuro, azul claro
ROBOT_COLORS = {
    0: (60,  60, 220),
    1: (80,  80, 200),
    2: (220, 80,  60),
    3: (200, 100, 80),
}
TEAM_COLOR = {'red': (60, 60, 220), 'blue': (220, 80, 60)}
ROL_LABELS = {'atacante': 'A', 'defensor': 'D'}

# ---------------------------------------------------------------------------
# Timeline log: canales y cadencia
# Poner un canal en False lo excluye completamente del timeline.
# _SNAP_EVERY_N: un 'snap' cada N llamadas a on_decision (~1 snap/s a 25 Hz).
# ---------------------------------------------------------------------------
LOG_CHANNELS: dict = {
    'bt_state':       True,   # cambios de acción BT:  {t, ev:'bt',  r, team, bt}
    'rol_change':     True,   # cambios de rol:        {t, ev:'rol', r, team, rol}
    'game_state':     True,   # flags partido:         {t, ev:'gs',  s, v}
    'position':       True,   # pos en snap: p=[x,y]
    'angle_in_snap':  True,   # ángulo en snap: a=float
    'pwm':            False,  # PWM en snap: L=int, R=int  (debug controlador)
    'target_in_snap': False,  # target en snap: tgt=[x,y]
    'path_in_snap':   False,  # waypoints en snap: path=[[x,y],...]
}
_SNAP_EVERY_N  = 25    # ~1 snap/s a 25 Hz de decisión
_TIMELINE_CAP  = 2000


# =============================================================================
# Captura de métricas del partido (observación pura sobre los pipes)
# =============================================================================

# Acciones del ciclo de ataque directo, por etapa del embudo:
# 1=posicionamiento, 2=avance al contacto, 3=contacto confirmado, 4=disparo.
_ATTACK_STAGE = {
    'retreating_from_ball': 1,
    'circle_ball':          1,
    'move_behind_ball':     1,
    'behind_ball_ready':    1,
    'advancing_to_contact': 2,
    'settling_contact':     2,
    'contact_confirmed':    3,
    'kick_immediately':     4,
}

_DEFENDER_CAPTURE_ACTIONS = {'capturing_ball', 'ball_captured_with_motor'}

# Escala px → mm por eje (misma convención que metrics_robot_precision.py)
_MM_X = FIELD_PHYSICAL_WIDTH_CM * 10.0 / CAMERA_PERSPECTIVE_WIDTH
_MM_Y = FIELD_PHYSICAL_HEIGHT_CM * 10.0 / CAMERA_PERSPECTIVE_HEIGHT


def _cross_track_error_mm(pos, seg_a, seg_b):
    """Distancia en mm del robot al segmento activo de su ruta (escala por eje)."""
    px, py = float(pos[0]), float(pos[1])
    ax, ay = float(seg_a[0]), float(seg_a[1])
    bx, by = float(seg_b[0]), float(seg_b[1])
    dx, dy = bx - ax, by - ay
    l2 = dx * dx + dy * dy
    t = 0.0 if l2 <= 1e-9 else max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / l2))
    ex = (px - (ax + t * dx)) * _MM_X
    ey = (py - (ay + t * dy)) * _MM_Y
    return math.hypot(ex, ey)


def _stats(values):
    """mean/std/p95/max/n de una lista de valores, o None si está vacía."""
    if not values:
        return None
    vs = sorted(values)
    return {
        'mean': round(statistics.fmean(vs), 3),
        'std': round(statistics.pstdev(vs), 3) if len(vs) > 1 else 0.0,
        'p95': round(vs[min(len(vs) - 1, int(round(0.95 * (len(vs) - 1))))], 3),
        'max': round(vs[-1], 3),
        'n': len(vs),
    }


class _MatchRecorder:
    """Acumula métricas del partido observando los payloads de los pipes.

    Solo observa: no modifica payloads ni interactúa con percepción, decisión
    o RF. Las métricas de juego se acotan a los tramos con ambos equipos
    activos y sin pausas (init, reset post-gol, pelota fuera).
    """

    def __init__(self):
        self.t0 = None                 # timestamp del primer estado de decisión
        self._prev_ts = None
        self._in_play = False
        self.active_s = 0.0
        self.play_segments = 0
        self.decision_deltas_ms = []
        self._prev_ball_out = False
        self.ball_out_events = 0
        self.goals = []
        self.events = []               # cronología compacta (cap 800)
        self._timeline: list = []      # cronología debuggeable (ver LOG_CHANNELS)
        self._snap_counter: int = 0
        self._tl_prev_ball_out:   bool = False
        self._tl_prev_goal_reset: bool = False
        self._tl_prev_init_phase: bool = False
        # Percepción: contadores acumulados que publica el proceso de percepción
        self._perc_first = None        # (timestamp, frame_idx)
        self._perc_last = None
        self.perc_partido_frames = 0
        self.perc_partido_ball_det = 0
        self.perc_partido_detect = {}
        self.perc_samples_in_play = 0
        self.perc_robots_hist = {n: 0 for n in range(len(ALL_IDS) + 1)}
        self.robots = {
            rid: {
                'action_prev': None,
                'rol_prev': None,
                'has_ball_prev': False,
                'pos_prev': None,
                'dwell_s': {},
                'action_entries': {},
                'role_changes': 0,
                'kicks': 0,
                'seq_initiated': 0,
                'seq_completed': 0,
                'seq_aborted_stage': {1: 0, 2: 0, 3: 0},
                '_seq_max_stage': 0,
                'defender_ball_gains': 0,
                'defender_captures': 0,
                'has_ball_s': 0.0,
                'dist_px': 0.0,
                'tracking_mm': [],
            }
            for rid in ALL_IDS
        }

    def _event(self, t_rel, kind, **extra):
        if len(self.events) < 800:
            self.events.append({'t': round(t_rel, 2), 'event': kind, **extra})

    def _tl(self, t_rel: float, ev: str, **extra) -> None:
        if len(self._timeline) < _TIMELINE_CAP:
            entry = {'t': round(t_rel, 2), 'ev': ev}
            entry.update(extra)
            self._timeline.append(entry)

    def _record_snap(self, data: dict, t_rel: float) -> None:
        players = data.get('players') or {}
        robots_snap: dict = {}
        for rid in ALL_IDS:
            p = players.get(rid)
            if p is None or p.get('pos') is None:
                continue
            rob: dict = {
                'bt':  p.get('last_action'),
                'rol': ROL_LABELS.get(p.get('rol'), '?'),
            }
            if LOG_CHANNELS.get('position'):
                pos = p['pos']
                rob['p'] = [round(pos[0]), round(pos[1])]
            if LOG_CHANNELS.get('angle_in_snap') and p.get('angle_deg') is not None:
                rob['a'] = round(p['angle_deg'], 1)
            if LOG_CHANNELS.get('pwm'):
                rob['L'] = p.get('left_pwm')
                rob['R'] = p.get('right_pwm')
            if LOG_CHANNELS.get('target_in_snap') and p.get('target'):
                rob['tgt'] = [round(p['target'][0]), round(p['target'][1])]
            if LOG_CHANNELS.get('path_in_snap') and p.get('path'):
                rob['path'] = [[round(w[0]), round(w[1])] for w in p['path']]
            robots_snap[str(rid)] = rob
        ball_pos = data.get('ball_pos')
        self._tl(t_rel, 'snap',
                 ball=list(ball_pos) if ball_pos else None,
                 robots=robots_snap)

    def on_goal(self, team):
        t_rel = 0.0 if self.t0 is None else time.time() - self.t0
        self.goals.append({'t_s': round(t_rel, 2), 'team': team})
        self._event(t_rel, 'goal', team=team)

    def on_perception(self, data):
        """Payload de percepción → FPS y tasas de detección durante el partido."""
        ts = data.get('timestamp')
        fi = data.get('frame_idx')
        if ts is not None and fi is not None:
            if self._perc_first is None:
                self._perc_first = (ts, fi)
            self._perc_last = (ts, fi)
        self.perc_partido_frames = data.get('partido_frames', self.perc_partido_frames)
        self.perc_partido_ball_det = data.get('partido_ball_det', self.perc_partido_ball_det)
        pdc = data.get('partido_detect_counts')
        if pdc:
            self.perc_partido_detect = pdc
        if self._in_play:
            n_rob = len(data.get('robots') or {})
            if n_rob in self.perc_robots_hist:
                self.perc_robots_hist[n_rob] += 1
            self.perc_samples_in_play += 1

    def on_decision(self, data):
        """Payload de decisión → tiempo activo, roles, acciones, embudo, rutas."""
        ts = data.get('timestamp')
        if ts is None:
            return
        if self.t0 is None:
            self.t0 = ts
        t_rel = ts - self.t0

        in_play = bool(
            data.get('active_red') and data.get('active_blue')
            and not data.get('pre_init') and not data.get('init_phase')
            and not data.get('goal_reset') and not data.get('ball_out')
        )

        ball_out = bool(data.get('ball_out'))
        if ball_out and not self._prev_ball_out:
            self.ball_out_events += 1
            self._event(t_rel, 'ball_out')
        self._prev_ball_out = ball_out

        # --- Timeline: game_state ---
        if LOG_CHANNELS.get('game_state'):
            for _fld, _attr in (
                ('ball_out',   '_tl_prev_ball_out'),
                ('goal_reset', '_tl_prev_goal_reset'),
                ('init_phase', '_tl_prev_init_phase'),
            ):
                _val = bool(data.get(_fld))
                if _val != getattr(self, _attr):
                    self._tl(t_rel, 'gs', s=_fld, v=int(_val))
                    setattr(self, _attr, _val)

        dt = None
        if self._in_play and in_play and self._prev_ts is not None:
            dt = ts - self._prev_ts
            self.active_s += dt
            self.decision_deltas_ms.append(dt * 1000.0)
        if in_play and not self._in_play:
            self.play_segments += 1
            self._event(t_rel, 'play_start')
        elif self._in_play and not in_play:
            self._event(t_rel, 'play_pause')
            for st in self.robots.values():
                st['pos_prev'] = None
        self._in_play = in_play
        self._prev_ts = ts

        # --- Timeline: snap periódico (todas las fases, no solo in_play) ---
        self._snap_counter += 1
        if self._snap_counter % _SNAP_EVERY_N == 0:
            self._record_snap(data, t_rel)

        if not in_play:
            return

        players = data.get('players') or {}
        for rid, st in self.robots.items():
            p = players.get(rid)
            if not p:
                continue
            rol = p.get('rol')
            action = p.get('last_action')
            has_ball = bool(p.get('has_ball'))
            pos = p.get('pos')

            if st['rol_prev'] is not None and rol != st['rol_prev']:
                st['role_changes'] += 1
                self._event(t_rel, 'role_change', robot=rid,
                            from_rol=st['rol_prev'], to_rol=rol)
                if LOG_CHANNELS.get('rol_change'):
                    self._tl(t_rel, 'rol',
                             r=rid, team=p.get('team', ''),
                             rol=ROL_LABELS.get(rol, '?'))
            st['rol_prev'] = rol

            if action is not None and dt is not None:
                st['dwell_s'][action] = st['dwell_s'].get(action, 0.0) + dt
            if action != st['action_prev']:
                if action is not None:
                    st['action_entries'][action] = st['action_entries'].get(action, 0) + 1
                self._on_action_change(st, rid, action, t_rel)
                st['action_prev'] = action
                if LOG_CHANNELS.get('bt_state') and action is not None:
                    self._tl(t_rel, 'bt',
                             r=rid, team=p.get('team', ''),
                             bt=action)

            if has_ball and dt is not None:
                st['has_ball_s'] += dt
            if has_ball and not st['has_ball_prev'] and rol == 'defensor':
                st['defender_ball_gains'] += 1
                self._event(t_rel, 'defender_ball_gain', robot=rid)
            st['has_ball_prev'] = has_ball

            if pos is not None:
                if st['pos_prev'] is not None:
                    st['dist_px'] += math.hypot(pos[0] - st['pos_prev'][0],
                                                pos[1] - st['pos_prev'][1])
                st['pos_prev'] = pos
                path = p.get('path') or []
                wp_idx = p.get('wp_idx', -1)
                if wp_idx is not None and 1 <= wp_idx < len(path):
                    st['tracking_mm'].append(
                        _cross_track_error_mm(pos, path[wp_idx - 1], path[wp_idx]))

    def _on_action_change(self, st, rid, action, t_rel):
        """Embudo de ataque y recuperaciones, contados por transición de acción."""
        stage = _ATTACK_STAGE.get(action)
        prev_stage = st['_seq_max_stage']
        if stage is not None:
            if prev_stage == 0:
                st['seq_initiated'] += 1
            st['_seq_max_stage'] = max(prev_stage, stage)
            if stage == 4:
                st['kicks'] += 1
                st['seq_completed'] += 1
                st['_seq_max_stage'] = 0
                self._event(t_rel, 'kick', robot=rid)
        else:
            if prev_stage in (1, 2, 3):
                st['seq_aborted_stage'][prev_stage] += 1
            st['_seq_max_stage'] = 0
            if st['rol_prev'] == 'defensor' and action in _DEFENDER_CAPTURE_ACTIONS:
                st['defender_captures'] += 1
                self._event(t_rel, 'defender_capture', robot=rid)

    def summary(self, score):
        team_of = {rid: ('red' if rid in TEAM_RED_IDS else 'blue') for rid in ALL_IDS}
        per_robot = {}
        for rid, st in self.robots.items():
            per_robot[str(rid)] = {
                'team': team_of[rid],
                'kicks': st['kicks'],
                'attack_sequences': {
                    'initiated': st['seq_initiated'],
                    'completed': st['seq_completed'],
                    'aborted_by_stage': {str(k): v
                                         for k, v in st['seq_aborted_stage'].items()},
                },
                'role_changes': st['role_changes'],
                'defender_ball_gains': st['defender_ball_gains'],
                'defender_captures': st['defender_captures'],
                'has_ball_s': round(st['has_ball_s'], 2),
                'distance_traveled_px': round(st['dist_px'], 1),
                'tracking_error_mm': _stats(st['tracking_mm']),
                'actions': {
                    a: {'dwell_s': round(st['dwell_s'].get(a, 0.0), 2),
                        'entries': st['action_entries'].get(a, 0)}
                    for a in sorted(set(st['dwell_s']) | set(st['action_entries']))
                },
            }

        def team_sum(field_fn):
            return {team: sum(field_fn(self.robots[rid]) for rid in ids)
                    for team, ids in (('red', TEAM_RED_IDS), ('blue', TEAM_BLUE_IDS))}

        fps = None
        if (self._perc_first and self._perc_last
                and self._perc_last[0] > self._perc_first[0]):
            fps = round((self._perc_last[1] - self._perc_first[1])
                        / (self._perc_last[0] - self._perc_first[0]), 2)
        ball_rate = (round(100.0 * self.perc_partido_ball_det / self.perc_partido_frames, 1)
                     if self.perc_partido_frames else None)
        robot_rates = ({str(r): round(100.0 * c / self.perc_partido_frames, 1)
                        for r, c in (self.perc_partido_detect or {}).items()}
                       if self.perc_partido_frames else None)

        duration_wall = (round(self._prev_ts - self.t0, 1)
                         if self.t0 is not None and self._prev_ts is not None else 0.0)
        return {
            'mode': 'match_2v2',
            'teams': {'red': list(TEAM_RED_IDS), 'blue': list(TEAM_BLUE_IDS)},
            'robot_id_note': 'IDs de código 0-3 (en prosa de la tesis: 1-4)',
            'duration_wall_s': duration_wall,
            'duration_active_s': round(self.active_s, 1),
            'play_segments': self.play_segments,
            'score': dict(score),
            'goals': self.goals,
            'ball_out_events': self.ball_out_events,
            'decision_cadence_ms': _stats(self.decision_deltas_ms),
            'perception_in_match': {
                'camera_fps_session': fps,
                'partido_frames': self.perc_partido_frames,
                'ball_detection_rate_pct': ball_rate,
                'robot_detection_rate_pct': robot_rates,
                'robots_detected_hist_sampled': {str(k): v
                                                 for k, v in self.perc_robots_hist.items()},
                'n_samples_in_play': self.perc_samples_in_play,
            },
            'kicks_team': team_sum(lambda s: s['kicks']),
            'attack_sequences_team': {
                'initiated': team_sum(lambda s: s['seq_initiated']),
                'completed': team_sum(lambda s: s['seq_completed']),
            },
            'role_changes_total': sum(s['role_changes'] for s in self.robots.values()),
            'action_repertoire_in_play': sorted(
                {a for st in self.robots.values() for a in st['action_entries']}),
            'per_robot': per_robot,
            'events': self.events,
            'notes': [
                'Métricas acotadas a tramos con ambos equipos activos (sin init, reset post-gol ni pelota fuera).',
                'Muestreo por estados de decisión (~25 Hz); el BT publica cambios cada ~100 ms, ninguna acción queda sin muestrear.',
                'Embudo de ataque: 1=posicionamiento, 2=avance, 3=contacto, 4=disparo; abortos contados por etapa máxima alcanzada.',
                'tracking_error_mm: distancia del robot al segmento activo de su ruta RRT*, escala por eje px→mm.',
                'decision_cadence_ms: intervalo entre estados publicados por el proceso de decisión (throttle nominal 40 ms) bajo carga real.',
                'partido_frames/tasas de detección: contadores del proceso de percepción acotados por game_active (incluyen tramos de pelota fuera).',
            ],
        }

    def timeline_data(self, score) -> dict:
        """Cronología compacta para debug — archivo separado del de métricas."""
        return {
            'teams': {'red': list(TEAM_RED_IDS), 'blue': list(TEAM_BLUE_IDS)},
            'score': dict(score),
            'duration_active_s': round(self.active_s, 1),
            'log_channels': {k: v for k, v in LOG_CHANNELS.items() if v},
            'snap_every_n_decisions': _SNAP_EVERY_N,
            'n_entries': len(self._timeline),
            'timeline': self._timeline,
        }


# =============================================================================
# PROCESO 1: Percepción
# =============================================================================

def perception_process_2v2(control_pipe, viz_pipe, camera_id, shm_name,
                            frame_counter, game_active):
    import cv2
    import numpy as np
    from robot_soccer.perception.player_tracking import create_aruco_detector

    log.info("Percepcion 2v2 iniciada — robots %s", ALL_IDS)

    cap = cv2.VideoCapture(camera_id)
    if not cap.isOpened():
        log.error("No se pudo abrir camara %d", camera_id)
        return

    perspective_matrix = None
    if CAMERA_PERSPECTIVE_ENABLED:
        src = np.float32(CAMERA_PERSPECTIVE_SRC_POINTS)
        dst = np.float32([
            [0, 0],
            [CAMERA_PERSPECTIVE_WIDTH - 1, 0],
            [CAMERA_PERSPECTIVE_WIDTH - 1, CAMERA_PERSPECTIVE_HEIGHT - 1],
            [0, CAMERA_PERSPECTIVE_HEIGHT - 1],
        ])
        perspective_matrix = cv2.getPerspectiveTransform(src, dst)

    detector     = create_aruco_detector(use_camera=True)
    robot_ids_set = set(ALL_IDS)

    frame_shape  = (CAMERA_PERSPECTIVE_HEIGHT, CAMERA_PERSPECTIVE_WIDTH, 3)
    shm          = shared_memory.SharedMemory(name=shm_name)
    shared_array = np.ndarray(frame_shape, dtype=np.uint8, buffer=shm.buf)

    ball_lower = np.array(RANGO_COLOR_NARANJO[0])
    ball_upper = np.array(RANGO_COLOR_NARANJO[1])

    frame_count   = 0
    detect_counts = {rid: 0 for rid in ALL_IDS}
    ball_det_count = 0
    partido_frames   = 0   # solo cuenta frames con game_active==1
    partido_ball_det = 0   # solo cuenta detecciones con game_active==1
    partido_detect_counts = {rid: 0 for rid in ALL_IDS}  # detecciones ArUco con game_active==1
    t0 = time.time()
    last_goal_time = 0.0

    try:
        while True:
            ret, raw_frame = cap.read()
            if not ret:
                continue

            frame = (cv2.warpPerspective(raw_frame, perspective_matrix,
                     (CAMERA_PERSPECTIVE_WIDTH, CAMERA_PERSPECTIVE_HEIGHT))
                     if perspective_matrix is not None else raw_frame)
            frame_count += 1
            in_partido = bool(game_active.value)
            if in_partido:
                partido_frames += 1

            # ArUco
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            corners, ids, _ = detector.detectMarkers(gray)

            robots_data = {}
            if ids is not None:
                for i, mid in enumerate(ids.flatten()):
                    if mid in robot_ids_set:
                        pts = corners[i].reshape(4, 2)
                        cx  = int(pts[:, 0].mean())
                        cy  = int(pts[:, 1].mean())
                        vec = pts[1] - pts[0]
                        ang = float(np.degrees(np.arctan2(vec[1], vec[0])))
                        robots_data[int(mid)] = {'x': cx, 'y': cy, 'angulo': ang}
                        detect_counts[int(mid)] = detect_counts.get(int(mid), 0) + 1
                        if in_partido:
                            partido_detect_counts[int(mid)] = (
                                partido_detect_counts.get(int(mid), 0) + 1)

            # Pelota HSV
            hsv  = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv, ball_lower, ball_upper)
            k    = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  k, iterations=2)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k, iterations=2)

            ball_detected = False
            ball_pos      = None
            contours, _   = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if contours:
                largest = max(contours, key=cv2.contourArea)
                if cv2.contourArea(largest) > 10:
                    (bx, by), radius = cv2.minEnclosingCircle(largest)
                    if 2 <= radius <= 30:
                        ball_pos      = (int(bx), int(by))
                        ball_detected = True
                        ball_det_count += 1
                        if in_partido:
                            partido_ball_det += 1

            # Detección de gol en ambos arcos
            goal_event = None
            if ball_detected and ball_pos is not None:
                bx, by = ball_pos
                now    = time.time()
                # Rojo anota: arco derecho
                if (bx >= FIELD_CAM.goal_right_x - BALL_RADIUS_PX and
                        FIELD_CAM.goal_right_top_y <= by <= FIELD_CAM.goal_right_bottom_y):
                    if now - last_goal_time >= GOAL_COOLDOWN_S:
                        goal_event     = 'red'
                        last_goal_time = now
                        log.info("GOL equipo ROJO en (%d, %d)", bx, by)
                # Azul anota: arco izquierdo
                elif (bx <= FIELD_CAM.goal_left_x + BALL_RADIUS_PX and
                        FIELD_CAM.goal_left_top_y <= by <= FIELD_CAM.goal_left_bottom_y):
                    if now - last_goal_time >= GOAL_COOLDOWN_S:
                        goal_event     = 'blue'
                        last_goal_time = now
                        log.info("GOL equipo AZUL en (%d, %d)", bx, by)

            # Pelota fuera de límites (excluye zonas de arco)
            ball_out = False
            if ball_detected and ball_pos is not None:
                bx, by = ball_pos
                m = BALL_OUT_MARGIN_PX
                if by < m or by > CAMERA_PERSPECTIVE_HEIGHT - m:
                    ball_out = True
                elif (bx < m and
                      not (FIELD_CAM.goal_left_top_y <= by <= FIELD_CAM.goal_left_bottom_y)):
                    ball_out = True
                elif (bx > CAMERA_PERSPECTIVE_WIDTH - m and
                      not (FIELD_CAM.goal_right_top_y <= by <= FIELD_CAM.goal_right_bottom_y)):
                    ball_out = True

            # Shared memory
            np.copyto(shared_array, frame)
            with frame_counter.get_lock():
                frame_counter.value += 1

            payload = {
                'robots': robots_data,
                'ball_detected': ball_detected,
                'ball_pos': ball_pos,
                'ball_out': ball_out,
                'goal_event': goal_event,
                'timestamp': time.time(),
                # Contadores acumulados para métricas (leídos por visualización)
                'frame_idx': frame_count,
                'partido_frames': partido_frames,
                'partido_ball_det': partido_ball_det,
                'partido_detect_counts': dict(partido_detect_counts),
            }
            for pipe in (control_pipe, viz_pipe):
                try:
                    while pipe.poll():
                        _ = pipe.recv()
                    pipe.send(payload)
                except Exception:
                    pass

            # Stats cada 5s — Pelota acotada al partido en curso (game_active==1)
            elapsed = time.time() - t0
            if frame_count % 150 == 0 and elapsed > 0:
                fps    = frame_count / elapsed
                rates  = " | ".join(
                    f"R{rid}={detect_counts.get(rid,0)/frame_count*100:.0f}%"
                    for rid in ALL_IDS
                )
                if partido_frames > 0:
                    b_rate = partido_ball_det / partido_frames * 100
                    log.info("FPS=%.1f | %s | Pelota=%.0f%% (partido)",
                             fps, rates, b_rate)
                else:
                    log.info("FPS=%.1f | %s | Pelota=---%% (sin partido)",
                             fps, rates)

    finally:
        cap.release()
        shm.close()
        log.info("Percepcion 2v2 finalizada")


# =============================================================================
# PROCESO 3: Visualización
# =============================================================================

def visualization_process_2v2(perception_pipe, decision_pipe, keyboard_pipe,
                               shm_name, frame_counter):
    import cv2
    import numpy as np

    log.info("Visualizacion 2v2 iniciada")

    frame_shape  = (CAMERA_PERSPECTIVE_HEIGHT, CAMERA_PERSPECTIVE_WIDTH, 3)
    shm          = shared_memory.SharedMemory(name=shm_name)
    shared_array = np.ndarray(frame_shape, dtype=np.uint8, buffer=shm.buf)

    last_frame_counter = 0
    last_frame = None

    robots_perc   = {}
    ball_pos_perc = None
    players_state = {}
    ball_pos_ctrl = None
    active_red    = False
    active_blue   = False
    robot_available = False
    prev_robot_available = False

    score          = {'red': 0, 'blue': 0}
    last_goal_team = None
    goal_reset_viz = False   # True → esperando SPACE tras gol
    ball_out_viz   = False   # True → pelota fuera de límites
    pre_init_viz   = True    # True → robots quietos, esperando ESPACIO1
    init_phase_viz = False   # True → robots organizándose (entre ESPACIO1 y ESPACIO2)
    init_ready_viz = False   # True → robots listos, esperando ESPACIO2 para iniciar
    ball_detected_viz = False

    window_name = 'Partido 2v2 — RoboCup'
    cv2.namedWindow(window_name)

    # Overlay del kick_point: lectura directa de config.py para inspección.
    # La calibración interactiva vive en scripts/calibrate_behavior_thresholds.py.
    show_kick_point = True

    recorder = _MatchRecorder()

    try:
        while True:
            now = time.time()

            # Frame
            current = frame_counter.value
            if current != last_frame_counter:
                last_frame         = shared_array.copy()
                last_frame_counter = current

            # Percepción
            if perception_pipe.poll():
                try:
                    data          = perception_pipe.recv()
                    robots_perc   = data.get('robots', {})
                    ball_pos_perc = data.get('ball_pos')
                    goal_event    = data.get('goal_event')
                    recorder.on_perception(data)
                    if goal_event in ('red', 'blue'):
                        score[goal_event] += 1
                        last_goal_team = goal_event
                        goal_reset_viz = True
                        recorder.on_goal(goal_event)
                        tablero_cmd = 2 if goal_event == 'red' else 3
                        try:
                            keyboard_pipe.send({'command': 'goal_scored', 'team': goal_event})
                            keyboard_pipe.send({'command': 'tablero', 'cmd': tablero_cmd})
                        except Exception:
                            pass
                        log.info("GOL %s! Marcador R%d - A%d",
                                 goal_event.upper(), score['red'], score['blue'])
                except Exception:
                    pass

            # Decision
            if decision_pipe.poll():
                try:
                    data            = decision_pipe.recv()
                    players_state   = data.get('players', {})
                    ball_pos_ctrl   = data.get('ball_pos')
                    active_red      = data.get('active_red', False)
                    active_blue     = data.get('active_blue', False)
                    robot_available = data.get('robot_available', False)
                    ball_out_viz    = data.get('ball_out', False)
                    pre_init_viz      = data.get('pre_init', False)
                    init_phase_viz    = data.get('init_phase', False)
                    init_ready_viz    = data.get('init_ready', False)
                    ball_detected_viz = data.get('ball_detected', False)
                    if not data.get('goal_reset', False):
                        goal_reset_viz = False   # decision confirmó salida del reset
                    if robot_available and not prev_robot_available:
                        try:
                            keyboard_pipe.send({'command': 'tablero', 'cmd': 4})
                            keyboard_pipe.send({'command': 'tablero', 'cmd': 5})
                        except Exception:
                            pass
                    prev_robot_available = robot_available
                    recorder.on_decision(data)
                except Exception:
                    pass

            if last_frame is not None:
                frame = last_frame.copy()
                h, w  = frame.shape[:2]

                # Campo
                cv2.line(frame, (w // 2, 0), (w // 2, h), (40, 80, 40), 1)
                # Arco izquierdo — azul
                cv2.rectangle(frame,
                    (0, FIELD_CAM.goal_left_top_y),
                    (FIELD_CAM.goal_left_x, FIELD_CAM.goal_left_bottom_y),
                    TEAM_COLOR['blue'], 2)
                cv2.putText(frame, "<- AZUL",
                    (4, FIELD_CAM.goal_left_top_y - 6),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.38, TEAM_COLOR['blue'], 1)
                # Arco derecho — rojo
                cv2.rectangle(frame,
                    (FIELD_CAM.goal_right_x, FIELD_CAM.goal_right_top_y),
                    (w, FIELD_CAM.goal_right_bottom_y),
                    TEAM_COLOR['red'], 2)
                cv2.putText(frame, "ROJO ->",
                    (FIELD_CAM.goal_right_x + 2, FIELD_CAM.goal_right_top_y - 6),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.38, TEAM_COLOR['red'], 1)

                ball_pos = ball_pos_ctrl or ball_pos_perc

                # Robots
                for rid in ALL_IDS:
                    pstate   = players_state.get(rid, {})
                    rd       = robots_perc.get(rid)
                    pos      = pstate.get('pos')
                    angle_deg = pstate.get('angle_deg')
                    rol      = pstate.get('rol', 'defensor')
                    has_ball = pstate.get('has_ball', False)
                    target   = pstate.get('target')
                    team     = pstate.get('team', 'red' if rid in TEAM_RED_IDS else 'blue')

                    if pos is None and rd:
                        pos       = (rd['x'], rd['y'])
                        angle_deg = rd['angulo']

                    base_color   = ROBOT_COLORS.get(rid, (180, 180, 180))
                    robot_color  = (0, 255, 255) if has_ball else base_color

                    # Path RRT* del robot: segmentos entre waypoints + círculos
                    # en cada uno. El waypoint activo se destaca con relleno.
                    path = pstate.get('path') or []
                    wp_idx = pstate.get('wp_idx', -1)
                    if len(path) >= 1 and pos is not None:
                        prev = pos
                        for i, wp in enumerate(path):
                            cv2.line(frame, prev, wp, base_color, 1, cv2.LINE_AA)
                            prev = wp
                        for i, wp in enumerate(path):
                            r = 4 if i == wp_idx else 3
                            cv2.circle(frame, wp, r,
                                       base_color, -1 if i == wp_idx else 1,
                                       cv2.LINE_AA)

                    if target:
                        tx, ty = target
                        arm = 8
                        cv2.line(frame, (tx-arm, ty), (tx+arm, ty), (255, 0, 255), 2, cv2.LINE_AA)
                        cv2.line(frame, (tx, ty-arm), (tx, ty+arm), (255, 0, 255), 2, cv2.LINE_AA)

                    if pos:
                        rx, ry = pos
                        if target and rol == 'atacante' and not path:
                            cv2.line(frame, (rx, ry), target, (200, 0, 200), 1, cv2.LINE_AA)
                        cv2.circle(frame, (rx, ry), 16, robot_color, 2, cv2.LINE_AA)
                        if angle_deg is not None:
                            ar = math.radians(angle_deg)
                            ex = int(rx + 25 * math.cos(ar))
                            ey = int(ry + 25 * math.sin(ar))
                            cv2.arrowedLine(frame, (rx, ry), (ex, ey),
                                            robot_color, 2, cv2.LINE_AA, tipLength=0.3)

                            # Overlay kick_point: punto donde se asume que impacta
                            # el solenoide (lectura de config.py). Verde si la
                            # pelota cae dentro de la tolerancia (CONTACTO
                            # geométrico), amarillo en caso contrario.
                            if show_kick_point:
                                kpx = int(rx + KICK_POINT_OFFSET_PX * math.cos(ar))
                                kpy = int(ry + KICK_POINT_OFFSET_PX * math.sin(ar))
                                ball_in_zone = False
                                if ball_pos is not None:
                                    bxc, byc = ball_pos
                                    err = math.hypot(bxc - kpx, byc - kpy)
                                    ball_in_zone = err <= KICK_POINT_TOLERANCE_PX
                                kp_col = ((40, 220, 40) if ball_in_zone
                                          else (40, 220, 220))
                                # Línea punteada robot → kick_point
                                steps = 6
                                for s in range(steps):
                                    if s % 2 == 0:
                                        x0 = int(rx + (kpx - rx) * s / steps)
                                        y0 = int(ry + (kpy - ry) * s / steps)
                                        x1 = int(rx + (kpx - rx) * (s + 1) / steps)
                                        y1 = int(ry + (kpy - ry) * (s + 1) / steps)
                                        cv2.line(frame, (x0, y0), (x1, y1),
                                                 kp_col, 1, cv2.LINE_AA)
                                # Cruz en el kick_point
                                cv2.line(frame, (kpx - 4, kpy), (kpx + 4, kpy),
                                         kp_col, 2, cv2.LINE_AA)
                                cv2.line(frame, (kpx, kpy - 4), (kpx, kpy + 4),
                                         kp_col, 2, cv2.LINE_AA)
                                # Círculo de tolerancia
                                cv2.circle(frame, (kpx, kpy),
                                           max(1, int(KICK_POINT_TOLERANCE_PX)),
                                           kp_col, 1, cv2.LINE_AA)

                        rol_l = ROL_LABELS.get(rol, '?')
                        cv2.putText(frame, f"{rid}{rol_l}", (rx - 8, ry - 20),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, robot_color, 2)

                # Pelota
                if ball_pos:
                    bx, by = ball_pos
                    any_has = any(players_state.get(r, {}).get('has_ball', False)
                                  for r in ALL_IDS)
                    bcol = (0, 255, 255) if any_has else (0, 165, 255)
                    cv2.circle(frame, (bx, by), 12, bcol, 2, cv2.LINE_AA)
                    cv2.circle(frame, (bx, by),  2, bcol, -1, cv2.LINE_AA)

                # Estado BT por equipo (esquina superior izquierda)
                def _bt_label(active, label):
                    return f"BT {label}: {'ACTIVO' if active else 'PAUSADO'}"
                y = 24
                for active, label, color in (
                    (active_red,  "ROJO", TEAM_COLOR['red']),
                    (active_blue, "AZUL", TEAM_COLOR['blue']),
                ):
                    c = color if active else (80, 80, 80)
                    cv2.putText(frame, _bt_label(active, label), (10, y),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, c, 1)
                    y += 18

                rf_col = (0, 220, 80) if robot_available else (80, 80, 80)
                cv2.putText(frame, "RF: OK" if robot_available else "RF: --",
                            (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.42, rf_col, 1)
                y += 16

                for rid in ALL_IDS:
                    pstate = players_state.get(rid, {})
                    pos    = pstate.get('pos')
                    rol    = pstate.get('rol', '?')
                    act    = pstate.get('last_action')
                    hball  = pstate.get('has_ball', False)
                    bc     = ROBOT_COLORS.get(rid, (160, 160, 160))
                    rc     = (0, 255, 255) if hball else bc
                    ps     = f"({pos[0]},{pos[1]})" if pos else "--"
                    as_    = f" {str(act)[:14]}" if act else ""
                    cv2.putText(frame, f"R{rid}[{ROL_LABELS.get(rol,'?')}] {ps}{as_}",
                                (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.37, rc, 1)
                    y += 16

                # Marcador centrado
                score_text = f"ROJO: {score['red']}    AZUL: {score['blue']}"
                (tw, _), _ = cv2.getTextSize(score_text, cv2.FONT_HERSHEY_SIMPLEX, 0.75, 2)
                cv2.putText(frame, score_text, ((w - tw) // 2, 28),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.75, (255, 255, 255), 2, cv2.LINE_AA)

                cv2.putText(frame, "ESPACIO=Ambos  R=Rojo  B=Azul  K=KickPt  ESC=Salir",
                            (10, h - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (180, 180, 180), 1)

                # HUD del kick_point: valores activos en config.py
                if show_kick_point:
                    kp_hud = (f"KICK_POINT  offset={KICK_POINT_OFFSET_PX}px  "
                              f"tol={KICK_POINT_TOLERANCE_PX}px  "
                              f"(calibrar en calibrate_behavior_thresholds.py)")
                    (kw, _), _ = cv2.getTextSize(kp_hud, cv2.FONT_HERSHEY_SIMPLEX, 0.4, 1)
                    cv2.putText(frame, kp_hud, (w - kw - 10, h - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.4, (40, 220, 220), 1, cv2.LINE_AA)

                # Overlay pre-juego: pre_init (quieto) / init_phase (organizando) / init_ready (listo)
                if pre_init_viz or init_phase_viz:
                    robots_seen = sum(
                        1 for rid in ALL_IDS
                        if players_state.get(rid, {}).get('pos') is not None
                    )
                    overlay = frame.copy()
                    cv2.rectangle(overlay, (0, h // 3), (w, 2 * h // 3), (15, 40, 15), -1)
                    cv2.addWeighted(overlay, 0.6, frame, 0.4, 0, frame)

                    if pre_init_viz:
                        if robots_seen == len(ALL_IDS):
                            main_txt = "ROBOTS DETECTADOS"
                            sub_txt  = "Presiona ESPACIO para organizar"
                            txt_col  = (80, 220, 80)
                        else:
                            main_txt = f"Detectando robots... ({robots_seen}/{len(ALL_IDS)})"
                            sub_txt  = "Posiciona todos los robots en el campo"
                            txt_col  = (80, 160, 220)
                    elif init_ready_viz:
                        ball_ok   = ball_detected_viz
                        main_txt  = "ROBOTS LISTOS"
                        sub_txt   = ("Presiona ESPACIO para iniciar el partido"
                                     if ball_ok else
                                     "Coloca la pelota en el campo y presiona ESPACIO")
                        txt_col   = (60, 220, 60) if ball_ok else (30, 160, 220)
                    else:
                        main_txt = "ORGANIZANDO ROBOTS..."
                        sub_txt  = "Espera que los robots lleguen a sus posiciones"
                        txt_col  = (80, 220, 80)

                    (tw_m, th_m), _ = cv2.getTextSize(main_txt, cv2.FONT_HERSHEY_SIMPLEX, 1.0, 2)
                    cv2.putText(frame, main_txt,
                                ((w - tw_m) // 2, h // 2 - 8),
                                cv2.FONT_HERSHEY_SIMPLEX, 1.0, txt_col, 2, cv2.LINE_AA)
                    (tw_s, _), _ = cv2.getTextSize(sub_txt, cv2.FONT_HERSHEY_SIMPLEX, 0.55, 1)
                    cv2.putText(frame, sub_txt,
                                ((w - tw_s) // 2, h // 2 + th_m + 8),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (180, 180, 180), 1, cv2.LINE_AA)

                    # Hint de posición de pelota (centro del campo)
                    bx, by = w // 2, h // 2
                    hint_col = (40, 200, 40) if ball_detected_viz else (40, 140, 255)
                    cv2.circle(frame, (bx, by), 38, hint_col, 2, cv2.LINE_AA)
                    cv2.circle(frame, (bx, by), 10, hint_col, -1, cv2.LINE_AA)
                    cv2.line(frame, (bx - 14, by), (bx + 14, by), (0, 0, 0), 1, cv2.LINE_AA)
                    cv2.line(frame, (bx, by - 14), (bx, by + 14), (0, 0, 0), 1, cv2.LINE_AA)
                    pelota_lbl = "PELOTA OK" if ball_detected_viz else "PELOTA AQUI"
                    cv2.putText(frame, pelota_lbl,
                                (bx - 38, by + 54),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.42, hint_col, 1, cv2.LINE_AA)

                    # Marcadores X en posiciones de inicio
                    for rid, rpos in RESET_POS.items():
                        rc  = TEAM_COLOR['red'] if rid in TEAM_RED_IDS else TEAM_COLOR['blue']
                        rx, ry = rpos
                        arm = 12
                        cv2.line(frame, (rx-arm, ry-arm), (rx+arm, ry+arm), rc, 2, cv2.LINE_AA)
                        cv2.line(frame, (rx+arm, ry-arm), (rx-arm, ry+arm), rc, 2, cv2.LINE_AA)
                        cv2.circle(frame, rpos, arm + 3, rc, 1, cv2.LINE_AA)

                # Overlay GOOOL! + PRESS SPACE + marcadores reset
                if goal_reset_viz and last_goal_team is not None:
                    overlay     = frame.copy()
                    banner_y1   = h // 3
                    banner_y2   = 2 * h // 3
                    banner_col  = ((0, 0, 160)   if last_goal_team == 'red'
                                   else (160, 0, 0))
                    text_col    = (TEAM_COLOR['red'] if last_goal_team == 'red'
                                   else TEAM_COLOR['blue'])
                    goal_label  = ("GOOOL!  ROJO" if last_goal_team == 'red'
                                   else "GOOOL!  AZUL")
                    cv2.rectangle(overlay, (0, banner_y1), (w, banner_y2), banner_col, -1)
                    cv2.addWeighted(overlay, 0.55, frame, 0.45, 0, frame)
                    (gw, gh), _ = cv2.getTextSize(goal_label, cv2.FONT_HERSHEY_SIMPLEX, 2.0, 4)
                    gy = (banner_y1 + banner_y2 + gh) // 2
                    cv2.putText(frame, goal_label, ((w - gw) // 2, gy),
                                cv2.FONT_HERSHEY_SIMPLEX, 2.0, text_col, 4, cv2.LINE_AA)
                    sub_text = "Presiona ESPACIO para continuar"
                    (sw, _), _ = cv2.getTextSize(sub_text, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 1)
                    cv2.putText(frame, sub_text, ((w - sw) // 2, banner_y2 + 24),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (220, 220, 220), 1, cv2.LINE_AA)
                    # Marcadores X en posiciones de reset
                    for rid, rpos in RESET_POS.items():
                        rc  = TEAM_COLOR['red'] if rid in TEAM_RED_IDS else TEAM_COLOR['blue']
                        rx, ry = rpos
                        arm = 10
                        cv2.line(frame, (rx-arm, ry-arm), (rx+arm, ry+arm), rc, 2, cv2.LINE_AA)
                        cv2.line(frame, (rx+arm, ry-arm), (rx-arm, ry+arm), rc, 2, cv2.LINE_AA)
                        cv2.circle(frame, rpos, arm + 2, rc, 1, cv2.LINE_AA)

                # Overlay PELOTA FUERA
                if ball_out_viz:
                    hc = h // 2
                    cv2.rectangle(frame, (0, hc - 28), (w, hc + 28), (20, 20, 20), -1)
                    out_text = "PELOTA FUERA"
                    (tw_o, _), _ = cv2.getTextSize(out_text, cv2.FONT_HERSHEY_SIMPLEX, 1.0, 2)
                    cv2.putText(frame, out_text, ((w - tw_o) // 2, hc + 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 165, 255), 2, cv2.LINE_AA)

                cv2.imshow(window_name, frame)
            else:
                placeholder = np.zeros(frame_shape, dtype=np.uint8)
                cv2.putText(placeholder, "Esperando frames...",
                            (CAMERA_PERSPECTIVE_WIDTH // 2 - 100,
                             CAMERA_PERSPECTIVE_HEIGHT // 2),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (128, 128, 128), 2)
                cv2.imshow(window_name, placeholder)

            key = cv2.waitKey(1) & 0xFF
            if key == 27:  # ESC
                try:
                    keyboard_pipe.send({'command': 'exit'})
                except Exception:
                    pass
                break
            elif key == ord(' '):
                goal_reset_viz = False
                try:
                    keyboard_pipe.send({'command': 'toggle'})
                    # Enviar señal tablero sólo cuando el partido arranca de verdad
                    # (desde init_ready) o cuando se toggle durante juego activo
                    if init_ready_viz or (not pre_init_viz and not init_phase_viz):
                        keyboard_pipe.send({'command': 'tablero', 'cmd': 1})
                except Exception:
                    pass
            elif key in (ord('r'), ord('R')):
                try:
                    keyboard_pipe.send({'command': 'toggle_red'})
                except Exception:
                    pass
            elif key in (ord('b'), ord('B')):
                try:
                    keyboard_pipe.send({'command': 'toggle_blue'})
                except Exception:
                    pass
            elif key in (ord('k'), ord('K')):
                show_kick_point = not show_kick_point
                log.info("kick_point overlay: %s",
                         "ON" if show_kick_point else "OFF")

    except KeyboardInterrupt:
        pass
    finally:
        try:
            out_path = save_metrics('match_2v2', recorder.summary(score))
            log.info("Métricas del partido guardadas en %s", out_path)
            tl_path = save_metrics('timeline_2v2', recorder.timeline_data(score))
            log.info("Timeline de debug guardado en %s", tl_path)
        except Exception as e:
            log.error("No se pudieron guardar las métricas del partido: %s", e)
        shm.close()
        cv2.destroyAllWindows()
        log.info("Visualizacion 2v2 finalizada")


# =============================================================================
# MAIN
# =============================================================================

def main():
    parser = argparse.ArgumentParser(
        description='Test de integración: partido 2v2 con dos equipos autónomos (Paso 7)')
    parser.add_argument('--serial-port', default='/dev/ttyUSB0',
                        help='Puerto serial compartido (default: /dev/ttyUSB0)')
    parser.add_argument('--camera-id', type=int, default=None,
                        help='ID de cámara (auto-detecta si no se especifica)')
    args = parser.parse_args()

    if args.camera_id is None:
        camera_id = get_camera_index(prefer_droidcam=True, fallback_index=0)
        log.info("Cámara auto-detectada: /dev/video%d", camera_id)
    else:
        camera_id = args.camera_id

    shm_name = SHM_NAME

    try:
        old = shared_memory.SharedMemory(name=shm_name)
        old.close()
        old.unlink()
    except FileNotFoundError:
        pass

    frame_size = CAMERA_PERSPECTIVE_HEIGHT * CAMERA_PERSPECTIVE_WIDTH * 3
    shm        = shared_memory.SharedMemory(create=True, name=shm_name, size=frame_size)
    frame_counter = Value('i', 0)
    game_active   = Value('b', 0)   # 0=pre/init/reset, 1=partido en curso

    perc_to_dec_s, perc_to_dec_r = multiprocessing.Pipe()
    perc_to_viz_s, perc_to_viz_r = multiprocessing.Pipe()
    dec_to_viz_s,  dec_to_viz_r  = multiprocessing.Pipe()
    viz_to_dec_s,  viz_to_dec_r  = multiprocessing.Pipe()

    log.info("=" * 65)
    log.info("  PARTIDO 2v2 — RoboCup Soccer")
    log.info("=" * 65)
    log.info("Equipo ROJO  : robots %s  (arco derecho)", TEAM_RED_IDS)
    log.info("Equipo AZUL  : robots %s  (arco izquierdo)", TEAM_BLUE_IDS)
    log.info("Puerto serial: %s", args.serial_port)
    log.info("Cámara       : /dev/video%d", camera_id)
    log.info("Campo        : FIELD_CAM %dx%d", CAMERA_PERSPECTIVE_WIDTH, CAMERA_PERSPECTIVE_HEIGHT)
    log.info("=" * 65)
    log.info("Secuencia de inicio:")
    log.info("  1. Verificar que los 4 robots aparecen en la visualizacion")
    log.info("  2. Colocar la pelota en el centro del campo (indicador naranja)")
    log.info("  3. Presionar ESPACIO: robots se organizan en posiciones de inicio")
    log.info("  4. Esperar a que el indicador diga ROBOTS LISTOS")
    log.info("  5. Presionar ESPACIO: partido inicia (solo si robots y pelota OK)")
    log.info("  (R / B: activar solo rojo / azul para pruebas)")
    log.info("=" * 65)

    p1 = multiprocessing.Process(
        target=perception_process_2v2,
        args=(perc_to_dec_s, perc_to_viz_s, camera_id, shm_name,
              frame_counter, game_active),
        name="Perception"
    )
    p2 = multiprocessing.Process(
        target=decision_process_2v2,
        args=(perc_to_dec_r, dec_to_viz_s, viz_to_dec_r,
              TEAM_RED_IDS, TEAM_BLUE_IDS, args.serial_port, game_active),
        name="Decision2v2"
    )
    p3 = multiprocessing.Process(
        target=visualization_process_2v2,
        args=(perc_to_viz_r, dec_to_viz_r, viz_to_dec_s, shm_name, frame_counter),
        name="Visualization"
    )

    processes = [p1, p2, p3]
    try:
        for proc in processes:
            proc.start()
            log.info("  %s iniciado (PID: %d)", proc.name, proc.pid)
        log.info("")
        log.info("Sistema corriendo. Presiona ESPACIO para organizar robots, luego ESPACIO de nuevo para iniciar.")
        for proc in processes:
            proc.join()
    except KeyboardInterrupt:
        log.info("Interrumpido por usuario")
        for proc in processes:
            if proc.is_alive():
                proc.terminate()
                proc.join(timeout=2)
    finally:
        try:
            shm.close()
            shm.unlink()
        except Exception:
            pass
        log.info("Partido 2v2 finalizado")


if __name__ == '__main__':
    main()
