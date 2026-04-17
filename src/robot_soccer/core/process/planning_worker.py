"""Subprocess de planificación RRT* para el sistema de robot soccer.

Corre en un proceso separado para no bloquear el loop de control BT.
Recibe requests (robot_pos, goal_pos, obstacles) por pipe y devuelve
paths via queue. El goal es dinámico: se incluye en cada request.
"""
import logging
import time

from robot_soccer.ai.path_planning.rrt_star_smart import RrtStarSmart
from robot_soccer.config import (
    FIELD_CAM,
    PATH_PLANNING_OBSTACLE_CLEARANCE,
    RRT_GOAL_SAMPLE_RATE,
    RRT_ITER_MAX,
    RRT_SEARCH_RADIUS,
    RRT_STEP_LEN,
)

log = logging.getLogger(__name__)


def planning_worker(ctrl_pipe, path_queue, clearance=None):
    """Subprocess: recibe (robot_pos, goal_pos, obstacles) y devuelve path via queue.

    Args:
        ctrl_pipe: Pipe de entrada con requests de planificación.
            Formato: {'robot_pos': (x,y), 'goal_pos': (x,y), 'obstacles': [...]}
        path_queue: Queue de salida (maxsize=1) con el path calculado.
            Formato: {'path': [(x,y), ...], 'goal': (x,y)}
        clearance: Margen de seguridad en px añadido a cada obstáculo.
            Si es None usa PATH_PLANNING_OBSTACLE_CLEARANCE de config.
    """
    if clearance is None:
        clearance = PATH_PLANNING_OBSTACLE_CLEARANCE

    rrt = RrtStarSmart(
        step_len=RRT_STEP_LEN,
        goal_sample_rate=RRT_GOAL_SAMPLE_RATE,
        search_radius=RRT_SEARCH_RADIUS,
        iter_max=RRT_ITER_MAX,
        field=FIELD_CAM,
        clearance=clearance,
    )

    try:
        while True:
            try:
                data = ctrl_pipe.recv()
            except EOFError:
                break

            robot_pos = data['robot_pos']
            goal_pos  = data['goal_pos']
            obstacles = data.get('obstacles', [])

            log.info("Planificando desde %s hacia %s | %d obstaculos...",
                     robot_pos, goal_pos, len(obstacles))
            t0 = time.time()
            try:
                rrt.setup(robot_pos, goal_pos, obstacles, field=FIELD_CAM, clearance=clearance)
                rrt.planning()
            except Exception as e:
                log.error("Error en RRT* planning: %s", e)
                continue

            elapsed = time.time() - t0
            path = rrt.path
            if path and len(path) > 0:
                path = list(reversed(path))
                path = [(int(p[0]), int(p[1])) for p in path]
                log.info("Path encontrado: %d wp en %.2fs", len(path), elapsed)
                try:
                    path_queue.get_nowait()   # descartar path obsoleto si la cola estaba llena
                except Exception:
                    pass
                path_queue.put_nowait({'path': path, 'goal': goal_pos})
            else:
                log.warning("RRT* no encontro ruta en %.2fs", elapsed)

    except KeyboardInterrupt:
        pass
    log.info("planning_worker finalizado")
