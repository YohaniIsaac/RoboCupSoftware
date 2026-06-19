"""Subprocess de planificación RRT* para el sistema de robot soccer.

Corre en un proceso separado para no bloquear el loop de control BT.
Recibe requests (robot_pos, goal_pos, obstacles) por pipe y devuelve
paths via queue. El goal es dinámico: se incluye en cada request.
"""
import logging
import math
import time

from robot_soccer.ai.path_planning.rrt_star_smart import RrtStarSmart
from robot_soccer.config import (
    FIELD_CAM,
    PATH_PLANNING_OBSTACLE_CLEARANCE,
    RRT_GOAL_PROJECTION_MARGIN_PX,
    RRT_GOAL_SAMPLE_RATE,
    RRT_ITER_MAX,
    RRT_SEARCH_RADIUS,
    RRT_STEP_LEN,
    RRT_WAYPOINT_ARRIVAL_PX,
)

log = logging.getLogger(__name__)


def _release_start(robot_pos, obstacles, clearance, margin):
    """Encoge (o descarta) obstáculos cuyo radio inflado contiene al start.

    Si el robot ya está dentro de la zona inflada de un obstáculo (e.g. en
    contacto físico con otro robot), RRT* no puede expandir el árbol desde el
    start y falla instantáneamente en cada replan. El robot está físicamente
    ahí, así que la inflación — no la geometría real — es lo violado: se reduce
    el radio del obstáculo para dejar el start justo afuera. Si ni con radio 0
    se libera (el clearance global ya lo cubre), se descarta ese obstáculo para
    que al menos exista una ruta de escape.
    """
    rx, ry = robot_pos
    released = []
    for ox, oy, orad in obstacles:
        d = math.hypot(rx - ox, ry - oy)
        if d >= orad + clearance + margin:
            released.append([ox, oy, orad])
            continue
        new_rad = d - clearance - margin
        if new_rad > 0:
            log.info("Start %s dentro de obstáculo (%d,%d) r=%d → radio reducido a %d",
                     robot_pos, ox, oy, orad, int(new_rad))
            released.append([ox, oy, new_rad])
        else:
            log.warning("Start %s a %.0fpx de obstáculo (%d,%d): clearance ya lo "
                        "cubre → obstáculo descartado para permitir escape",
                        robot_pos, d, ox, oy)
    return released


def _project_goal(goal_pos, robot_pos, obstacles, clearance, margin):
    """Proyecta el goal fuera de los obstáculos inflados que lo contienen.

    Un goal dentro del radio inflado de un obstáculo es inalcanzable para RRT*
    (todo edge hacia él colisiona). Se desplaza radialmente al borde del
    obstáculo + margen, iterando por si la proyección cae dentro de otro.
    Retorna el goal (posiblemente desplazado) como tupla de ints.
    """
    gx, gy = float(goal_pos[0]), float(goal_pos[1])
    for _ in range(4):
        blocker = None
        for ox, oy, orad in obstacles:
            if math.hypot(gx - ox, gy - oy) < orad + clearance + margin:
                blocker = (ox, oy, orad)
                break
        if blocker is None:
            break
        ox, oy, orad = blocker
        d = math.hypot(gx - ox, gy - oy)
        if d < 1e-6:
            # Goal exactamente en el centro: alejar hacia el lado del robot
            d_rob = math.hypot(robot_pos[0] - ox, robot_pos[1] - oy)
            ux, uy = ((robot_pos[0] - ox) / d_rob, (robot_pos[1] - oy) / d_rob) \
                if d_rob > 1e-6 else (1.0, 0.0)
        else:
            ux, uy = (gx - ox) / d, (gy - oy) / d
        r_out = orad + clearance + margin
        gx, gy = ox + ux * r_out, oy + uy * r_out
    return (int(gx), int(gy))


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
            # Umbral de llegada por-request (idea B umbrales dinámicos):
            # permite al caller relajar el sanity check final cuando el target
            # es un punto "ilustrativo" (defensa, behind_ball) en vez de uno
            # crítico (captura).
            arrival_px = data.get('arrival_px', RRT_WAYPOINT_ARRIVAL_PX)
            # Clearance por-request (inflación dependiente del contexto): el caller
            # envía un clearance reducido en la zona de disputa de la pelota, donde
            # el global vuelve imposible maniobrar. Si no lo envía, se usa el global.
            req_clearance = data.get('clearance', clearance)

            # Sanear requests degenerados: start atrapado en un obstáculo inflado
            # (robots en contacto) y/o goal dentro de uno (otro robot parado encima).
            # Sin esto RRT* quema iter_max sin solución y el control cae a PID
            # directo sin evasión.
            obstacles_eff = _release_start(
                robot_pos, obstacles, req_clearance, RRT_GOAL_PROJECTION_MARGIN_PX)
            plan_goal = _project_goal(
                goal_pos, robot_pos, obstacles_eff, req_clearance, RRT_GOAL_PROJECTION_MARGIN_PX)
            projected = plan_goal != tuple(goal_pos)
            if projected:
                log.info("Goal %s bloqueado por obstáculo → planificando a %s "
                         "(el robot esperará ahí a que se despeje)", goal_pos, plan_goal)

            log.info("Planificando desde %s hacia %s | %d obstaculos | clearance=%d...",
                     robot_pos, plan_goal, len(obstacles_eff), req_clearance)
            t0 = time.time()
            try:
                rrt.setup(robot_pos, plan_goal, obstacles_eff, field=FIELD_CAM, clearance=req_clearance)
                rrt.planning()
            except Exception as e:
                log.error("Error en RRT* planning: %s", e)
                continue

            elapsed = time.time() - t0
            path = rrt.path
            if path and len(path) > 0:
                path = list(reversed(path))
                path = [(int(p[0]), int(p[1])) for p in path]
                # Sanity check: el último waypoint debe estar cerca del goal
                # planificado (el proyectado si el original estaba bloqueado).
                # Si no, el RRT* SMART produjo un path inválido (e.g.
                # x_goal sin parent → path = [start] solamente, o smoothing
                # erróneo que sustituyó el último wp). Descartar para evitar
                # que el robot navegue hacia un punto sin relación con el goal.
                last_wp_to_goal = math.hypot(path[-1][0] - plan_goal[0],
                                             path[-1][1] - plan_goal[1])
                if last_wp_to_goal > arrival_px:
                    log.warning("Path RRT* descartado: ultimo wp %s a %.0fpx "
                                "del goal %s (>%dpx)", path[-1], last_wp_to_goal,
                                plan_goal, arrival_px)
                    continue
                log.info("Path encontrado: %d wp en %.2fs", len(path), elapsed)
                try:
                    path_queue.get_nowait()   # descartar path obsoleto si la cola estaba llena
                except Exception:
                    pass
                try:
                    # 'goal' lleva el goal ORIGINAL para que el caller lo asocie a su
                    # request; 'projected' indica que el path termina antes del goal real.
                    path_queue.put_nowait({'path': path, 'goal': goal_pos,
                                           'projected': projected})
                except Exception:
                    pass  # cola llena por race condition — el proceso no debe morir
            else:
                log.warning("RRT* no encontro ruta en %.2fs", elapsed)

    except KeyboardInterrupt:
        pass
    log.info("planning_worker finalizado")
