#!/usr/bin/env python3
"""Métricas aisladas del planificador RRT* Smart para OBJ 2 (sin hardware).

Sin cámara, sin RF, sin robots: ejercita directamente RrtStarSmart sobre el
mismo marco de coordenadas (FIELD_CAM, 640x480 px) y con los mismos parámetros
de config que usa el pipeline real (test_path_planning_1robot.py). Ejecutar con:

    cd ~/git/RoboCupSoftware
    python scripts/metrics/metrics_rrt_isolated.py

Produce LOG/rrt_isolated_<timestamp>.json con, por escenario:
  - time_mean_ms, time_std_ms, time_min_ms, time_max_ms
  - waypoints_avg
  - success_rate

Datos para tab:tiempos_planificacion (Cap 4, F2.2).
"""

import sys
import time
import argparse
import statistics
import logging
from pathlib import Path

ROOT_DIR = Path(__file__).parent.parent.parent
sys.path.insert(0, str(ROOT_DIR / "src"))
sys.path.insert(0, str(ROOT_DIR / "scripts"))

logging.basicConfig(
    level=logging.WARNING,
    format="%(asctime)s %(levelname)-8s %(message)s",
)
log = logging.getLogger(__name__)

from robot_soccer.config import (
    FIELD_CAM,
    CAMERA_PERSPECTIVE_WIDTH, CAMERA_PERSPECTIVE_HEIGHT,
    RRT_STEP_LEN, RRT_GOAL_SAMPLE_RATE, RRT_SEARCH_RADIUS, RRT_ITER_MAX,
    PATH_PLANNING_OBSTACLE_CLEARANCE, PATH_PLANNING_ROBOT_OBSTACLE_RADIUS,
)
from robot_soccer.ai.path_planning.rrt_star_smart import RrtStarSmart

from metrics.metrics_capture import save_metrics

# ---------------------------------------------------------------------------
# Escenarios: (nombre, start (x,y), goal (x,y), list_obs)
# Coordenadas en FIELD_CAM (640 x 480 px), el mismo marco del planner real.
# Obstáculos como círculos [x, y, radio]; cada robot se modela con
# PATH_PLANNING_ROBOT_OBSTACLE_RADIUS y recibe el clearance interno del planner.
# Distancias en línea recta: corta ~180 px, media ~390 px, larga ~596 px.
# Nota: con clearance=60 el rango válido de y es [61, 419]. Los valores
# originales (70,420) y (560,60) caían exactamente sobre la zona de clearance
# de las paredes sup/inf → is_inside_obs() los marcaba como colisión.
# Obstáculos media: (240,240) y (360,240), ambos sobre el trayecto horizontal.
# Zonas de clearance solapadas → fuerzan desvío a y<135 o y>345. Start y goal
# quedan a 150 px y 120 px de los obstáculos respectivamente (>105 px, seguro).
# ---------------------------------------------------------------------------
_R = PATH_PLANNING_ROBOT_OBSTACLE_RADIUS

SCENARIOS = [
    ("corta",            (120, 240), (300, 240), []),
    ("media",            (90,  240), (480, 240), []),
    ("media_obstaculos", (90,  240), (480, 240), [[240, 240, _R], [360, 240, _R]]),
    ("larga",            (70,  410), (560,  70), []),
    (
        "larga_obstaculos",
        (70, 410),
        (560, 70),
        [[250, 290, _R], [400, 160, _R]],
    ),
]

N_PLANS = 50


def _straight_line_dist(start, goal):
    return ((goal[0] - start[0]) ** 2 + (goal[1] - start[1]) ** 2) ** 0.5


def _path_length(path):
    """Longitud acumulada (px) de una ruta dada como lista de [x, y]."""
    total = 0.0
    for i in range(1, len(path)):
        dx = path[i][0] - path[i - 1][0]
        dy = path[i][1] - path[i - 1][1]
        total += (dx * dx + dy * dy) ** 0.5
    return total


def run_scenario(name, start, goal, list_obs, n_plans=N_PLANS):
    times_ms = []
    waypoints = []
    path_lengths = []
    successes = 0

    for _ in range(n_plans):
        # Instancia nueva por planificación: aísla el resultado y evita que un
        # rrt.path previo se arrastre si una corrida no encuentra ruta.
        rrt = RrtStarSmart(
            step_len=RRT_STEP_LEN,
            goal_sample_rate=RRT_GOAL_SAMPLE_RATE,
            search_radius=RRT_SEARCH_RADIUS,
            iter_max=RRT_ITER_MAX,
            field=FIELD_CAM,
            clearance=PATH_PLANNING_OBSTACLE_CLEARANCE,
        )

        t0 = time.perf_counter()
        try:
            rrt.setup(start, goal, list_obs, field=FIELD_CAM,
                      clearance=PATH_PLANNING_OBSTACLE_CLEARANCE)
            rrt.planning()
        except Exception as e:
            log.warning("planning() falló en escenario '%s': %s", name, e)
            times_ms.append((time.perf_counter() - t0) * 1000.0)
            continue
        elapsed_ms = (time.perf_counter() - t0) * 1000.0
        times_ms.append(elapsed_ms)

        path = rrt.path
        if path and len(path) >= 2:
            successes += 1
            waypoints.append(len(path))
            path_lengths.append(_path_length(path))

    straight = _straight_line_dist(start, goal)
    result = {
        "scenario": name,
        "n_plans": n_plans,
        "start": list(start),
        "goal": list(goal),
        "n_obstacles": len(list_obs),
        "straight_line_dist_px": round(straight, 1),
        "success_rate": round(successes / n_plans, 3),
        "time_mean_ms": round(statistics.mean(times_ms), 3) if times_ms else 0.0,
        "time_std_ms": round(statistics.stdev(times_ms), 3) if len(times_ms) > 1 else 0.0,
        "time_min_ms": round(min(times_ms), 3) if times_ms else 0.0,
        "time_max_ms": round(max(times_ms), 3) if times_ms else 0.0,
        "waypoints_avg": round(statistics.mean(waypoints), 2) if waypoints else 0.0,
        "waypoints_std": round(statistics.stdev(waypoints), 2) if len(waypoints) > 1 else 0.0,
        "path_length_avg_px": round(statistics.mean(path_lengths), 1) if path_lengths else 0.0,
    }
    return result


def main():
    parser = argparse.ArgumentParser(
        description="Métricas aisladas de RRT* Smart (OBJ 2, sin hardware)."
    )
    parser.add_argument("--seed", type=int, default=42,
                        help="Semilla para reproducibilidad (default 42).")
    parser.add_argument("--n-plans", type=int, default=N_PLANS,
                        help=f"Planificaciones por escenario (default {N_PLANS}).")
    args = parser.parse_args()
    n_plans = args.n_plans

    if args.seed is not None:
        import random
        import numpy as np
        random.seed(args.seed)
        np.random.seed(args.seed)

    print("=" * 64)
    print("  RRT* SMART AISLADO — MÉTRICAS OBJ 2")
    print(f"  Campo: FIELD_CAM ({CAMERA_PERSPECTIVE_WIDTH} x {CAMERA_PERSPECTIVE_HEIGHT} px)")
    print(f"  step_len={RRT_STEP_LEN}  goal_rate={RRT_GOAL_SAMPLE_RATE}  "
          f"iter_max={RRT_ITER_MAX}  clearance={PATH_PLANNING_OBSTACLE_CLEARANCE}")
    print(f"  {n_plans} planificaciones por escenario | seed={args.seed}")
    print("=" * 64)

    all_results = []
    for name, start, goal, list_obs in SCENARIOS:
        print(f"\n[{name}] dist={_straight_line_dist(start, goal):.0f}px "
              f"obs={len(list_obs)} ... ", end="", flush=True)
        try:
            result = run_scenario(name, start, goal, list_obs, n_plans)
            all_results.append(result)
            print(f"t={result['time_mean_ms']:.1f}ms  "
                  f"wp={result['waypoints_avg']:.1f}  "
                  f"éxito={result['success_rate'] * 100:.0f}%")
        except Exception as e:
            print(f"ERROR: {e}")

    if not all_results:
        print("\nNo se generaron resultados.")
        return

    summary = {
        "n_scenarios": len(all_results),
        "n_plans_per_scenario": n_plans,
        "seed": args.seed,
        "field_px": {"width": CAMERA_PERSPECTIVE_WIDTH, "height": CAMERA_PERSPECTIVE_HEIGHT},
        "rrt_params": {
            "step_len": RRT_STEP_LEN,
            "goal_sample_rate": RRT_GOAL_SAMPLE_RATE,
            "search_radius": RRT_SEARCH_RADIUS,
            "iter_max": RRT_ITER_MAX,
            "obstacle_clearance_px": PATH_PLANNING_OBSTACLE_CLEARANCE,
            "robot_obstacle_radius_px": PATH_PLANNING_ROBOT_OBSTACLE_RADIUS,
        },
        "scenarios": all_results,
    }

    try:
        out = save_metrics("rrt_isolated", summary)
        print(f"\n{'=' * 64}")
        print(f"  Métricas guardadas en: {out}")
    except Exception as e:
        print(f"\n  ADVERTENCIA: no se pudieron guardar métricas: {e}")
    print(f"  Escenarios procesados: {len(all_results)}/{len(SCENARIOS)}")
    print("=" * 64)


if __name__ == "__main__":
    main()
