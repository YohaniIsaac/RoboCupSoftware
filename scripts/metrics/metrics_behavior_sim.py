#!/usr/bin/env python3
"""Simulación headless para métricas de OBJ 3 (árboles de comportamiento).

Sin hardware, sin cámara, sin RF. Ejecutar con:
    cd ~/git/RoboCupSoftware
    python scripts/metrics/metrics_behavior_sim.py

Produce LOG/behavior_sim_<timestamp>.json con datos para:
  - Subsección "Lógica difusa": salidas (estado_pelota, equipo_cercano, zona_pelota) por escenario
  - Subsección "Asignación de roles": qué robot es atacante y por qué
  - Subsección "Árboles de comportamiento": latencia de update() en ms
"""

import sys
import time
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

from robot_soccer.entities.player import Player
from robot_soccer.entities.ball import Ball
from robot_soccer.ai.fuzzy_logic.game_context import FuzzyRobotTeamManager
from robot_soccer.ai.behavior_tree.manager import BehaviorManager
from robot_soccer.config import ROL_ATACANTE, ROL_DEFENSIVO, ANCHO_CAMPO, ALTO_CAMPO

from metrics.metrics_capture import save_metrics

# ---------------------------------------------------------------------------
# Escenarios de prueba: (nombre, {id: [x, y, angle, team]}, [bx, by])
# Campo: 1500 × 900 px. Rojo ataca hacia la derecha (x alto = zona ataque).
# ---------------------------------------------------------------------------
SCENARIOS = [
    (
        "pelota_centro",
        {1: [400, 450, 0, "red"], 2: [300, 650, 0, "red"],
         3: [1100, 250, 180, "blue"], 4: [1200, 650, 180, "blue"]},
        [750, 450],
    ),
    (
        "pelota_zona_ataque",
        {1: [500, 400, 0, "red"], 2: [350, 650, 0, "red"],
         3: [1200, 250, 180, "blue"], 4: [1300, 650, 180, "blue"]},
        [1100, 450],
    ),
    (
        "pelota_zona_defensa",
        {1: [600, 450, 0, "red"], 2: [250, 450, 0, "red"],
         3: [1200, 250, 180, "blue"], 4: [1200, 700, 180, "blue"]},
        [250, 450],
    ),
    (
        "robot1_cerca_pelota",
        {1: [760, 460, 0, "red"], 2: [300, 450, 0, "red"],
         3: [1200, 250, 180, "blue"], 4: [1200, 700, 180, "blue"]},
        [750, 450],
    ),
    (
        "robot2_cerca_pelota",
        {1: [900, 200, 0, "red"], 2: [755, 455, 0, "red"],
         3: [1200, 250, 180, "blue"], 4: [1200, 700, 180, "blue"]},
        [750, 450],
    ),
    (
        "ambos_lejos_pelota",
        {1: [200, 200, 0, "red"], 2: [200, 700, 0, "red"],
         3: [1300, 200, 180, "blue"], 4: [1300, 700, 180, "blue"]},
        [750, 450],
    ),
]

N_CYCLES = 50


def run_scenario(name, player_data, ball_pos):
    players = [
        Player(pid, d[0], d[1], d[2], d[3])
        for pid, d in player_data.items()
    ]
    ball = Ball(ball_pos[0], ball_pos[1])

    fuzzy = FuzzyRobotTeamManager(players, ball, team="red")
    bm = BehaviorManager(players, ball, team="red",
                         use_real_robots=False, serial_port="/dev/ttyUSB0")

    fuzzy_posesion, fuzzy_proximidad, fuzzy_zona = [], [], []
    role_history = []
    bt_times_ms = []

    for _ in range(N_CYCLES):
        try:
            ctx = fuzzy.evaluar_ms_logic_difusse()
            fuzzy_posesion.append(ctx["estado_pelota"])
            fuzzy_proximidad.append(ctx["equipo_cercano"])
            fuzzy_zona.append(ctx["zona_pelota"])
            bm.update_game_context((
                ctx["estado_pelota"],
                ctx["equipo_cercano"],
                ctx["zona_pelota"],
            ))
        except Exception:
            fuzzy_posesion.append(0.5)
            fuzzy_proximidad.append(1.0)
            fuzzy_zona.append(1.0)

        t0 = time.perf_counter()
        try:
            bm.update()
        except Exception:
            pass
        bt_times_ms.append((time.perf_counter() - t0) * 1000.0)

        roles = {p.id: p.rol for p in players if p.team == "red"}
        role_history.append(roles)

    # Contar cambios de rol
    role_changes = 0
    for i in range(1, len(role_history)):
        if role_history[i] != role_history[i - 1]:
            role_changes += 1

    final_roles = role_history[-1] if role_history else {}
    final_attacker = next(
        (pid for pid, rol in final_roles.items() if rol == ROL_ATACANTE), None
    )

    # Distancias reales al finalizar (para interpretar la asignación)
    red_players = [p for p in players if p.team == "red"]
    distances = {p.id: round(p.distance_to_ball(ball), 1) for p in red_players}

    result = {
        "scenario": name,
        "n_cycles": N_CYCLES,
        "ball_pos": ball_pos,
        "player_positions": {pid: d[:3] for pid, d in player_data.items()},
        "distances_to_ball_px": distances,
        "fuzzy": {
            "estado_pelota_mean": round(statistics.mean(fuzzy_posesion), 3),
            "estado_pelota_std": round(statistics.stdev(fuzzy_posesion), 3) if len(fuzzy_posesion) > 1 else 0.0,
            "equipo_cercano_mean": round(statistics.mean(fuzzy_proximidad), 3),
            "equipo_cercano_std": round(statistics.stdev(fuzzy_proximidad), 3) if len(fuzzy_proximidad) > 1 else 0.0,
            "zona_pelota_mean": round(statistics.mean(fuzzy_zona), 3),
            "zona_pelota_std": round(statistics.stdev(fuzzy_zona), 3) if len(fuzzy_zona) > 1 else 0.0,
        },
        "role_assignment": {
            "role_changes": role_changes,
            "final_attacker_id": final_attacker,
            "final_roles": {str(k): ("atacante" if v == ROL_ATACANTE else "defensor" if v == ROL_DEFENSIVO else "sin_rol")
                            for k, v in final_roles.items()},
        },
        "bt_performance": {
            "update_time_mean_ms": round(statistics.mean(bt_times_ms), 4),
            "update_time_std_ms": round(statistics.stdev(bt_times_ms), 4) if len(bt_times_ms) > 1 else 0.0,
            "update_time_max_ms": round(max(bt_times_ms), 4),
            "update_time_min_ms": round(min(bt_times_ms), 4),
        },
    }

    bm.shutdown()
    return result


def main():
    print("=" * 60)
    print("  SIMULACION HEADLESS — MÉTRICAS OBJ 3")
    print(f"  Campo: {ANCHO_CAMPO} × {ALTO_CAMPO} px | {N_CYCLES} ciclos por escenario")
    print("=" * 60)

    all_results = []
    for name, player_data, ball_pos in SCENARIOS:
        print(f"\n[{name}] ", end="", flush=True)
        try:
            result = run_scenario(name, player_data, ball_pos)
            all_results.append(result)
            ra = result["role_assignment"]
            ft = result["bt_performance"]["update_time_mean_ms"]
            print(f"atacante=R{ra['final_attacker_id']}  "
                  f"cambios_rol={ra['role_changes']}  "
                  f"BT={ft:.3f}ms  "
                  f"OK")
        except Exception as e:
            print(f"ERROR: {e}")

    if not all_results:
        print("\nNo se generaron resultados.")
        return

    # Resumen global de tiempos BT
    all_bt_means = [r["bt_performance"]["update_time_mean_ms"] for r in all_results]
    summary = {
        "n_scenarios": len(all_results),
        "n_cycles_per_scenario": N_CYCLES,
        "field_px": {"width": ANCHO_CAMPO, "height": ALTO_CAMPO},
        "bt_global_mean_ms": round(statistics.mean(all_bt_means), 4),
        "scenarios": all_results,
    }

    out = save_metrics("behavior_sim", summary)
    print(f"\n{'=' * 60}")
    print(f"  Métricas guardadas en: {out}")
    print(f"  Escenarios procesados: {len(all_results)}/{len(SCENARIOS)}")
    print(f"  BT update time global: {summary['bt_global_mean_ms']:.3f} ms")
    print("=" * 60)


if __name__ == "__main__":
    main()
