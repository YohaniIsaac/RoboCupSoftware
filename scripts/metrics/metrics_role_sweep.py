#!/usr/bin/env python3
"""Barrido del espacio de condiciones para la validación cuantitativa de OBJ 3.

Sin hardware, sin cámara, sin RF. Ejecutar con:
    cd ~/git/RoboCupSoftware
    .venv/bin/python scripts/metrics/metrics_role_sweep.py

A diferencia de metrics_behavior_sim.py (6 configuraciones representativas), este
script barre cientos de configuraciones aleatorias para producir métricas de
cumplimiento del objetivo, contrastadas contra un criterio geométrico independiente
del algoritmo bajo prueba.

Cada configuración se clasifica por la regla con prioridad que la gobierna
(posesión, zona de compromiso, histéresis decisiva o banda muerta) y se contrasta
la decisión del algoritmo con la esperada por esa regla. Así cada regla se valida
estadísticamente y la prueba puede fallar (lo hizo, antes de corregir el doble
asignador de roles documentado en metrics_behavior_sim.py).

Produce LOG/role_sweep_<timestamp>.json con:
  - Parte A (correctitud): acierto por régimen de regla y tasa de conmutaciones
    espurias partiendo del titular correcto (estabilidad).
  - Parte B (variedad): número de combinaciones de acción (atacante, defensor)
    distintas sobre una grilla de condiciones, como medida de la diversidad de
    comportamiento en función del estado del campo.
"""

import sys
import time
import logging
from pathlib import Path

import numpy as np

ROOT_DIR = Path(__file__).parent.parent.parent
sys.path.insert(0, str(ROOT_DIR / "src"))
sys.path.insert(0, str(ROOT_DIR / "scripts"))

logging.basicConfig(level=logging.ERROR, format="%(levelname)-8s %(message)s")
log = logging.getLogger(__name__)

from robot_soccer.entities.player import Player
from robot_soccer.entities.ball import Ball
from robot_soccer.ai.behavior_tree.manager import BehaviorManager
from robot_soccer.config import (
    ROL_ATACANTE, ROL_DEFENSIVO, FIELD_SIM,
    BT_ROLE_SWITCH_HYSTERESIS, BT_ROLE_COMMITMENT_RATIO,
    CAPTURE_CONFIRM_DISTANCE_PX,
)

from metrics.metrics_capture import save_metrics

FIELD = FIELD_SIM
K_H = BT_ROLE_SWITCH_HYSTERESIS              # 1.5
COMMITMENT_PX = FIELD.ratio_to_px(BT_ROLE_COMMITMENT_RATIO)  # ~345
POSSESSION_PX = CAPTURE_CONFIRM_DISTANCE_PX  # 35
MARGIN = 60
N_CONFIGS = 1000
SEED = 42


def _rand_xy(rng):
    return [int(rng.uniform(MARGIN, FIELD.width - MARGIN)),
            int(rng.uniform(MARGIN, FIELD.height - MARGIN))]


def _eval_role(red, ball, initial_attacker_id):
    """Ejecuta el algoritmo de asignación real (_update_roles) sobre un estado
    estático, partiendo de un titular dado, y devuelve el id del atacante final.

    El cooldown se desactiva (cada configuración es una instantánea independiente);
    la posesión se modela físicamente: solo el robot más cercano puede tener la
    pelota, y solo si está dentro del umbral de captura.
    """
    closest = min(red, key=lambda p: p.distance_to_ball(ball))
    for p in red:
        p._has_ball = (p is closest and p.distance_to_ball(ball) < POSSESSION_PX)
        p.set_rol(ROL_ATACANTE if p.id == initial_attacker_id else ROL_DEFENSIVO)

    m = BehaviorManager.__new__(BehaviorManager)
    m.team_players = red
    m.ball = ball
    m.field = FIELD
    m.trees = {p.id: None for p in red}
    m._role_last_switch_t = 0.0  # cooldown vencido: la instantánea decide por sí sola
    m._update_roles()
    return next(p.id for p in red if p.rol == ROL_ATACANTE)


def run_correctness_sweep():
    """Clasifica cada configuración por la regla con prioridad aplicable y mide
    el acierto del algoritmo frente a la decisión esperada por esa regla."""
    rng = np.random.default_rng(SEED)
    # buckets: [n, aciertos] por régimen
    b = {k: [0, 0] for k in ("posesion", "compromiso", "histeresis", "banda_muerta")}
    n_from_correct = spurious = 0

    for _ in range(N_CONFIGS):
        ball = Ball(*_rand_xy(rng))
        r1 = Player(1, *_rand_xy(rng), 0, "red")
        r2 = Player(2, *_rand_xy(rng), 0, "red")
        red = [r1, r2]

        d1, d2 = r1.distance_to_ball(ball), r2.distance_to_ball(ball)
        if min(d1, d2) < 1.0:
            continue  # pelota sobre un robot: razón indefinida
        closest = r1 if d1 < d2 else r2
        farthest = r2 if closest is r1 else r1
        d_near, d_far = min(d1, d2), max(d1, d2)
        ratio = d_far / d_near

        # Modo corrección: titular = el robot equivocado (el más lejano).
        # Régimen gobernante en orden de prioridad de las reglas del algoritmo.
        final = _eval_role(red, ball, initial_attacker_id=farthest.id)
        if d_near < POSSESSION_PX:
            regime, expected = "posesion", closest.id        # regla 1: poseedor → atacante
        elif d_far < COMMITMENT_PX:
            regime, expected = "compromiso", farthest.id      # regla 3: mantener titular
        elif ratio > K_H:
            regime, expected = "histeresis", closest.id       # regla 4: cambiar al más cercano
        else:
            regime, expected = "banda_muerta", farthest.id    # regla 4: mantener titular
        b[regime][0] += 1
        if final == expected:
            b[regime][1] += 1

        # Modo estabilidad: titular = el robot correcto (el más cercano).
        final2 = _eval_role(red, ball, initial_attacker_id=closest.id)
        n_from_correct += 1
        if final2 != closest.id:
            spurious += 1

    pct = lambda a, n: round(100.0 * a / n, 1) if n else None
    regimes = {k: {"n": v[0], "aciertos": v[1], "pct": pct(v[1], v[0])} for k, v in b.items()}
    return {
        "n_configs": N_CONFIGS,
        "seed": SEED,
        "k_hysteresis": K_H,
        "commitment_px": COMMITMENT_PX,
        "possession_px": POSSESSION_PX,
        "field": {"width": FIELD.width, "height": FIELD.height},
        "regimes": regimes,
        "global_correct": sum(v[1] for v in b.values()),
        "global_n": sum(v[0] for v in b.values()),
        "global_pct": pct(sum(v[1] for v in b.values()), sum(v[0] for v in b.values())),
        "n_from_correct": n_from_correct,
        "spurious_switches": spurious,
        "spurious_switch_pct": pct(spurious, n_from_correct),
    }


# ---------------------------------------------------------------------------
# Parte B: variedad de comportamiento sobre una grilla de condiciones.
# Requiere ticar el árbol; reutiliza el contexto difuso con su asignador de roles
# heredado neutralizado (ver metrics_behavior_sim.py).
# ---------------------------------------------------------------------------
GRID_ALLY_LAYOUTS = [
    {1: [400, 300], 2: [400, 600]},
    {1: [700, 250], 2: [250, 650]},
]
GRID_BALL_X = [200, 500, 750, 1000, 1300]
GRID_BALL_Y = [250, 450, 650]


def run_diversity_sweep():
    from robot_soccer.ai.fuzzy_logic.game_context import FuzzyRobotTeamManager

    att_actions, def_actions, pairs = set(), set(), set()
    n = 0
    for layout in GRID_ALLY_LAYOUTS:
        for bx in GRID_BALL_X:
            for by in GRID_BALL_Y:
                ball = Ball(bx, by)
                players = [
                    Player(1, *layout[1], 0, "red"),
                    Player(2, *layout[2], 0, "red"),
                    Player(3, 1200, 250, 180, "blue"),
                    Player(4, 1200, 700, 180, "blue"),
                ]
                red = [p for p in players if p.team == "red"]
                d = {p.id: p.distance_to_ball(ball) for p in red}
                att0 = min(d, key=d.get)
                for p in red:
                    p.set_rol(ROL_ATACANTE if p.id == att0 else ROL_DEFENSIVO)

                fuzzy = FuzzyRobotTeamManager(players, ball, team="red")
                fuzzy.role_assigner.assign_roles = lambda *a, **k: None
                bm = BehaviorManager(players, ball, team="red", use_real_robots=False)
                try:
                    ctx = fuzzy.evaluar_ms_logic_difusse()
                except Exception:
                    ctx = (0.5, 1.0, 1.0)
                bm.update_game_context(ctx)
                bm.update()

                acts = {}
                for p in red:
                    bb = bm.blackboards.get(p.id)
                    acts[p.rol] = bb.last_action if bb else None
                a, df = acts.get(ROL_ATACANTE), acts.get(ROL_DEFENSIVO)
                if a:
                    att_actions.add(a)
                if df:
                    def_actions.add(df)
                pairs.add((a, df))
                n += 1
                bm.shutdown()

    return {
        "n_grid_configs": n,
        "distinct_attacker_actions": sorted(x for x in att_actions if x),
        "distinct_defender_actions": sorted(x for x in def_actions if x),
        "n_distinct_attacker_actions": len(att_actions),
        "n_distinct_defender_actions": len(def_actions),
        "n_distinct_action_pairs": len(pairs),
    }


def main():
    print("=" * 60)
    print("  BARRIDO DE CONDICIONES — MÉTRICAS OBJ 3")
    print(f"  Campo: {FIELD.width} × {FIELD.height} px | k_h={K_H} | "
          f"compromiso={COMMITMENT_PX}px | posesión<{POSSESSION_PX}px")
    print("=" * 60)

    sweep = run_correctness_sweep()
    print(f"\n[Correctitud]  {sweep['n_configs']} configuraciones (semilla {sweep['seed']})")
    for k, v in sweep["regimes"].items():
        print(f"  {k:<13s} n={v['n']:<4d} acierto={v['pct']}%")
    print(f"  GLOBAL acierto={sweep['global_pct']}% ({sweep['global_correct']}/{sweep['global_n']})")
    print(f"  Conmutaciones espurias (titular correcto): {sweep['spurious_switch_pct']}%")

    try:
        diversity = run_diversity_sweep()
        print(f"\n[Variedad]  {diversity['n_grid_configs']} configuraciones en grilla")
        print(f"  Acciones de atacante distintas: {diversity['n_distinct_attacker_actions']} "
              f"{diversity['distinct_attacker_actions']}")
        print(f"  Acciones de defensor distintas: {diversity['n_distinct_defender_actions']} "
              f"{diversity['distinct_defender_actions']}")
        print(f"  Combinaciones (atacante, defensor) distintas: {diversity['n_distinct_action_pairs']}")
    except Exception as e:
        print(f"\n[Variedad]  ERROR: {e}")
        diversity = {"error": str(e)}

    summary = {"correctness": sweep, "diversity": diversity}
    try:
        out = save_metrics("role_sweep", summary)
        print(f"\n{'=' * 60}\n  Métricas guardadas en: {out}\n{'=' * 60}")
    except Exception as e:
        print(f"\n  ADVERTENCIA: no se pudieron guardar metricas: {e}")


if __name__ == "__main__":
    main()
