#!/usr/bin/env python3
"""Calidad de la decisión de asignación de roles bajo un costo de tiempo realista.

Evaluación OFFLINE (sin hardware) de OBJ 3. No mide el desenlace del juego, sino
si la decisión del algoritmo (mandar al robot más cercano en DISTANCIA a buscar la
pelota) coincide con la decisión que tomaría un criterio de TIEMPO realista de
llegada, que sí considera la orientación inicial y los giros.

Modelo de tiempo (validado contra el comportamiento real del controlador, cuyo
docstring indica: "si error angular > umbral: v=0, rotación pura"):
    - El robot recorre la ruta punto a punto.
    - En cada waypoint, si el error de orientación supera 30 grados, gira en el
      lugar (a la velocidad angular medida, distinta en sentido horario y
      antihorario) y luego avanza recto; si no, avanza con corrección continua.

Los parámetros cinemáticos por robot (v en px/s, omega CW/CCW en deg/s) provienen
de la caracterización con hardware (metrics_motion_characterization.py ->
motion_profile_*.json). Si no hay perfil, se usa uno PROVISIONAL marcado como tal,
solo para validar el flujo; los números no son reales hasta caracterizar.

Salida: LOG/role_decision_quality_<timestamp>.json con el % de configuraciones en
que la decisión por distancia coincide con la decisión por tiempo realista y la
distribución de la penalización de tiempo cuando difieren.
"""

import sys
import json
import glob
import math
import time
import logging
import statistics
from pathlib import Path

import numpy as np

ROOT_DIR = Path(__file__).parent.parent.parent
sys.path.insert(0, str(ROOT_DIR / "src"))
sys.path.insert(0, str(ROOT_DIR / "scripts"))

logging.basicConfig(level=logging.WARNING, format="%(levelname)-8s %(message)s")
log = logging.getLogger(__name__)

from robot_soccer.config import (
    FIELD_SIM, ROBOT_LINEAR_START_ANGLE_THRESHOLD_DEG,
)
from metrics.metrics_capture import save_metrics

FIELD = FIELD_SIM
TURN_THRESHOLD_DEG = ROBOT_LINEAR_START_ANGLE_THRESHOLD_DEG  # 30
MARGIN = 60
N_CONFIGS = 1000
SEED = 7

# Perfil PROVISIONAL (NO real). Sustituir por motion_profile_*.json de hardware.
PROVISIONAL_PROFILE = {
    "_provisional": True,
    "default": {
        "v_px_s": 200.0,          # velocidad lineal
        "omega_cw_deg_s": 120.0,  # velocidad angular horaria
        "omega_ccw_deg_s": 110.0, # velocidad angular antihoraria (asimetría)
        "t_turn_overhead_s": 0.12,
        "t_move_overhead_s": 0.10,
    },
}


def load_profile():
    """Carga el perfil de movimiento más reciente, o el provisional con aviso."""
    matches = sorted(glob.glob(str(ROOT_DIR / "LOG" / "motion_profile_*.json")))
    if matches:
        with open(matches[-1], encoding="utf-8") as fh:
            prof = json.load(fh)
        prof["_provisional"] = False
        return prof, matches[-1]
    log.warning("Sin motion_profile_*.json: se usa perfil PROVISIONAL (números no reales).")
    return PROVISIONAL_PROFILE, None


def _scalar(val, fallback=None):
    """El perfil puede guardar cada parámetro como float O como dict {mean,std,n}."""
    if isinstance(val, dict):
        return val.get("mean") or fallback
    return val if val is not None else fallback


def _robot_params(profile, robot_id):
    raw = profile.get(str(robot_id), profile.get("default", PROVISIONAL_PROFILE["default"]))
    dfl = profile.get("default", PROVISIONAL_PROFILE["default"])
    return {
        "v_px_s":           _scalar(raw.get("v_px_s"),          _scalar(dfl.get("v_px_s"), 100.0)),
        "omega_cw_deg_s":   _scalar(raw.get("omega_cw_deg_s"),  _scalar(dfl.get("omega_cw_deg_s"), 60.0)),
        "omega_ccw_deg_s":  _scalar(raw.get("omega_ccw_deg_s"), _scalar(dfl.get("omega_ccw_deg_s"), 60.0)),
        "t_turn_overhead_s": raw.get("t_turn_overhead_s", dfl.get("t_turn_overhead_s", 0.12)),
        "t_move_overhead_s": raw.get("t_move_overhead_s", dfl.get("t_move_overhead_s", 0.10)),
    }


def _wrap180(a):
    return (a + 180.0) % 360.0 - 180.0


def estimate_time_to_target(start_xy, start_heading_deg, target_xy, params):
    """Tiempo estimado en recorrer una ruta de un solo tramo (recta) hacia el
    objetivo, con reorientación inicial si el error supera el umbral de 30 grados.

    Para rutas con obstáculos basta pasar los waypoints intermedios a
    estimate_path_time(); aquí se usa la recta para aislar el efecto de la
    orientación, que es el factor dominante que la distancia ignora.
    """
    return estimate_path_time([start_xy, target_xy], start_heading_deg, params)


def estimate_path_time(waypoints, start_heading_deg, params):
    v = params["v_px_s"]
    w_cw = params["omega_cw_deg_s"]
    w_ccw = params["omega_ccw_deg_s"]
    t_turn_oh = params.get("t_turn_overhead_s", 0.0)
    t_move_oh = params.get("t_move_overhead_s", 0.0)

    t = 0.0
    heading = start_heading_deg
    for a, b in zip(waypoints[:-1], waypoints[1:]):
        seg_dir = math.degrees(math.atan2(b[1] - a[1], b[0] - a[0]))
        dtheta = _wrap180(seg_dir - heading)
        if abs(dtheta) > TURN_THRESHOLD_DEG:
            omega = w_ccw if dtheta > 0 else w_cw   # signo: convención de giro
            t += abs(dtheta) / omega + t_turn_oh
        seg_len = math.hypot(b[0] - a[0], b[1] - a[1])
        t += seg_len / v + t_move_oh
        heading = seg_dir
    return t


def run(profile):
    rng = np.random.default_rng(SEED)
    p_default = profile.get("default", PROVISIONAL_PROFILE["default"])

    n = 0
    agree = 0
    penalties = []        # tiempo extra (s) cuando la decisión por distancia no es la óptima
    dist_margins = []     # diferencia de distancia (px) en los casos de desacuerdo
    for _ in range(N_CONFIGS):
        ball = (rng.uniform(MARGIN, FIELD.width - MARGIN),
                rng.uniform(MARGIN, FIELD.height - MARGIN))
        robots = []
        for rid in (1, 2):
            robots.append({
                "id": rid,
                "xy": (rng.uniform(MARGIN, FIELD.width - MARGIN),
                       rng.uniform(MARGIN, FIELD.height - MARGIN)),
                "heading": rng.uniform(-180, 180),
            })
        # distancia y tiempo realista por robot
        for r in robots:
            r["dist"] = math.hypot(r["xy"][0] - ball[0], r["xy"][1] - ball[1])
            params = _robot_params(profile, r["id"]) if str(r["id"]) in profile else p_default
            r["time"] = estimate_time_to_target(r["xy"], r["heading"], ball, params)
        if min(r["dist"] for r in robots) < 1.0:
            continue

        choice_dist = min(robots, key=lambda r: r["dist"])   # decisión del algoritmo
        choice_time = min(robots, key=lambda r: r["time"])   # decisión por tiempo realista
        n += 1
        if choice_dist["id"] == choice_time["id"]:
            agree += 1
        else:
            penalties.append(choice_dist["time"] - choice_time["time"])
            dist_margins.append(abs(robots[0]["dist"] - robots[1]["dist"]))

    def stats(xs):
        if not xs:
            return {"n": 0}
        return {
            "n": len(xs),
            "mean": round(statistics.mean(xs), 3),
            "median": round(statistics.median(xs), 3),
            "p90": round(sorted(xs)[int(0.9 * len(xs)) - 1], 3),
            "max": round(max(xs), 3),
        }

    return {
        "n_configs": n,
        "seed": SEED,
        "turn_threshold_deg": TURN_THRESHOLD_DEG,
        "profile_provisional": profile.get("_provisional", True),
        "agreement": agree,
        "agreement_pct": round(100.0 * agree / n, 1) if n else None,
        "disagreement": n - agree,
        "disagreement_pct": round(100.0 * (n - agree) / n, 1) if n else None,
        "time_penalty_s": stats(penalties),
        "dist_margin_px_on_disagreement": stats(dist_margins),
    }


def main():
    profile, src = load_profile()
    res = run(profile)
    print("=" * 64)
    print("  CALIDAD DE LA DECISIÓN DE ROL (distancia vs tiempo realista)")
    print("=" * 64)
    if res["profile_provisional"]:
        print("  *** PERFIL PROVISIONAL — números NO reales (falta caracterizar) ***")
    else:
        print(f"  Perfil de movimiento: {src}")
    print(f"  Configuraciones: {res['n_configs']} (semilla {res['seed']})")
    print(f"  Acuerdo distancia==tiempo realista: {res['agreement_pct']}% "
          f"({res['agreement']}/{res['n_configs']})")
    print(f"  Desacuerdo: {res['disagreement_pct']}% ({res['disagreement']})")
    tp = res["time_penalty_s"]
    if tp["n"]:
        print(f"  Penalización de tiempo en desacuerdos (s): "
              f"media={tp['mean']} mediana={tp['median']} p90={tp['p90']} max={tp['max']}")
    dm = res["dist_margin_px_on_disagreement"]
    if dm["n"]:
        print(f"  Margen de distancia en desacuerdos (px): media={dm['mean']} max={dm['max']}")

    try:
        out = save_metrics("role_decision_quality", res)
        print(f"\n  Métricas guardadas en: {out}")
    except Exception as e:
        print(f"\n  ADVERTENCIA: no se pudieron guardar metricas: {e}")
    print("=" * 64)


if __name__ == "__main__":
    main()
