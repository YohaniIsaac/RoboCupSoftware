#!/usr/bin/env python3
"""[metrics] Extrae los episodios de la FASE DE EMPUJE del overshoot (advance_to_contact)
desde el log de test_behavior_1robot.py a un JSON estructurado, para calibrar
CONTACT_PUSH_DISTANCE_PX / CONTACT_PUSH_STALL_* y evaluar si hace falta push_pwm.

NO instrumenta el pipeline: solo lee líneas [EVENT ]/[STATUS] ya emitidas por
robot_status_logger (forma segura, CLAUDE.md). Escribe LOG/push_episodes_<ts>.json
vía metrics_capture.save_metrics (único punto que escribe en LOG/).

Limitación heredada del log (no del parser): timestamps a 1 s y STATUS a ~2 Hz, así
que un episodio de empuje (<1 s) trae 1-2 muestras de STATUS. Sirve para AGREGAR muchos
episodios (cuántos terminan en overshoot vs trabado, avance medio, cv alcanzado), no para
una curva sub-segundo. La L/D de contacto y el avance final salen de los [EVENT ], exactos.

Uso:
    python scripts/integration/test_behavior_1robot.py --robot-id 0 2>&1 | tee LOG/run.log
    python scripts/metrics/metrics_push_episodes.py LOG/run.log
    # o por stdin en vivo:
    python scripts/integration/test_behavior_1robot.py --robot-id 0 2>&1 | \
        python scripts/metrics/metrics_push_episodes.py -
"""
from __future__ import annotations

import re
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from scripts.metrics.metrics_capture import save_metrics  # noqa: E402

# ── Regex sobre el cuerpo del mensaje (tras el prefijo de logging) ──────────────
_TS      = re.compile(r"^(\d{2}):(\d{2}):(\d{2})")
_EVENT   = re.compile(r"\[EVENT\s*\]\s*R(\d+)\s*\|\s*(.*)$")
_STATUS  = re.compile(r"\[STATUS\]\s*R(\d+)\s*\|\s*(.*)$")

_CONTACT  = re.compile(r"CONTACTO(?:\s+INMEDIATO)?:\s*L=([\d.]+)px\s+D=([\d.]+)px")
_OVERSHOOT = re.compile(r"OVERSHOOT OK:\s*avance=([\d.]+)px")
_STALL     = re.compile(r"EMPUJE TRABADO:\s*avance=([\d.]+)px\s+en\s+([\d.]+)s")
_SETTLE_OK = re.compile(r"ASENTAMIENTO OK:\s*t=([\d.]+)s\s+L=([\d.]+)px\s+D=([\d.]+)px")
_ESCAPE    = re.compile(r"PELOTA ESCAPO")
_POSSESS   = re.compile(r"TOPE POSESION")
_RESTAGE   = re.compile(r"(DESALINEADO|DERIVA ANGULAR|ASENTAMIENTO DESCENTRADO)")

# Campos del STATUS que interesan a la fase de empuje.
_STATUS_KEYS = ("state", "pos", "db", "lat", "drb", "cv", "L", "R")


def _ts_seconds(line: str):
    m = _TS.match(line)
    if not m:
        return None
    h, mn, s = (int(x) for x in m.groups())
    return h * 3600 + mn * 60 + s


def _body(line: str) -> str:
    """Quita el prefijo de logging (asctime/proceso/nivel) dejando el mensaje."""
    # formato: 'HH:MM:SS [Proc] LEVEL    <msg>'. El msg arranca en [EVENT/[STATUS.
    i = line.find("[EVENT")
    if i == -1:
        i = line.find("[STATUS")
    return line[i:] if i != -1 else line


def _parse_status_fields(rest: str) -> dict:
    """Parsea 'k=v | k=v | ...' del STATUS a un dict con las claves de interés."""
    out = {}
    for tok in rest.split("|"):
        tok = tok.strip()
        if "=" not in tok:
            continue
        k, _, v = tok.partition("=")
        k, v = k.strip(), v.strip()
        if k in _STATUS_KEYS and v not in ("", "---"):
            out[k] = v
    return out


def parse(lines) -> dict:
    """Recorre el log y arma la lista de episodios de empuje por robot."""
    episodes: list[dict] = []
    open_ep: dict[int, dict] = {}   # robot_id -> episodio abierto

    def _close(rid, end_type, t_sec, **extra):
        ep = open_ep.pop(rid, None)
        if ep is None:
            return
        ep["end"] = {"type": end_type, **extra}
        if t_sec is not None and ep.get("t_start_s") is not None:
            ep["duration_s"] = max(0, t_sec - ep["t_start_s"])
        episodes.append(ep)

    for raw in lines:
        line = raw.rstrip("\n")
        t_sec = _ts_seconds(line)
        body = _body(line)

        ev = _EVENT.search(body)
        if ev:
            rid, msg = int(ev.group(1)), ev.group(2)
            if "advance_contact" not in msg:
                continue
            m = _CONTACT.search(msg)
            if m:  # abre (o reabre) un episodio de empuje
                open_ep[rid] = {
                    "robot": rid,
                    "t_start_s": t_sec,
                    "contact": {"L": float(m.group(1)), "D": float(m.group(2))},
                    "status_samples": [],
                    "events": [msg.strip()],
                }
                continue
            if rid in open_ep:
                open_ep[rid]["events"].append(msg.strip())
            m = _OVERSHOOT.search(msg)
            if m:
                _close(rid, "overshoot_ok", t_sec, avance=float(m.group(1)))
                continue
            m = _STALL.search(msg)
            if m:
                _close(rid, "trabado", t_sec, avance=float(m.group(1)), window_s=float(m.group(2)))
                continue
            if _ESCAPE.search(msg):
                _close(rid, "escaped", t_sec)
                continue
            if _POSSESS.search(msg):
                _close(rid, "timeout_posesion", t_sec)
                continue
            if _RESTAGE.search(msg):
                _close(rid, "restage", t_sec)
                continue
            continue

        st = _STATUS.search(body)
        if st:
            rid = int(st.group(1))
            if rid in open_ep:
                fields = _parse_status_fields(st.group(2))
                if fields:
                    fields["t_s"] = t_sec
                    open_ep[rid]["status_samples"].append(fields)

    # episodios que nunca cerraron (corte del log)
    for rid, ep in list(open_ep.items()):
        ep["end"] = {"type": "unterminated"}
        episodes.append(ep)

    return _summarize(episodes)


def _summarize(episodes: list[dict]) -> dict:
    by_type: dict[str, int] = {}
    avances = []
    cv_seen = []
    for ep in episodes:
        et = ep.get("end", {}).get("type", "unterminated")
        by_type[et] = by_type.get(et, 0) + 1
        a = ep.get("end", {}).get("avance")
        if a is not None:
            avances.append(a)
        for s in ep["status_samples"]:
            if "cv" in s:
                try:
                    cv_seen.append(int(s["cv"]))
                except ValueError:
                    pass

    def _stat(xs):
        return {"n": len(xs), "min": min(xs), "max": max(xs),
                "mean": round(sum(xs) / len(xs), 2)} if xs else {"n": 0}

    return {
        "n_episodes": len(episodes),
        "summary": {
            "by_end_type": by_type,
            "avance_px": _stat(avances),
            "creep_pwm_cv": _stat(cv_seen),
            "note": "timestamps a 1 s y STATUS ~2 Hz: usar para agregados, no curvas sub-segundo",
        },
        "episodes": episodes,
    }


def main(argv):
    if len(argv) < 2:
        print(__doc__)
        return 1
    src = argv[1]
    if src == "-":
        lines = sys.stdin.readlines()
    else:
        lines = Path(src).read_text(encoding="utf-8", errors="replace").splitlines()

    data = parse(lines)
    out = save_metrics("push_episodes", data)
    s = data["summary"]
    print(f"Episodios de empuje: {data['n_episodes']}  ->  {out}")
    print(f"  por fin: {s['by_end_type']}")
    print(f"  avance px: {s['avance_px']}")
    print(f"  creep cv : {s['creep_pwm_cv']}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))
