#!/usr/bin/env python3
"""[metrics] Extrae los episodios de captura (advance_to_contact: contacto -> empuje ->
desenlace) desde el log de test_behavior_1robot.py a un JSON, para responder la pregunta
de diseño: ¿cuándo el robot REALMENTE tiene la pelota?

Cada episodio va del CONTACTO al desenlace real del intento (KICK / PELOTA ESCAPO /
re-stage / tope), capturando la evolución de `db` (dist robot-pelota) para distinguir:
  - la pelota ACOMPAÑÓ al robot (db pequeño y estable)  -> posesión real
  - la pelota SE FUE (db crece)                          -> el robot la tocó y la perdió
El push_end (overshoot_ok / trabado) se guarda como atributo; el criterio de posesión
es db, no el avance del robot (ver análisis de la sesión).

NO instrumenta el pipeline: solo lee [EVENT ]/[STATUS] ya emitidos (forma segura,
CLAUDE.md) y escribe LOG/push_episodes_<ts>.json vía metrics_capture.save_metrics.
Resolución heredada del log: STATUS ~2 Hz; usar agregados, no curvas sub-segundo.

Uso:
    python scripts/metrics/metrics_push_episodes.py LOG/run_1robot_*.log
    python scripts/metrics/metrics_push_episodes.py LOG/run.log --db-threshold 50
    cat LOG/run.log | python scripts/metrics/metrics_push_episodes.py -
"""
from __future__ import annotations

import re
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from scripts.metrics.metrics_capture import save_metrics  # noqa: E402

# umbral por defecto de db (px) para clasificar "la pelota se fue" durante el episodio.
DEFAULT_DB_THRESHOLD = 50

_TS     = re.compile(r"^(\d{2}):(\d{2}):(\d{2})")
_EVENT  = re.compile(r"\[EVENT\s*\]\s*R(\d+)\s*\|\s*(.*)$")
_STATUS = re.compile(r"\[STATUS\]\s*R(\d+)\s*\|\s*(.*)$")

_CONTACT   = re.compile(r"CONTACTO(?:\s+INMEDIATO)?:\s*L=([\d.]+)px\s+D=([\d.]+)px")
_CONFIRMED = re.compile(r"POSESION CONFIRMADA:\s*L=([\d.]+)px\s+D=([\d.]+)px\s+held=([\d.]+)s")
_LOST      = re.compile(r"EMPUJE PELOTA SE FUE:\s*db=([\d.]+)")
_AIMLOST   = re.compile(r"TIRO FUERA:\s*slack=([-\d.]+)")
_CONE      = re.compile(r"cono=(OK|FUERA)\(slack=([-\d.]+)")
_OVERSHOOT = re.compile(r"OVERSHOOT OK:\s*avance=([\d.]+)px")   # logs previos al criterio de posesión
_STALL     = re.compile(r"EMPUJE TRABADO:\s*avance=([\d.]+)px")  # logs previos al criterio de posesión
_SETTLE_OK = re.compile(r"ASENTAMIENTO OK:\s*t=([\d.]+)s\s+L=([\d.]+)px\s+D=([\d.]+)px")
_KICK      = re.compile(r"KICK SOLENOIDE")
_ESCAPE    = re.compile(r"PELOTA ESCAPO[^:]*:\s*dist=([\d.]+)")
_DESCENTRA = re.compile(r"ASENTAMIENTO DESCENTRADO:\s*L=([\d.]+)px\s+D=([\d.]+)px")
_RESTAGE   = re.compile(r"(DESALINEADO|DERIVA ANGULAR)")
_POSSESS   = re.compile(r"TOPE POSESION")

_STATUS_KEYS = ("state", "pos", "db", "lat", "drb", "cv")


def _ts_seconds(line: str):
    m = _TS.match(line)
    if not m:
        return None
    h, mn, s = (int(x) for x in m.groups())
    return h * 3600 + mn * 60 + s


def _body(line: str) -> str:
    i = line.find("[EVENT")
    if i == -1:
        i = line.find("[STATUS")
    return line[i:] if i != -1 else line


def _num(v):
    """'37px' / ' 16' / '+3.0px' -> float, o None."""
    if v is None:
        return None
    m = re.search(r"[-+]?\d+(?:\.\d+)?", str(v))
    return float(m.group()) if m else None


def _parse_status_fields(rest: str) -> dict:
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


def parse_lines(lines, db_threshold=DEFAULT_DB_THRESHOLD) -> list[dict]:
    """Devuelve la lista de episodios (contacto -> desenlace) de UN stream de log."""
    episodes: list[dict] = []
    open_ep: dict[int, dict] = {}

    def _close(rid, outcome, t_sec, **extra):
        ep = open_ep.pop(rid, None)
        if ep is None:
            return
        ep["outcome"] = outcome
        ep.update(extra)
        if t_sec is not None and ep.get("t_start_s") is not None:
            ep["duration_s"] = max(0, t_sec - ep["t_start_s"])
        # métricas de db a lo largo del episodio
        dbs = [s["db_px"] for s in ep["samples"] if s.get("db_px") is not None]
        if dbs:
            ep["db_contact"] = dbs[0]
            ep["db_max"] = max(dbs)
            ep["db_final"] = dbs[-1]
        ep["possession_kept"] = (ep.get("db_max") is not None
                                 and ep["db_max"] <= db_threshold)
        episodes.append(ep)

    for raw in lines:
        line = raw.rstrip("\n")
        t_sec = _ts_seconds(line)
        body = _body(line)

        ev = _EVENT.search(body)
        if ev:
            rid, msg = int(ev.group(1)), ev.group(2)
            m = _CONTACT.search(msg)
            if m:
                # nuevo contacto: si había uno abierto sin desenlace, ciérralo como 'reabierto'
                if rid in open_ep:
                    _close(rid, "reopened", t_sec)
                _c = _CONE.search(msg)
                open_ep[rid] = {
                    "robot": rid, "t_start_s": t_sec,
                    "contact": {"L": float(m.group(1)), "D": float(m.group(2)),
                                "cone": _c.group(1) if _c else None,
                                "slack": float(_c.group(2)) if _c else None},
                    "push": {"end": None, "avance": None},
                    "samples": [], "events": [msg.strip()],
                }
                continue
            if rid not in open_ep:
                continue
            open_ep[rid]["events"].append(msg.strip())
            m = _CONFIRMED.search(msg)
            if m:
                _c = _CONE.search(msg)
                open_ep[rid]["push"] = {"end": "posesion_confirmada", "L": float(m.group(1)),
                                        "D": float(m.group(2)), "held": float(m.group(3)),
                                        "cone": _c.group(1) if _c else None,
                                        "slack": float(_c.group(2)) if _c else None}
                continue
            m = _LOST.search(msg)
            if m:
                _close(rid, "se_fue", t_sec, dist_final=float(m.group(1)))
                continue
            m = _AIMLOST.search(msg)
            if m:
                _close(rid, "tiro_fuera", t_sec, slack=float(m.group(1)))
                continue
            m = _OVERSHOOT.search(msg)
            if m:
                open_ep[rid]["push"] = {"end": "overshoot_ok", "avance": float(m.group(1))}
                continue
            m = _STALL.search(msg)
            if m:
                open_ep[rid]["push"] = {"end": "trabado", "avance": float(m.group(1))}
                continue
            m = _KICK.search(msg)
            if m:
                _close(rid, "kicked", t_sec)
                continue
            m = _ESCAPE.search(msg)
            if m:
                _close(rid, "escaped", t_sec, dist_final=float(m.group(1)))
                continue
            m = _DESCENTRA.search(msg)
            if m:
                _close(rid, "restage_descentrado", t_sec,
                       settle_L=float(m.group(1)), settle_D=float(m.group(2)))
                continue
            if _RESTAGE.search(msg):
                _close(rid, "restage", t_sec)
                continue
            if _POSSESS.search(msg):
                _close(rid, "timeout_posesion", t_sec)
                continue
            continue

        st = _STATUS.search(body)
        if st:
            rid = int(st.group(1))
            if rid in open_ep:
                f = _parse_status_fields(st.group(2))
                if f:
                    f["t_s"] = t_sec
                    f["db_px"] = _num(f.get("db"))
                    open_ep[rid]["samples"].append(f)

    for rid in list(open_ep):
        _close(rid, "unterminated", None)
    return episodes


def _stat(xs):
    return ({"n": len(xs), "min": round(min(xs), 1), "max": round(max(xs), 1),
             "mean": round(sum(xs) / len(xs), 1)} if xs else {"n": 0})


def summarize(episodes: list[dict], db_threshold: int) -> dict:
    by_outcome: dict[str, int] = {}
    by_push: dict[str, int] = {}
    cross: dict[str, dict[str, int]] = {}
    db_max_by_outcome: dict[str, list] = {}

    for ep in episodes:
        oc = ep.get("outcome", "unterminated")
        pe = (ep.get("push") or {}).get("end") or "none"
        by_outcome[oc] = by_outcome.get(oc, 0) + 1
        by_push[pe] = by_push.get(pe, 0) + 1
        cross.setdefault(pe, {}).setdefault(oc, 0)
        cross[pe][oc] += 1
        if ep.get("db_max") is not None:
            db_max_by_outcome.setdefault(oc, []).append(ep["db_max"])

    kept_ok = sum(1 for ep in episodes
                  if ep.get("possession_kept") and ep.get("outcome") == "kicked")
    kept_total = sum(1 for ep in episodes if ep.get("possession_kept"))
    lost_total = sum(1 for ep in episodes if ep.get("possession_kept") is False)
    lost_escaped = sum(1 for ep in episodes
                       if ep.get("possession_kept") is False
                       and ep.get("outcome") in ("escaped", "restage_descentrado"))

    return {
        "n_episodes": len(episodes),
        "db_threshold_px": db_threshold,
        "summary": {
            "by_outcome": by_outcome,
            "by_push_end": by_push,
            "push_end_x_outcome": cross,
            "db_max_by_outcome": {k: _stat(v) for k, v in db_max_by_outcome.items()},
            "possession_rule_check": {
                "rule": f"possession_kept := db_max <= {db_threshold}px",
                "kept_and_kicked": kept_ok,
                "kept_total": kept_total,
                "lost_total": lost_total,
                "lost_and_(escaped|descentrado)": lost_escaped,
            },
            "note": "STATUS ~2 Hz: agregados, no curvas sub-segundo",
        },
        "episodes": episodes,
    }


# compat: el wrapper capture_1robot llama parse(lines) -> dict con summary
def parse(lines, db_threshold=DEFAULT_DB_THRESHOLD) -> dict:
    return summarize(parse_lines(lines, db_threshold), db_threshold)


def main(argv):
    args = [a for a in argv[1:] if not a.startswith("--")]
    db_threshold = DEFAULT_DB_THRESHOLD
    for a in argv[1:]:
        if a.startswith("--db-threshold"):
            db_threshold = int(a.split("=", 1)[1]) if "=" in a else DEFAULT_DB_THRESHOLD
    if not args:
        print(__doc__)
        return 1

    all_eps: list[dict] = []
    for src in args:
        lines = (sys.stdin.readlines() if src == "-"
                 else Path(src).read_text(encoding="utf-8", errors="replace").splitlines())
        all_eps.extend(parse_lines(lines, db_threshold))

    data = summarize(all_eps, db_threshold)
    out = save_metrics("push_episodes", data)
    s = data["summary"]
    print(f"Episodios: {data['n_episodes']}  (db_threshold={db_threshold}px)  ->  {out}")
    print(f"  outcome        : {s['by_outcome']}")
    print(f"  push_end       : {s['by_push_end']}")
    print(f"  push x outcome : {s['push_end_x_outcome']}")
    print(f"  db_max/outcome : {s['db_max_by_outcome']}")
    print(f"  regla posesión : {s['possession_rule_check']}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))
