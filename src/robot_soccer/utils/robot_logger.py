"""Logger unificado para datos de estado de robots.

Proporciona un formato fijo de columnas para todos los logs periódicos de robot,
independientemente del estado en que se encuentre (rotando, avanzando, etc.).

Uso básico:
    from robot_soccer.utils.robot_logger import robot_status_logger

    # Actualizar campos (cada módulo aporta su slice de datos):
    robot_status_logger.update(robot_id, state="move_behind_ball", ang=128.3)
    robot_status_logger.update(robot_id, left_pwm=-19, right_pwm=19)

    # Emitir línea [STATUS] (solo desde el loop principal, ~2 Hz):
    robot_status_logger.emit(robot_id)

    # Emitir evento puntual:
    robot_status_logger.emit_event(robot_id, "FASE INIT: advancing_to_contact")

Para OCULTAR un campo: cambiar enabled=False en FIELD_DEFS (un solo lugar).
Para AÑADIR un campo:
    1. Añadir atributo Optional a RobotStatus
    2. Añadir FieldDef a FIELD_DEFS con label y formatter
    3. Llamar update(robot_id, nuevo_campo=valor) donde aplique
    Sin cambios en ningún otro archivo.
"""

from __future__ import annotations

import logging
from dataclasses import dataclass
from typing import Callable, Optional

log = logging.getLogger(__name__)


# ─── Formatters reutilizables ─────────────────────────────────────────────────

def _fmt_angle(v: Optional[float]) -> str:
    """Ángulo con signo, 7 chars + '°'. Ej: ' +128.3°' o '    ---°'."""
    return f"{v:+7.1f}°" if v is not None else "    ---°"


def _fmt_pos(v: Optional[tuple]) -> str:
    """Posición (x, y), 9 chars. Ej: '(275,210)' o '(---,---)'."""
    return f"({v[0]:3d},{v[1]:3d})" if v is not None else "(---,---)"


def _fmt_dist(v: Optional[float]) -> str:
    """Distancia en píxeles, 6 chars. Ej: '  93px' o ' ---px'."""
    return f"{v:4.0f}px" if v is not None else " ---px"


def _fmt_pwm(v: Optional[int]) -> str:
    """PWM con signo, 4 chars. Ej: ' -19' o ' ---'."""
    return f"{v:+4d}" if v is not None else " ---"


def _fmt_stuck(v: Optional[int]) -> str:
    """Boost anti-atasco, 3 chars. Ej: '  0', '  5', ' 12' o '---'."""
    return f"{v:3d}" if v is not None else "---"


def _fmt_count(v: Optional[int]) -> str:
    """Contador genérico, 3 chars. Ej: '  7', '  0' o '---'."""
    return f"{v:3d}" if v is not None else "---"


def _fmt_state(v: Optional[str]) -> str:
    """Estado alineado a la izquierda, 22 chars."""
    return f"{(v or '---'):<22s}"


def _fmt_lat(v: Optional[float]) -> str:
    """Desvío lateral con signo respecto a la nariz, px. Ej: ' -3.7px' o ' ---px'."""
    return f"{v:+5.1f}px" if v is not None else "  ---px"


def _fmt_drb(v: Optional[str]) -> str:
    """Estado del dribbler, 4 chars. Ej: ' C50', ' H30', ' off' o '  --' (desenganchado)."""
    return f"{(v or '--'):>4s}"


# ─── Definición de campos ─────────────────────────────────────────────────────

@dataclass
class FieldDef:
    """Descriptor de un campo del STATUS. Un campo = un FieldDef."""
    key: str            # nombre del atributo en RobotStatus
    label: str          # prefijo visible en el log (ej: "ang=")
    formatter: Callable # (value) → str de ancho fijo
    enabled: bool = True  # False = columna ausente de la línea (no muestra '---')


# ─── CONFIGURACIÓN DE CAMPOS ──────────────────────────────────────────────────
# Para ocultar un campo: enabled=False
# Para añadir un campo: nueva FieldDef + atributo en RobotStatus
FIELD_DEFS: list[FieldDef] = [
    FieldDef("state",     "state=",  _fmt_state, enabled=True),
    FieldDef("ang",       "ang=",    _fmt_angle, enabled=True),
    FieldDef("tgt_ang",   "tgt=",    _fmt_angle, enabled=True),
    FieldDef("err_ang",   "Da=",     _fmt_angle, enabled=True),
    FieldDef("pos",       "pos=",    _fmt_pos,   enabled=True),
    FieldDef("tgt_pos",   "-> ",     _fmt_pos,   enabled=True),
    FieldDef("dist",      "d=",      _fmt_dist,  enabled=True),
    FieldDef("ball_err",  "Dball=",  _fmt_angle, enabled=True),
    FieldDef("goal_err",  "Dgoal=",  _fmt_angle, enabled=True),
    FieldDef("ball_dist", "db=",     _fmt_dist,  enabled=True),
    FieldDef("kick_lat",  "lat=",    _fmt_lat,   enabled=True),
    FieldDef("dribbler",  "drb=",    _fmt_drb,   enabled=True),
    FieldDef("stuck_boost", "sb=",  _fmt_stuck, enabled=True),
    FieldDef("creep_pwm",   "cv=",  _fmt_count, enabled=True),
    FieldDef("left_pwm",  "L=",      _fmt_pwm,   enabled=True),
    FieldDef("right_pwm", "R=",      _fmt_pwm,   enabled=True),
    FieldDef("rrt_len",   "rrt=",    _fmt_count, enabled=True),
    FieldDef("n_obs",     "obs=",    _fmt_count, enabled=True),
]


# ─── Estado por robot ─────────────────────────────────────────────────────────

@dataclass
class RobotStatus:
    """Estado acumulado de un robot. Cada módulo actualiza su slice vía update()."""
    state:     str            = "---"
    ang:       Optional[float] = None  # angulo actual del robot (grados)
    tgt_ang:   Optional[float] = None  # angulo objetivo (grados), None si no aplica
    err_ang:   Optional[float] = None  # error angular (grados)
    pos:       Optional[tuple] = None  # posicion actual (x, y) en pixeles
    tgt_pos:   Optional[tuple] = None  # posicion objetivo (x, y) en pixeles
    dist:      Optional[float] = None  # distancia al target en pixeles
    ball_err:  Optional[float] = None  # error angular hacia la pelota (grados)
    goal_err:  Optional[float] = None  # error angular hacia el arco (grados)
    ball_dist: Optional[float] = None  # distancia robot→pelota (px) — diagnóstico de captura
    kick_lat:  Optional[float] = None  # desvío lateral pelota↔nariz con signo (px), mismo signo que Dball
    dribbler:  Optional[str]   = None  # estado dribbler: 'C50' captura / 'H30' sostén / 'off' pulso / None desenganchado
    stuck_boost: Optional[int]  = None  # boost anti-atasco activo (0=libre, max=kick)
    creep_pwm:  Optional[int]   = None  # cap de velocidad regulado del creep de captura (PWM)
    left_pwm:  Optional[int]   = None  # PWM motor izquierdo (-255..255)
    right_pwm: Optional[int]   = None  # PWM motor derecho (-255..255)
    rrt_len:   Optional[int]   = None  # waypoints restantes en path actual (0=PID directo)
    n_obs:     Optional[int]   = None  # obstáculos que ve el planner de este robot


# ─── Logger central ───────────────────────────────────────────────────────────

class RobotStatusLogger:
    """Logger centralizado con formato unificado para todos los robots.

    Uso tipico:
        robot_status_logger.update(robot_id, ang=128.3, left_pwm=-19, ...)
        robot_status_logger.emit(robot_id)  # cada ~0.5s desde el loop principal
    """

    def __init__(self) -> None:
        self._status: dict[int, RobotStatus] = {}

    def update(self, robot_id: int, **kwargs) -> None:
        """Actualizar uno o varios campos del estado del robot.

        Los campos desconocidos se ignoran silenciosamente.
        Llamar desde cualquier modulo (controlador, behavior, loop principal).
        """
        if robot_id not in self._status:
            self._status[robot_id] = RobotStatus()
        s = self._status[robot_id]
        for k, v in kwargs.items():
            if hasattr(s, k):
                setattr(s, k, v)

    def emit(self, robot_id: int) -> None:
        """Emitir una linea [STATUS] con todos los campos habilitados.

        Solo llamar desde el loop principal (~2 Hz). Los campos con valor None
        se muestran como '---' (dato no disponible en este estado).
        Los campos con enabled=False no aparecen en absoluto.
        """
        if robot_id not in self._status:
            return
        s = self._status[robot_id]
        parts = [f"R{robot_id}"]
        for fd in FIELD_DEFS:
            if not fd.enabled:
                continue
            val = getattr(s, fd.key, None)
            parts.append(f"{fd.label}{fd.formatter(val)}")
        log.info("[STATUS] %s", " | ".join(parts))

    def emit_event(self, robot_id: int, event: str) -> None:
        """Emitir un evento puntual [EVENT ] (transicion de fase, contacto, etc.)."""
        log.info("[EVENT ] R%d | %s", robot_id, event)


# Instancia global — importar y usar directamente
robot_status_logger = RobotStatusLogger()
