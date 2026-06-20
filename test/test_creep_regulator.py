"""Tests headless del regulador de velocidad del creep de captura.

Verifica:
  - Regulador por cámara (`_regulate_creep_speed`): el PWM base sube cuando el robot se
    mueve menos del objetivo (no detectable) y baja cuando se mueve de más (empujaría),
    acotado a [CREEP_BASE_MIN_PWM, ceiling]. Es el lazo cerrado sobre el desplazamiento
    real inter-frame ("acercarse lo más lento que la cámara aún detecte").
  - Desaturado inferior (anti-pivote): a velocidad base baja, la corrección ω no deja
    una rueda bajo el piso de movimiento — ambas giran y el ángulo se corrige (recto,
    no pivote).
  - Flag off → cap estático CAPTURE_CREEP_SPEED_PWM (comportamiento previo).
  - No-regresión: fuera del creep el STUCK normal sigue subiendo hasta STUCK_BOOST_MAX.

Sin hardware (rf_controller=None). Los tests monkeypatchean `time.monotonic` (que usa el
regulador y el detector de atasco) para avanzar ventanas de forma determinista.
"""

import logging
import time

import pytest

import robot_soccer.controllers.differential_drive as dd
from robot_soccer.controllers.differential_drive import DifferentialDriveController
from robot_soccer.ai.behavior_tree.soccer_behaviors import _creep_ceiling_pwm
import robot_soccer.ai.behavior_tree.soccer_behaviors as sb
from robot_soccer.config import (
    CAPTURE_CREEP_SPEED_PWM,
    CREEP_BASE_MIN_PWM,
    CREEP_BASE_MAX_PWM,
    CREEP_REG_WINDOW_S,
    STUCK_DETECTION_WINDOW_S,
    STUCK_BOOST_MAX,
)


@pytest.fixture(autouse=True)
def _quiet_robot_logger():
    """Silencia el logger de estado durante estos tests.

    Otros tests del suite (test_behavior_commands, test_boardgame) reconfiguran el
    logging global con `logging.basicConfig` a nivel de módulo; eso hace que los
    eventos de `robot_status_logger` (emit/emit_event) se rendericen y, por una doble
    formateo de esos handlers, revienten. Estos tests no asertan sobre logs, así que
    bajar el nivel del logger los vuelve independientes del orden de ejecución.
    """
    lg = logging.getLogger("robot_soccer.utils.robot_logger")
    prev = lg.level
    lg.setLevel(logging.WARNING)
    yield
    lg.setLevel(prev)


class FakeRobot:
    """Robot mínimo con los atributos que lee el controlador."""

    def __init__(self, robot_id=0, x=0.0, y=0.0, angle=0.0):
        self.id = robot_id
        self.x = x
        self.y = y
        self.angle = angle  # radianes (move_to_position usa robot.angle en atan2)
        self.dx = 0.0
        self.dy = 0.0
        self.dw = 0.0


# ─────────────────────────────────────────────────────────────────────────────
# Ceiling del creep (helper del comportamiento)
# ─────────────────────────────────────────────────────────────────────────────

def test_ceiling_is_base_max_when_regulator_on():
    assert _creep_ceiling_pwm() == CREEP_BASE_MAX_PWM


def test_ceiling_is_static_cap_when_regulator_off(monkeypatch):
    monkeypatch.setattr(sb, 'CREEP_REGULATOR_ENABLED', False)
    assert _creep_ceiling_pwm() == CAPTURE_CREEP_SPEED_PWM


# ─────────────────────────────────────────────────────────────────────────────
# Regulador por cámara (vía move_to_position, monkeypatch de time.monotonic)
# ─────────────────────────────────────────────────────────────────────────────

def test_regulator_raises_base_when_not_moving(monkeypatch):
    clock = {'t': 0.0}
    monkeypatch.setattr(time, 'monotonic', lambda: clock['t'])

    ctrl = DifferentialDriveController(rf_controller=None)
    ctrl.max_linear_pwm_override = CREEP_BASE_MAX_PWM  # creep mode (ceiling/gate)
    robot = FakeRobot(robot_id=0, x=100.0, y=100.0, angle=0.0)
    target = (300, 100)  # recto al frente y lejos → modo lineal, no llega
    pid_st = ctrl._get_pid_state(0)

    ctrl.move_to_position(robot, target)  # inicializa: base = mínimo
    assert pid_st['creep_base_pwm'] == CREEP_BASE_MIN_PWM

    # Robot quieto a lo largo de varias ventanas → la cámara no detecta movimiento
    # → el base sube hasta el ceiling.
    for _ in range(12):
        clock['t'] += CREEP_REG_WINDOW_S + 0.05
        ctrl.move_to_position(robot, target)

    assert pid_st['creep_base_pwm'] == CREEP_BASE_MAX_PWM


def test_regulator_lowers_base_when_moving_fast(monkeypatch):
    clock = {'t': 0.0}
    monkeypatch.setattr(time, 'monotonic', lambda: clock['t'])

    ctrl = DifferentialDriveController(rf_controller=None)
    ctrl.max_linear_pwm_override = CREEP_BASE_MAX_PWM
    robot = FakeRobot(robot_id=0, x=100.0, y=100.0, angle=0.0)
    target = (2000, 100)  # muy lejos → nunca llega mientras avanza
    pid_st = ctrl._get_pid_state(0)

    ctrl.move_to_position(robot, target)        # activa el creep
    pid_st['creep_base_pwm'] = CREEP_BASE_MAX_PWM  # partir del techo para verlo bajar

    # Desplazamiento grande por ventana (> objetivo+banda) → el base baja al mínimo.
    for _ in range(12):
        clock['t'] += CREEP_REG_WINDOW_S + 0.05
        robot.x += 20.0
        ctrl.move_to_position(robot, target)

    assert pid_st['creep_base_pwm'] == CREEP_BASE_MIN_PWM


def test_bottom_desaturation_keeps_both_wheels_alive():
    """Con base baja y error de rumbo, la rueda interior no cae bajo el piso (no pivote)."""
    ctrl = DifferentialDriveController(rf_controller=None)
    ctrl.max_linear_pwm_override = CREEP_BASE_MAX_PWM  # creep mode
    # Pequeño error angular (~8.5°, < umbral de rotación 30°) → modo lineal con ω≠0.
    robot = FakeRobot(robot_id=0, x=0.0, y=0.0, angle=0.0)
    target = (200, 30)

    ctrl.move_to_position(robot, target)
    left, right = ctrl._last_pwm[0]

    # Ambas ruedas por encima del piso de movimiento (ninguna muerta).
    assert min(left, right) >= CREEP_BASE_MIN_PWM
    # La corrección angular se conserva (diferencial ≠ 0): corrige sin pivotear.
    assert left != right


def test_flag_off_uses_static_cap(monkeypatch):
    """Con el regulador apagado, el cap es el estático y no hay desaturado inferior."""
    monkeypatch.setattr(dd, 'CREEP_REGULATOR_ENABLED', False)

    ctrl = DifferentialDriveController(rf_controller=None)
    ctrl.max_linear_pwm_override = CAPTURE_CREEP_SPEED_PWM  # cap estático previo
    robot = FakeRobot(robot_id=0, x=0.0, y=0.0, angle=0.0)
    target = (200, 0)  # alineado → ω≈0 → ambas ruedas al cap estático

    ctrl.move_to_position(robot, target)
    left, right = ctrl._last_pwm[0]

    assert left == CAPTURE_CREEP_SPEED_PWM
    assert right == CAPTURE_CREEP_SPEED_PWM


def test_stuck_detection_unchanged_outside_creep(monkeypatch):
    clock = {'t': 0.0}
    monkeypatch.setattr(time, 'monotonic', lambda: clock['t'])

    ctrl = DifferentialDriveController(rf_controller=None)  # NO creep (override None)
    robot = FakeRobot(robot_id=0, x=100.0, y=100.0, angle=0.0)
    target = (300, 100)
    pid_st = ctrl._get_pid_state(0)

    for _ in range(10):
        ctrl.move_to_position(robot, target)
        clock['t'] += STUCK_DETECTION_WINDOW_S + 0.05

    assert pid_st['stuck_boost'] == STUCK_BOOST_MAX  # parámetros STUCK normales intactos
