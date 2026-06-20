"""Tests headless de la desaceleración predictiva del creep de captura.

Verifica:
  - `_creep_cap_for_distance` (rampa por distancia robot↔pelota): cap alto lejos,
    mínimo gentil al contacto, monótona, y fallback al cap estático con el flag off.
  - El anti-atasco del creep en `move_to_position`: con cap activo y robot quieto,
    el boost sube hasta CREEP_STALL_BOOST_MAX (no STUCK_BOOST_MAX) — garantiza
    "lento pero sin parar" sin embestir.
  - No-regresión: fuera del creep el STUCK sigue subiendo hasta STUCK_BOOST_MAX.

Sin hardware (rf_controller=None). Los tests de integración monkeypatchean
`time.monotonic` (que usa el bloque de detección de atasco).
"""

import time

import pytest

from robot_soccer.controllers.differential_drive import DifferentialDriveController
import robot_soccer.ai.behavior_tree.soccer_behaviors as sb
from robot_soccer.ai.behavior_tree.soccer_behaviors import _creep_cap_for_distance
from robot_soccer.config import (
    CAPTURE_CREEP_SPEED_PWM,
    CREEP_DECEL_START_DIST_PX,
    CREEP_DECEL_END_DIST_PX,
    CREEP_PWM_FAR,
    CREEP_PWM_NEAR,
    CREEP_STALL_WINDOW_S,
    CREEP_STALL_BOOST_MAX,
    STUCK_DETECTION_WINDOW_S,
    STUCK_BOOST_MAX,
)


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
# Rampa de desaceleración por distancia (_creep_cap_for_distance)
# ─────────────────────────────────────────────────────────────────────────────

def test_cap_far_is_fast():
    assert _creep_cap_for_distance(CREEP_DECEL_START_DIST_PX + 50) == CREEP_PWM_FAR


def test_cap_at_contact_is_gentle():
    assert _creep_cap_for_distance(CREEP_DECEL_END_DIST_PX - 5) == CREEP_PWM_NEAR


def test_cap_decreases_as_robot_approaches():
    far = _creep_cap_for_distance(CREEP_DECEL_START_DIST_PX)
    mid = _creep_cap_for_distance(
        (CREEP_DECEL_START_DIST_PX + CREEP_DECEL_END_DIST_PX) / 2.0)
    near = _creep_cap_for_distance(CREEP_DECEL_END_DIST_PX)
    # Más cerca → más lento (cap menor), y el punto medio queda dentro del rango.
    assert near <= mid <= far
    assert near == CREEP_PWM_NEAR and far == CREEP_PWM_FAR
    assert CREEP_PWM_NEAR < mid < CREEP_PWM_FAR


def test_cap_flag_off_uses_static(monkeypatch):
    monkeypatch.setattr(sb, 'CREEP_REGULATOR_ENABLED', False)
    # Con el flag apagado, a cualquier distancia devuelve el cap estático previo.
    assert _creep_cap_for_distance(CREEP_DECEL_END_DIST_PX) == CAPTURE_CREEP_SPEED_PWM
    assert _creep_cap_for_distance(CREEP_DECEL_START_DIST_PX + 50) == CAPTURE_CREEP_SPEED_PWM


# ─────────────────────────────────────────────────────────────────────────────
# Anti-atasco en el creep (vía move_to_position, monkeypatch de time.monotonic)
# ─────────────────────────────────────────────────────────────────────────────

def test_creep_antistall_boosts_to_creep_max(monkeypatch):
    clock = {'t': 0.0}
    monkeypatch.setattr(time, 'monotonic', lambda: clock['t'])

    ctrl = DifferentialDriveController(rf_controller=None)
    ctrl.max_linear_pwm_override = CREEP_PWM_NEAR  # creep mode activo
    robot = FakeRobot(robot_id=0, x=100.0, y=100.0, angle=0.0)
    target = (300, 100)  # recto al frente → modo lineal, no rotación
    pid_st = ctrl._get_pid_state(0)

    # Robot atascado a lo largo de varias ventanas de creep → el boost sube,
    # pero acotado a CREEP_STALL_BOOST_MAX (no embiste).
    for _ in range(15):
        ctrl.move_to_position(robot, target)
        clock['t'] += CREEP_STALL_WINDOW_S + 0.05

    assert pid_st['stuck_boost'] == CREEP_STALL_BOOST_MAX
    assert CREEP_STALL_BOOST_MAX < STUCK_BOOST_MAX  # acotado por debajo del STUCK normal


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
