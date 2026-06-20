"""Tests headless del regulador dinámico de velocidad del creep de captura.

Verifica `DifferentialDriveController._regulate_creep_speed` y su integración en
`move_to_position`, sin hardware (rf_controller=None). El regulador recibe el reloj
(`_now`) como parámetro, así que los tests directos son deterministas; los de
integración monkeypatchean `time.monotonic` (que usa el bloque de regulación).

Cubre:
  - entrada al creep (baseline = override estático),
  - ventana abierta → mantiene el cap,
  - rápido (empuja) → baja el cap hasta el piso (coasting),
  - lento/atascado → sube el cap hasta el techo,
  - dentro de banda → mantiene,
  - reset de stuck_boost durante el creep,
  - integración: el regulador corre en creep y el cap baja; stuck_boost queda en 0,
  - flag off → el regulador no corre (creep_pwm None → cae al cap estático),
  - no-regresión: el STUCK sigue subiendo el boost fuera del creep.
"""

import time

import pytest

from robot_soccer.controllers.differential_drive import DifferentialDriveController
from robot_soccer.config import (
    CAPTURE_CREEP_SPEED_PWM,
    CREEP_REGULATOR_WINDOW_S,
    CREEP_TARGET_DISPLACEMENT_MIN_PX,
    CREEP_TARGET_DISPLACEMENT_MAX_PX,
    CREEP_PWM_STEP,
    CREEP_PWM_MIN,
    CREEP_PWM_MAX,
    STUCK_DETECTION_WINDOW_S,
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


def _make_creep_controller():
    """Controlador en modo creep (override = CAPTURE_CREEP_SPEED_PWM), sin RF."""
    ctrl = DifferentialDriveController(rf_controller=None)
    ctrl.max_linear_pwm_override = CAPTURE_CREEP_SPEED_PWM
    return ctrl


# ─────────────────────────────────────────────────────────────────────────────
# Tests directos de _regulate_creep_speed (deterministas vía _now)
# ─────────────────────────────────────────────────────────────────────────────

def test_entry_initializes_baseline():
    ctrl = _make_creep_controller()
    robot = FakeRobot(robot_id=0, x=100.0, y=100.0)
    pid_st = ctrl._get_pid_state(0)

    ctrl._regulate_creep_speed(robot, pid_st, _now=0.0)

    assert pid_st['creep_pwm'] == CAPTURE_CREEP_SPEED_PWM
    assert pid_st['creep_ref_x'] == 100.0
    assert pid_st['creep_ref_y'] == 100.0
    assert pid_st['creep_window_start'] == 0.0


def test_open_window_holds_cap():
    ctrl = _make_creep_controller()
    robot = FakeRobot(robot_id=0, x=0.0, y=0.0)
    pid_st = ctrl._get_pid_state(0)

    ctrl._regulate_creep_speed(robot, pid_st, _now=0.0)  # entrada
    # Movimiento grande pero la ventana aún no cierra → no ajusta.
    robot.x = 500.0
    ctrl._regulate_creep_speed(robot, pid_st, _now=CREEP_REGULATOR_WINDOW_S / 2.0)

    assert pid_st['creep_pwm'] == CAPTURE_CREEP_SPEED_PWM


def test_fast_movement_lowers_cap_to_floor():
    ctrl = _make_creep_controller()
    robot = FakeRobot(robot_id=0, x=0.0, y=0.0)
    pid_st = ctrl._get_pid_state(0)

    ctrl._regulate_creep_speed(robot, pid_st, _now=0.0)  # entrada @20

    # Una ventana rápida: desplazamiento >> MAX → baja un step.
    t = CREEP_REGULATOR_WINDOW_S + 0.001
    robot.x += 100.0
    ctrl._regulate_creep_speed(robot, pid_st, _now=t)
    assert pid_st['creep_pwm'] == CAPTURE_CREEP_SPEED_PWM - CREEP_PWM_STEP

    # Muchas ventanas rápidas → satura en el piso (coasting), sin pasarse.
    for _ in range(40):
        t += CREEP_REGULATOR_WINDOW_S + 0.001
        robot.x += 100.0
        ctrl._regulate_creep_speed(robot, pid_st, _now=t)
    assert pid_st['creep_pwm'] == CREEP_PWM_MIN


def test_slow_movement_raises_cap_to_ceiling():
    ctrl = _make_creep_controller()
    robot = FakeRobot(robot_id=0, x=0.0, y=0.0)
    pid_st = ctrl._get_pid_state(0)

    ctrl._regulate_creep_speed(robot, pid_st, _now=0.0)  # entrada @20

    # Una ventana sin movimiento (< MIN) → sube un step.
    t = CREEP_REGULATOR_WINDOW_S + 0.001
    ctrl._regulate_creep_speed(robot, pid_st, _now=t)
    assert pid_st['creep_pwm'] == CAPTURE_CREEP_SPEED_PWM + CREEP_PWM_STEP

    # Robot atascado muchas ventanas → satura en el techo.
    for _ in range(40):
        t += CREEP_REGULATOR_WINDOW_S + 0.001
        ctrl._regulate_creep_speed(robot, pid_st, _now=t)
    assert pid_st['creep_pwm'] == CREEP_PWM_MAX


def test_in_band_holds_cap():
    ctrl = _make_creep_controller()
    robot = FakeRobot(robot_id=0, x=0.0, y=0.0)
    pid_st = ctrl._get_pid_state(0)

    ctrl._regulate_creep_speed(robot, pid_st, _now=0.0)  # entrada @20

    # Desplazamiento dentro de [MIN, MAX] → mantiene.
    mid = (CREEP_TARGET_DISPLACEMENT_MIN_PX + CREEP_TARGET_DISPLACEMENT_MAX_PX) / 2.0
    robot.x += mid
    ctrl._regulate_creep_speed(robot, pid_st, _now=CREEP_REGULATOR_WINDOW_S + 0.001)

    assert pid_st['creep_pwm'] == CAPTURE_CREEP_SPEED_PWM


def test_resets_stuck_boost_during_creep():
    ctrl = _make_creep_controller()
    robot = FakeRobot(robot_id=0, x=0.0, y=0.0)
    pid_st = ctrl._get_pid_state(0)
    pid_st['stuck_boost'] = 9  # residual de una fase previa

    ctrl._regulate_creep_speed(robot, pid_st, _now=0.0)

    assert pid_st['stuck_boost'] == 0


# ─────────────────────────────────────────────────────────────────────────────
# Tests de integración vía move_to_position (monkeypatch de time.monotonic)
# ─────────────────────────────────────────────────────────────────────────────

def test_creep_regulator_active_via_move_to_position(monkeypatch):
    clock = {'t': 0.0}
    monkeypatch.setattr(time, 'monotonic', lambda: clock['t'])

    ctrl = _make_creep_controller()
    robot = FakeRobot(robot_id=0, x=100.0, y=100.0, angle=0.0)
    target = (1000, 100)  # recto al frente: heading 0, error angular 0 → modo lineal
    pid_st = ctrl._get_pid_state(0)

    ctrl.move_to_position(robot, target)  # entrada: baseline
    assert pid_st['creep_pwm'] == CAPTURE_CREEP_SPEED_PWM

    # Robot avanzando rápido (50px/ventana > MAX) → el cap debe bajar.
    for _ in range(3):
        clock['t'] += CREEP_REGULATOR_WINDOW_S + 0.01
        robot.x += 50.0
        ctrl.move_to_position(robot, target)

    assert pid_st['creep_pwm'] < CAPTURE_CREEP_SPEED_PWM
    assert pid_st['stuck_boost'] == 0  # el regulador inhibe el boost aditivo


def test_flag_off_keeps_static_cap(monkeypatch):
    import robot_soccer.controllers.differential_drive as dd
    monkeypatch.setattr(dd, 'CREEP_REGULATOR_ENABLED', False)

    clock = {'t': 0.0}
    monkeypatch.setattr(time, 'monotonic', lambda: clock['t'])

    ctrl = _make_creep_controller()
    robot = FakeRobot(robot_id=0, x=100.0, y=100.0, angle=0.0)
    pid_st = ctrl._get_pid_state(0)

    for _ in range(3):
        clock['t'] += CREEP_REGULATOR_WINDOW_S + 0.01
        robot.x += 50.0
        ctrl.move_to_position(robot, (1000, 100))

    # Con el flag apagado el regulador nunca corre → creep_pwm queda None y el cap
    # cae al override estático (comportamiento idéntico al previo).
    assert pid_st['creep_pwm'] is None


def test_stuck_detection_unchanged_outside_creep(monkeypatch):
    clock = {'t': 0.0}
    monkeypatch.setattr(time, 'monotonic', lambda: clock['t'])

    ctrl = DifferentialDriveController(rf_controller=None)  # NO creep (override None)
    robot = FakeRobot(robot_id=0, x=100.0, y=100.0, angle=0.0)
    target = (300, 100)  # recto al frente → modo lineal, no rotación pura
    pid_st = ctrl._get_pid_state(0)

    # Robot inmóvil a lo largo de varias ventanas STUCK → el boost debe crecer.
    for _ in range(5):
        ctrl.move_to_position(robot, target)
        clock['t'] += STUCK_DETECTION_WINDOW_S + 0.05

    assert pid_st['stuck_boost'] > 0
    assert pid_st['creep_pwm'] is None  # el regulador no tocó nada fuera del creep
