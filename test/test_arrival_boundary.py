"""Tests headless del borde de llegada a waypoint y su histéresis.

Regresión del atasco observado en el partido 2v2: un robot quedaba congelado a
`distance == position_threshold` exacto (coords de visión enteras: 520-515=5px).
El perfil de velocidad zeroa `v` con `distance <= position_threshold`, pero la
verificación de llegada usaba `distance < threshold` (estricto). En el borde
exacto ninguna rama latcheaba `arrived`: el robot recibía ~0 PWM y nunca
declaraba llegada. Con el fix (`<=`) el borde latchea como llegada.

Verifica además que la histéresis de llegada (mantener motores apagados mientras
`distance < 2× threshold`, liberar al salir de ese rango) sigue intacta tras el
cambio: el borde solo alimenta correctamente el estado que la histéresis maneja.

Sin hardware (rf_controller=None). target_angle=None para que la rama de llegada
ejecute `_send_motor_commands(0, 0)` y podamos leer `_last_pwm`.
"""

import logging

import pytest

from robot_soccer.controllers.differential_drive import DifferentialDriveController


@pytest.fixture(autouse=True)
def _quiet_robot_logger():
    """Silencia el logger de estado (independiza el test del orden del suite)."""
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
        self.angle = angle  # radianes; move_to_position usa robot.angle en atan2
        self.dx = 0.0
        self.dy = 0.0
        self.dw = 0.0


def _make_ctrl_robot():
    """Controlador headless + robot alineado al eje +x (error angular 0 → modo lineal)."""
    ctrl = DifferentialDriveController(rf_controller=None)
    robot = FakeRobot(robot_id=0, x=100.0, y=100.0, angle=0.0)
    return ctrl, robot


# ─────────────────────────────────────────────────────────────────────────────
# El fix: el borde exacto distance == threshold cuenta como llegada
# ─────────────────────────────────────────────────────────────────────────────

def test_arrives_exactly_at_threshold():
    """distance == position_threshold (el caso de R3) → llega, no queda en limbo."""
    ctrl, robot = _make_ctrl_robot()
    thr = ctrl.position_threshold
    target = (robot.x + thr, robot.y)  # distancia EXACTA == threshold, recto al frente
    pid_st = ctrl._get_pid_state(robot.id)

    arrived = ctrl.move_to_position(robot, target)  # target_angle=None

    assert arrived is True            # antes del fix: False (atascado en el borde)
    assert pid_st['arrived'] is True
    assert ctrl._last_pwm[robot.id] == (0, 0)


def test_arrives_below_threshold():
    """No-regresión: por debajo del umbral sigue siendo llegada."""
    ctrl, robot = _make_ctrl_robot()
    thr = ctrl.position_threshold
    target = (robot.x + thr * 0.5, robot.y)  # distancia < threshold

    arrived = ctrl.move_to_position(robot, target)

    assert arrived is True
    assert ctrl._last_pwm[robot.id] == (0, 0)


# ─────────────────────────────────────────────────────────────────────────────
# Histéresis intacta: mantiene dentro de 2× threshold, libera al salir
# ─────────────────────────────────────────────────────────────────────────────

def test_hysteresis_holds_within_2x():
    """Tras llegar, dentro de [threshold, 2× threshold) mantiene motores en 0."""
    ctrl, robot = _make_ctrl_robot()
    thr = ctrl.position_threshold
    target = (robot.x + thr, robot.y)
    pid_st = ctrl._get_pid_state(robot.id)

    assert ctrl.move_to_position(robot, target) is True  # llega y latchea arrived

    # El robot deriva a 1.5× threshold del mismo target (< 2× threshold).
    # Mismo target → no resetea 'arrived' (no es NUEVO TARGET).
    robot.x -= thr * 0.5  # distancia = 1.5× threshold
    held = ctrl.move_to_position(robot, target)

    assert held is True
    assert pid_st['arrived'] is True              # histéresis NO reactiva
    assert ctrl._last_pwm[robot.id] == (0, 0)     # motores siguen apagados


def test_hysteresis_releases_beyond_2x():
    """Tras llegar, más allá de 2× threshold libera 'arrived' y reactiva movimiento."""
    ctrl, robot = _make_ctrl_robot()
    thr = ctrl.position_threshold
    target = (robot.x + thr, robot.y)
    pid_st = ctrl._get_pid_state(robot.id)

    assert ctrl.move_to_position(robot, target) is True

    # El robot se aleja a 3× threshold (>= 2× threshold) del mismo target.
    robot.x -= thr * 2.0  # distancia = 3× threshold
    released = ctrl.move_to_position(robot, target)

    assert released is False           # ya no está "llegado": vuelve a moverse
    assert pid_st['arrived'] is False  # histéresis liberó el latch
