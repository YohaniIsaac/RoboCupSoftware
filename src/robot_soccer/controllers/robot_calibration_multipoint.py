"""Módulo de calibración simple de motores por robot.

Calibración de 5 valores por robot (en vez de 60+ del sistema multi-punto):
  - 4 dead zones: PWM mínimo per-motor per-dirección
  - 1 bias: corrección de deriva para movimiento recto

Mantiene la misma API pública para compatibilidad con rf_controller y
differential_drive (get_pwm_range, apply_calibration, get_calibration_at_speed).
"""
import json
import logging
from pathlib import Path
from typing import Dict, Tuple

logger = logging.getLogger(__name__)

# Rango PWM por defecto (si no se ha calibrado)
DEFAULT_PWM_MIN = 20
DEFAULT_PWM_MAX = 80


class RobotCalibrationMultipoint:
    """Gestiona calibración simple de motores por robot.

    Formato JSON por robot:
        pwm_min/pwm_max: Rango PWM útil (cámara detecta al robot).
        deadzone_left_fwd/rev, deadzone_right_fwd/rev: Arranque per-motor.
        bias: Corrección de deriva para movimiento recto.
    """

    def __init__(self, calibration_file: str = None):
        if calibration_file is None:
            base_path = Path(__file__).parent.parent / "config"
            calibration_file = base_path / "robot_calibration_multipoint.json"

        self.calibration_file = Path(calibration_file)
        self.calibrations: Dict[str, dict] = {}
        self.load()

    def load(self):
        """Carga calibraciones desde JSON (soporta formato simple y legacy)."""
        try:
            if not self.calibration_file.exists():
                logger.warning("Archivo de calibración no encontrado: %s",
                               self.calibration_file)
                self._create_default_file()
                return

            with open(self.calibration_file, 'r', encoding='utf-8') as f:
                data = json.load(f)

            for robot_id, cal_data in data.items():
                if robot_id.startswith('_'):
                    continue
                if not isinstance(cal_data, dict):
                    continue

                # Detectar formato: simple (tiene 'bias' directo) vs legacy (tiene 'calibration_points')
                if 'calibration_points' in cal_data:
                    # MIGRACIÓN: formato legacy multi-punto → simple
                    points = cal_data['calibration_points']
                    fwd_biases = [p['bias'] for p in points if p.get('pwm', 0) > 0]
                    avg_bias = sum(fwd_biases) / len(fwd_biases) if fwd_biases else 0.0

                    self.calibrations[robot_id] = {
                        'pwm_min': cal_data.get('pwm_min', DEFAULT_PWM_MIN),
                        'pwm_max': cal_data.get('pwm_max', DEFAULT_PWM_MAX),
                        'deadzone_left_fwd': cal_data.get('deadzone_left', 0),
                        'deadzone_left_rev': cal_data.get('deadzone_left', 0),
                        'deadzone_right_fwd': cal_data.get('deadzone_right', 0),
                        'deadzone_right_rev': cal_data.get('deadzone_right', 0),
                        'bias': round(avg_bias, 4),
                    }
                    logger.info("Robot %s: migrado de multi-punto → simple (bias=%.4f)",
                                robot_id, avg_bias)
                else:
                    # Formato simple directo
                    self.calibrations[robot_id] = {
                        'pwm_min': cal_data.get('pwm_min', DEFAULT_PWM_MIN),
                        'pwm_max': cal_data.get('pwm_max', DEFAULT_PWM_MAX),
                        'deadzone_left_fwd': cal_data.get('deadzone_left_fwd', 0),
                        'deadzone_left_rev': cal_data.get('deadzone_left_rev', 0),
                        'deadzone_right_fwd': cal_data.get('deadzone_right_fwd', 0),
                        'deadzone_right_rev': cal_data.get('deadzone_right_rev', 0),
                        'bias': cal_data.get('bias', 0.0),
                    }

                cal = self.calibrations[robot_id]
                logger.info("Robot %s: [%d-%d] PWM, bias=%.4f, dz=[LF:%d LR:%d RF:%d RR:%d]",
                            robot_id, cal['pwm_min'], cal['pwm_max'], cal['bias'],
                            cal['deadzone_left_fwd'], cal['deadzone_left_rev'],
                            cal['deadzone_right_fwd'], cal['deadzone_right_rev'])

            logger.info("Calibraciones cargadas para %d robots", len(self.calibrations))

        except (json.JSONDecodeError, Exception) as e:
            logger.error("Error cargando calibraciones: %s", e)

    def _create_default_file(self):
        """Crea archivo de calibración con valores por defecto."""
        default_data = {
            "_comment": "Calibración simple: dead zone per-motor per-direction + bias",
        }
        for i in range(4):
            default_data[str(i)] = {
                "pwm_min": DEFAULT_PWM_MIN,
                "pwm_max": DEFAULT_PWM_MAX,
                "deadzone_left_fwd": 0, "deadzone_left_rev": 0,
                "deadzone_right_fwd": 0, "deadzone_right_rev": 0,
                "bias": 0.0,
            }

        with open(self.calibration_file, 'w', encoding='utf-8') as f:
            json.dump(default_data, f, indent=2, ensure_ascii=False)
        logger.info("Archivo de calibración creado: %s", self.calibration_file)
        self.load()

    def save(self):
        """Guarda calibraciones al archivo JSON."""
        try:
            # Preservar campos de comentario del archivo existente
            existing = {}
            if self.calibration_file.exists():
                with open(self.calibration_file, 'r', encoding='utf-8') as f:
                    existing = json.load(f)

            for robot_id, cal in self.calibrations.items():
                existing[robot_id] = {
                    'pwm_min': cal['pwm_min'],
                    'pwm_max': cal['pwm_max'],
                    'deadzone_left_fwd': cal['deadzone_left_fwd'],
                    'deadzone_left_rev': cal['deadzone_left_rev'],
                    'deadzone_right_fwd': cal['deadzone_right_fwd'],
                    'deadzone_right_rev': cal['deadzone_right_rev'],
                    'bias': round(cal['bias'], 5),
                }

            with open(self.calibration_file, 'w', encoding='utf-8') as f:
                json.dump(existing, f, indent=2, ensure_ascii=False)
            logger.info("Calibraciones guardadas en %s", self.calibration_file)

        except Exception as e:
            logger.error("Error guardando calibraciones: %s", e)

    # ===== API PÚBLICA (compatible con consumers existentes) =====

    def get_pwm_range(self, robot_id: int) -> Tuple[int, int]:
        """Obtiene el rango PWM útil de un robot."""
        robot_key = str(robot_id)
        if robot_key not in self.calibrations:
            return (DEFAULT_PWM_MIN, DEFAULT_PWM_MAX)
        cal = self.calibrations[robot_key]
        return (cal.get('pwm_min', DEFAULT_PWM_MIN),
                cal.get('pwm_max', DEFAULT_PWM_MAX))

    def set_pwm_range(self, robot_id: int, pwm_min: int, pwm_max: int):
        """Establece el rango PWM útil de un robot."""
        robot_key = str(robot_id)
        if pwm_min >= pwm_max:
            logger.error("pwm_min (%d) debe ser < pwm_max (%d)", pwm_min, pwm_max)
            return
        if robot_key not in self.calibrations:
            self._init_robot(robot_id)
        self.calibrations[robot_key]['pwm_min'] = pwm_min
        self.calibrations[robot_key]['pwm_max'] = pwm_max
        logger.info("Robot %d: rango PWM → [%d, %d]", robot_id, pwm_min, pwm_max)

    def get_bias(self, robot_id: int) -> float:
        """Obtiene el bias de corrección lineal."""
        robot_key = str(robot_id)
        if robot_key not in self.calibrations:
            return 0.0
        return self.calibrations[robot_key].get('bias', 0.0)

    def set_bias(self, robot_id: int, bias: float):
        """Establece el bias de corrección lineal."""
        robot_key = str(robot_id)
        bias = max(-0.5, min(0.5, bias))
        if robot_key not in self.calibrations:
            self._init_robot(robot_id)
        self.calibrations[robot_key]['bias'] = bias
        logger.info("Robot %d: bias → %.4f", robot_id, bias)

    def get_deadzone(self, robot_id: int) -> Tuple[int, int]:
        """Obtiene dead-zone (máximo de fwd/rev por lado, para compatibilidad)."""
        robot_key = str(robot_id)
        if robot_key not in self.calibrations:
            return (0, 0)
        cal = self.calibrations[robot_key]
        dz_left = max(cal.get('deadzone_left_fwd', 0), cal.get('deadzone_left_rev', 0))
        dz_right = max(cal.get('deadzone_right_fwd', 0), cal.get('deadzone_right_rev', 0))
        return (dz_left, dz_right)

    def get_deadzone_detailed(self, robot_id: int) -> Tuple[int, int, int, int]:
        """Obtiene dead-zone detallado (4 valores)."""
        robot_key = str(robot_id)
        if robot_key not in self.calibrations:
            return (0, 0, 0, 0)
        cal = self.calibrations[robot_key]
        return (cal.get('deadzone_left_fwd', 0), cal.get('deadzone_left_rev', 0),
                cal.get('deadzone_right_fwd', 0), cal.get('deadzone_right_rev', 0))

    def set_deadzone(self, robot_id: int, deadzone_left: int, deadzone_right: int):
        """Establece dead-zone (mismo valor fwd/rev, para compatibilidad)."""
        robot_key = str(robot_id)
        if robot_key not in self.calibrations:
            self._init_robot(robot_id)
        cal = self.calibrations[robot_key]
        cal['deadzone_left_fwd'] = max(0, min(127, deadzone_left))
        cal['deadzone_left_rev'] = max(0, min(127, deadzone_left))
        cal['deadzone_right_fwd'] = max(0, min(127, deadzone_right))
        cal['deadzone_right_rev'] = max(0, min(127, deadzone_right))

    def set_deadzone_detailed(self, robot_id: int,
                              lf: int, lr: int, rf: int, rr: int):
        """Establece dead-zone detallado (4 valores independientes)."""
        robot_key = str(robot_id)
        if robot_key not in self.calibrations:
            self._init_robot(robot_id)
        cal = self.calibrations[robot_key]
        cal['deadzone_left_fwd'] = max(0, min(127, lf))
        cal['deadzone_left_rev'] = max(0, min(127, lr))
        cal['deadzone_right_fwd'] = max(0, min(127, rf))
        cal['deadzone_right_rev'] = max(0, min(127, rr))

    def get_calibration_at_speed(self, robot_id: int, speed: int
                                 ) -> Tuple[float, float, float]:
        """Retorna (max_left, max_right, bias) — compatible con consumers.

        En calibración simple, max_left y max_right son siempre 1.0.
        Solo el bias varía por robot.
        """
        bias = self.get_bias(robot_id)
        return (1.0, 1.0, bias)

    def apply_calibration(self, robot_id: int, left_speed: int, right_speed: int
                          ) -> Tuple[int, int]:
        """Aplica corrección de bias para movimiento recto.

        Solo modifica los comandos cuando ambas ruedas van en la misma dirección
        (movimiento lineal). Para rotación, pasa los valores sin modificar.
        """
        bias = self.get_bias(robot_id)

        left_cal = float(left_speed)
        right_cal = float(right_speed)

        # Bias solo para movimiento recto (misma dirección, velocidades similares)
        if abs(left_speed - right_speed) < 20 and left_speed * right_speed > 0:
            bias_value = bias * 127
            left_cal += bias_value
            right_cal -= bias_value

        left_cal = max(-127, min(127, int(left_cal)))
        right_cal = max(-127, min(127, int(right_cal)))
        return (left_cal, right_cal)

    def _init_robot(self, robot_id: int):
        """Inicializa calibración para un robot con valores neutros."""
        self.calibrations[str(robot_id)] = {
            'pwm_min': DEFAULT_PWM_MIN, 'pwm_max': DEFAULT_PWM_MAX,
            'deadzone_left_fwd': 0, 'deadzone_left_rev': 0,
            'deadzone_right_fwd': 0, 'deadzone_right_rev': 0,
            'bias': 0.0,
        }

    def reset_robot(self, robot_id: int):
        """Resetea la calibración de un robot a valores neutros."""
        self._init_robot(robot_id)


# Instancia global singleton
_CALIBRATION_MANAGER_MULTIPOINT = None


def get_calibration_manager_multipoint() -> RobotCalibrationMultipoint:
    """Obtiene la instancia global del gestor de calibración."""
    global _CALIBRATION_MANAGER_MULTIPOINT  # pylint: disable=global-statement
    if _CALIBRATION_MANAGER_MULTIPOINT is None:
        _CALIBRATION_MANAGER_MULTIPOINT = RobotCalibrationMultipoint()
    return _CALIBRATION_MANAGER_MULTIPOINT
