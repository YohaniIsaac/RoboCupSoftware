"""Módulo de calibración individual de motores por robot.

Este módulo permite cargar y guardar factores de calibración únicos para cada robot,
compensando diferencias físicas en motores, fricción, y desbalance.
"""
import json
import logging
from pathlib import Path
from typing import Dict, Tuple

logger = logging.getLogger(__name__)


class RobotCalibration:
    """Gestiona la calibración individual de motores por robot ID."""

    def __init__(self, calibration_file: str = None):
        """Inicializa el sistema de calibración.

        Args:
            calibration_file: Ruta al archivo JSON de calibración.
                             Si es None, usa la ruta por defecto.
        """
        if calibration_file is None:
            # Ruta por defecto
            base_path = Path(__file__).parent.parent / "config"
            calibration_file = base_path / "robot_calibration.json"

        self.calibration_file = Path(calibration_file)
        self.calibrations: Dict[str, dict] = {}
        self.load()

    def load(self):
        """Carga las calibraciones desde el archivo JSON."""
        try:
            if not self.calibration_file.exists():
                logger.warning("Archivo de calibración no encontrado: %s", self.calibration_file)
                logger.info("Usando valores neutros por defecto")
                return

            with open(self.calibration_file, 'r', encoding='utf-8') as f:
                data = json.load(f)

            # Filtrar solo los IDs numéricos (ignorar _comment, _instructions, etc)
            self.calibrations = {
                k: v for k, v in data.items()
                if not k.startswith('_')
            }

            logger.info("Calibraciones cargadas para %d robots", len(self.calibrations))

        except json.JSONDecodeError as e:
            logger.error("Error parseando JSON de calibración: %s", e)
        except Exception as e:
            logger.error("Error cargando calibraciones: %s", e)

    def save(self):
        """Guarda las calibraciones actuales al archivo JSON."""
        try:
            # Leer archivo existente para preservar comentarios
            existing_data = {}
            if self.calibration_file.exists():
                with open(self.calibration_file, 'r', encoding='utf-8') as f:
                    existing_data = json.load(f)

            # Actualizar solo los valores de calibración, mantener _comment, etc
            for robot_id, cal in self.calibrations.items():
                existing_data[robot_id] = cal

            # Guardar con formato bonito
            with open(self.calibration_file, 'w', encoding='utf-8') as f:
                json.dump(existing_data, f, indent=2, ensure_ascii=False)

            logger.info("Calibraciones guardadas en %s", self.calibration_file)

        except Exception as e:
            logger.error("Error guardando calibraciones: %s", e)

    def get_calibration(self, robot_id: int) -> Tuple[float, float, float]:
        """Obtiene la calibración para un robot específico.

        Args:
            robot_id: ID del robot (ArUco marker ID)

        Returns:
            Tupla (max_speed_left, max_speed_right, bias_correction)
            Si no hay calibración, retorna valores neutros (1.0, 1.0, 0.0)
        """
        robot_key = str(robot_id)

        if robot_key not in self.calibrations:
            # logger.debug(f"No hay calibración para robot {robot_id}, usando neutro")
            return (1.0, 1.0, 0.0)

        cal = self.calibrations[robot_key]
        return (
            cal.get('max_speed_left', 1.0),
            cal.get('max_speed_right', 1.0),
            cal.get('bias_correction', 0.0)
        )

    def set_calibration(self, robot_id: int, max_left: float, max_right: float, bias: float):
        """Establece la calibración para un robot.

        Args:
            robot_id: ID del robot
            max_left: Factor motor izquierdo (0.0-1.0)
            max_right: Factor motor derecho (0.0-1.0)
            bias: Corrección de sesgo (-0.3 a 0.3)
        """
        robot_key = str(robot_id)

        # Validar rangos
        max_left = max(0.0, min(1.0, max_left))
        max_right = max(0.0, min(1.0, max_right))
        bias = max(-0.3, min(0.3, bias))

        self.calibrations[robot_key] = {
            'max_speed_left': round(max_left, 3),
            'max_speed_right': round(max_right, 3),
            'bias_correction': round(bias, 3)
        }

        logger.info("Calibración actualizada para robot %d: L=%.3f, R=%.3f, B=%.3f",
                   robot_id, max_left, max_right, bias)

    def apply_calibration(self, robot_id: int, left_speed: int, right_speed: int
                         ) -> Tuple[int, int]:
        """Aplica la calibración a las velocidades de motor.

        Args:
            robot_id: ID del robot
            left_speed: Velocidad motor izquierdo (-255 a 255)
            right_speed: Velocidad motor derecho (-255 a 255)

        Returns:
            Tupla (left_calibrated, right_calibrated) con valores calibrados
        """
        max_left, max_right, bias = self.get_calibration(robot_id)

        # Aplicar factores de velocidad máxima
        left_cal = left_speed * max_left
        right_cal = right_speed * max_right

        # Aplicar corrección de sesgo solo cuando ambos motores van en la misma dirección
        # (movimiento recto adelante/atrás)
        if abs(left_speed - right_speed) < 20:  # Umbral de "movimiento recto"
            bias_value = bias * 255  # Convertir bias normalizado a PWM
            left_cal += bias_value
            right_cal -= bias_value

        # Limitar a rango válido
        left_cal = max(-255, min(255, int(left_cal)))
        right_cal = max(-255, min(255, int(right_cal)))

        return (left_cal, right_cal)

    def reset_robot(self, robot_id: int):
        """Resetea la calibración de un robot a valores neutros."""
        self.set_calibration(robot_id, 1.0, 1.0, 0.0)

    def reset_all(self):
        """Resetea todas las calibraciones a valores neutros."""
        for robot_id in self.calibrations.keys():
            self.reset_robot(int(robot_id))


# Instancia global singleton
_CALIBRATION_MANAGER = None


def get_calibration_manager() -> RobotCalibration:
    """Obtiene la instancia global del gestor de calibración."""
    global _CALIBRATION_MANAGER  # pylint: disable=global-statement
    if _CALIBRATION_MANAGER is None:
        _CALIBRATION_MANAGER = RobotCalibration()
    return _CALIBRATION_MANAGER
