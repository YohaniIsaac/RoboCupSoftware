"""Módulo de calibración multi-punto de motores por robot.

Este módulo permite calibración no-lineal usando múltiples puntos de calibración
y compensación individual de dead-zone por motor.
"""
import json
import logging
from pathlib import Path
from typing import Dict, Tuple, List, Optional

logger = logging.getLogger(__name__)

# Puntos de calibración predefinidos (PWM) - SOLO PARA FALLBACK
# Calibración bidireccional: adelante y atrás por separado
# Cada rueda usa su calibración según su dirección de giro
# NOTA: Estos puntos son solo para retrocompatibilidad.
#       Cada robot debería tener su rango personalizado (pwm_min, pwm_max)
DEFAULT_CALIBRATION_POINTS = [-80, -65, -50, -35, -20, 20, 35, 50, 65, 80]

# Rango PWM por defecto (si no se ha determinado con find_pwm_range)
DEFAULT_PWM_MIN = 20
DEFAULT_PWM_MAX = 80


def generate_calibration_points(pwm_min: int, pwm_max: int, num_points: int = 5) -> List[int]:
    """Genera puntos de calibración uniformemente distribuidos en el rango útil.

    Genera num_points para adelante y num_points para atrás (total 2*num_points).

    Args:
        pwm_min: PWM mínimo útil (positivo, ej: 15)
        pwm_max: PWM máximo útil (positivo, ej: 65)
        num_points: Número de puntos por dirección (default: 5)

    Returns:
        Lista de puntos PWM ordenados: [negativo_max, ..., negativo_min, positivo_min, ..., positivo_max]

    Example:
        >>> generate_calibration_points(15, 60, 5)
        [-60, -48, -37, -26, -15, 15, 26, 37, 48, 60]
    """
    if pwm_min >= pwm_max:
        raise ValueError(f"pwm_min ({pwm_min}) debe ser menor que pwm_max ({pwm_max})")

    if num_points < 2:
        raise ValueError(f"num_points debe ser al menos 2, recibido: {num_points}")

    # Generar puntos positivos (adelante) uniformemente espaciados
    positive_points = []
    step = (pwm_max - pwm_min) / (num_points - 1)
    for i in range(num_points):
        point = int(round(pwm_min + i * step))
        positive_points.append(point)

    # Generar puntos negativos (atrás) - mismos valores pero negativos, en orden inverso
    negative_points = [-p for p in reversed(positive_points)]

    # Combinar: negativos primero (más negativo a menos negativo), luego positivos
    all_points = negative_points + positive_points

    logger.debug("Puntos generados para rango [%d, %d]: %s", pwm_min, pwm_max, all_points)

    return all_points


class CalibrationPoint:
    """Representa un punto de calibración."""

    __slots__ = ('pwm', 'max_left', 'max_right', 'bias')

    def __init__(self, pwm: int, max_left: float = 1.0,
                 max_right: float = 1.0, bias: float = 0.0):
        """Inicializa un punto de calibración.

        Args:
            pwm: Valor PWM para este punto de calibración
            max_left: Factor de velocidad máxima del motor izquierdo
            max_right: Factor de velocidad máxima del motor derecho
            bias: Corrección de sesgo para movimiento recto
        """
        self.pwm = pwm
        self.max_left = max_left
        self.max_right = max_right
        self.bias = bias

    def to_dict(self) -> dict:
        """Convierte a diccionario para JSON."""
        return {
            'pwm': self.pwm,
            'max_left': round(self.max_left, 5),  # 5 decimales para preservar precisión 0.001
            'max_right': round(self.max_right, 5),
            'bias': round(self.bias, 5)
        }

    @staticmethod
    def from_dict(data: dict) -> 'CalibrationPoint':
        """Crea desde diccionario."""
        return CalibrationPoint(
            pwm=data['pwm'],
            max_left=data.get('max_left', 1.0),
            max_right=data.get('max_right', 1.0),
            bias=data.get('bias', 0.0)
        )


class RobotCalibrationMultipoint:
    """Gestiona calibración multi-punto de motores por robot."""

    def __init__(self, calibration_file: str = None):
        """Inicializa el sistema de calibración.

        Args:
            calibration_file: Ruta al archivo JSON de calibración.
        """
        if calibration_file is None:
            base_path = Path(__file__).parent.parent / "config"
            calibration_file = base_path / "robot_calibration_multipoint.json"

        self.calibration_file = Path(calibration_file)
        self.calibrations: Dict[str, dict] = {}
        self.load()

    def load(self):
        """Carga las calibraciones desde el archivo JSON."""
        try:
            if not self.calibration_file.exists():
                logger.warning("Archivo de calibración no encontrado: %s",
                             self.calibration_file)
                logger.info("Creando archivo con valores neutros por defecto")
                self._create_default_file()
                return

            with open(self.calibration_file, 'r', encoding='utf-8') as f:
                data = json.load(f)

            # Cargar calibraciones
            for robot_id, cal_data in data.items():
                if robot_id.startswith('_'):
                    continue

                # Parsear puntos de calibración
                points = []
                if 'calibration_points' in cal_data:
                    for point_data in cal_data['calibration_points']:
                        points.append(CalibrationPoint.from_dict(point_data))
                else:
                    # Formato legacy - crear punto único en 80 PWM
                    points = [CalibrationPoint(
                        pwm=80,
                        max_left=cal_data.get('max_speed_left', 1.0),
                        max_right=cal_data.get('max_speed_right', 1.0),
                        bias=cal_data.get('bias_correction', 0.0)
                    )]

                # Obtener rango PWM personalizado del robot (si existe)
                pwm_min = cal_data.get('pwm_min', DEFAULT_PWM_MIN)
                pwm_max = cal_data.get('pwm_max', DEFAULT_PWM_MAX)

                # Generar puntos esperados basados en el rango del robot
                expected_points = generate_calibration_points(pwm_min, pwm_max, num_points=5)

                # MIGRACIÓN AUTOMÁTICA: Si los puntos del JSON no coinciden con los esperados,
                # crear estructura completa con valores neutros donde falten
                points_dict = {p.pwm: p for p in points}  # Diccionario por PWM
                migrated_points = []

                for expected_pwm in expected_points:
                    if expected_pwm in points_dict:
                        # Punto existe en JSON, usar valores guardados
                        migrated_points.append(points_dict[expected_pwm])
                    else:
                        # Punto faltante, crear con valores neutros
                        migrated_points.append(CalibrationPoint(
                            pwm=expected_pwm,
                            max_left=1.0,
                            max_right=1.0,
                            bias=0.0
                        ))
                        logger.info("Punto %d PWM faltante para robot %s - usando valores neutros",
                                   expected_pwm, robot_id)

                self.calibrations[robot_id] = {
                    'deadzone_left': cal_data.get('deadzone_left', 0),
                    'deadzone_right': cal_data.get('deadzone_right', 0),
                    'pwm_min': pwm_min,
                    'pwm_max': pwm_max,
                    'points': migrated_points  # Ya están en orden correcto
                }

                logger.info("Robot %s: Rango PWM [%d, %d], %d puntos de calibración",
                           robot_id, pwm_min, pwm_max, len(migrated_points))

            logger.info("Calibraciones multi-punto cargadas para %d robots",
                       len(self.calibrations))

        except json.JSONDecodeError as e:
            logger.error("Error parseando JSON de calibración: %s", e)
        except Exception as e:
            logger.error("Error cargando calibraciones: %s", e)

    def _create_default_file(self):
        """Crea archivo de calibración con valores por defecto."""
        default_data = {
            "_comment": "Calibración bidireccional de motores - Puntos personalizados por robot",
            "_instructions": (
                "1. Usa 'find_pwm_range.py' para determinar pwm_min/pwm_max de cada robot. "
                "2. Los puntos se generan automáticamente en ese rango. "
                "3. Calibra cada punto para movimiento recto."
            ),
            "_bidirectional": "Cada rueda usa su calibración según dirección de giro (adelante/atrás)",
            "_format": {
                "pwm_min": "PWM mínimo útil (cámara detecta al robot)",
                "pwm_max": "PWM máximo útil (cámara detecta al robot)",
                "deadzone_left": "PWM mínimo para mover motor izquierdo (0-40)",
                "deadzone_right": "PWM mínimo para mover motor derecho (0-40)",
                "calibration_points": [
                    {"pwm": "Velocidad PWM (negativa=atrás)", "max_left": "Factor motor izq (0.5-2.0)",
                     "max_right": "Factor motor der (0.5-2.0)", "bias": "Corrección deriva (-0.5 a 0.5)"}
                ]
            }
        }

        # Crear valores neutros para 4 robots con puntos generados dinámicamente
        for i in range(4):
            # Usar rango por defecto
            points = generate_calibration_points(DEFAULT_PWM_MIN, DEFAULT_PWM_MAX, num_points=5)

            default_data[str(i)] = {
                "pwm_min": DEFAULT_PWM_MIN,
                "pwm_max": DEFAULT_PWM_MAX,
                "deadzone_left": 0,
                "deadzone_right": 0,
                "calibration_points": [
                    {"pwm": pwm, "max_left": 1.0, "max_right": 1.0, "bias": 0.0}
                    for pwm in points
                ]
            }

        with open(self.calibration_file, 'w', encoding='utf-8') as f:
            json.dump(default_data, f, indent=2, ensure_ascii=False)

        logger.info("Archivo de calibración creado: %s", self.calibration_file)

    def save(self):
        """Guarda las calibraciones actuales al archivo JSON."""
        try:
            # Leer archivo existente para preservar comentarios
            existing_data = {}
            if self.calibration_file.exists():
                with open(self.calibration_file, 'r', encoding='utf-8') as f:
                    existing_data = json.load(f)

            # Actualizar calibraciones
            for robot_id, cal in self.calibrations.items():
                existing_data[robot_id] = {
                    'pwm_min': cal.get('pwm_min', DEFAULT_PWM_MIN),
                    'pwm_max': cal.get('pwm_max', DEFAULT_PWM_MAX),
                    'deadzone_left': cal['deadzone_left'],
                    'deadzone_right': cal['deadzone_right'],
                    'calibration_points': [p.to_dict() for p in cal['points']]
                }

            # Guardar con formato bonito
            with open(self.calibration_file, 'w', encoding='utf-8') as f:
                json.dump(existing_data, f, indent=2, ensure_ascii=False)

            logger.info("Calibraciones guardadas en %s", self.calibration_file)

        except Exception as e:
            logger.error("Error guardando calibraciones: %s", e)

    def get_deadzone(self, robot_id: int) -> Tuple[int, int]:
        """Obtiene dead-zone para motores izquierdo y derecho.

        Args:
            robot_id: ID del robot

        Returns:
            Tupla (deadzone_left, deadzone_right) en PWM
        """
        robot_key = str(robot_id)

        if robot_key not in self.calibrations:
            return (0, 0)

        cal = self.calibrations[robot_key]
        return (cal['deadzone_left'], cal['deadzone_right'])

    def set_deadzone(self, robot_id: int, deadzone_left: int, deadzone_right: int):
        """Establece dead-zone para un robot.

        Args:
            robot_id: ID del robot
            deadzone_left: PWM mínimo motor izquierdo (0-40)
            deadzone_right: PWM mínimo motor derecho (0-40)
        """
        robot_key = str(robot_id)

        # Validar rangos
        deadzone_left = max(0, min(40, deadzone_left))
        deadzone_right = max(0, min(40, deadzone_right))

        if robot_key not in self.calibrations:
            self._init_robot_calibration(robot_id)

        self.calibrations[robot_key]['deadzone_left'] = deadzone_left
        self.calibrations[robot_key]['deadzone_right'] = deadzone_right

        logger.info("Dead-zone actualizado para robot %d: L=%d, R=%d",
                   robot_id, deadzone_left, deadzone_right)

    def get_calibration_point(self, robot_id: int, pwm_index: int) -> Optional[CalibrationPoint]:
        """Obtiene un punto de calibración específico.

        Args:
            robot_id: ID del robot
            pwm_index: Índice del punto (0-9 para 10 puntos bidireccionales)

        Returns:
            CalibrationPoint o None si no existe
        """
        robot_key = str(robot_id)

        if robot_key not in self.calibrations:
            return None

        points = self.calibrations[robot_key]['points']
        if pwm_index < 0 or pwm_index >= len(points):
            return None

        return points[pwm_index]

    def set_calibration_point(self, robot_id: int, pwm_index: int,
                             max_left: float, max_right: float, bias: float):
        """Establece valores de calibración para un punto específico.

        Args:
            robot_id: ID del robot
            pwm_index: Índice del punto (0-9 para 10 puntos bidireccionales)
            max_left: Factor motor izquierdo
            max_right: Factor motor derecho
            bias: Corrección de sesgo
        """
        robot_key = str(robot_id)

        if robot_key not in self.calibrations:
            self._init_robot_calibration(robot_id)

        points = self.calibrations[robot_key]['points']

        if pwm_index < 0 or pwm_index >= len(points):
            logger.error("Índice de punto inválido: %d", pwm_index)
            return

        # Validar rangos
        max_left = max(0.5, min(2.0, max_left))
        max_right = max(0.5, min(2.0, max_right))
        bias = max(-0.5, min(0.5, bias))

        point = points[pwm_index]
        point.max_left = max_left
        point.max_right = max_right
        point.bias = bias

        logger.info("Punto %d actualizado para robot %d @ %d PWM: L=%.3f, R=%.3f, B=%.3f",
                   pwm_index, robot_id, point.pwm, max_left, max_right, bias)

    def _init_robot_calibration(self, robot_id: int):
        """Inicializa calibración para un robot con valores neutros."""
        robot_key = str(robot_id)

        self.calibrations[robot_key] = {
            'deadzone_left': 0,
            'deadzone_right': 0,
            'points': [
                CalibrationPoint(pwm, 1.0, 1.0, 0.0)
                for pwm in DEFAULT_CALIBRATION_POINTS
            ]
        }

    def get_calibration_at_speed(self, robot_id: int, speed: int
                                ) -> Tuple[float, float, float]:
        """Obtiene calibración interpolada para velocidad específica.

        Usa interpolación lineal entre los 2 puntos más cercanos.
        Soporta calibración bidireccional (adelante y atrás por separado).
        Optimizado para velocidad (búsqueda lineal en lista pequeña).

        Args:
            robot_id: ID del robot
            speed: Velocidad PWM deseada (-127 a 127)

        Returns:
            Tupla (max_left, max_right, bias) interpolados
        """
        robot_key = str(robot_id)

        if robot_key not in self.calibrations:
            return (1.0, 1.0, 0.0)

        points = self.calibrations[robot_key]['points']

        # Caso especial: solo un punto
        if len(points) == 1:
            p = points[0]
            return (p.max_left, p.max_right, p.bias)

        # Búsqueda en el rango correcto según signo
        # Los puntos están ordenados: [...negativos..., ...positivos...]

        # Si speed <= primer punto (más negativo), usar primer punto
        if speed <= points[0].pwm:
            p = points[0]
            return (p.max_left, p.max_right, p.bias)

        # Si speed >= último punto (más positivo), usar último punto
        if speed >= points[-1].pwm:
            p = points[-1]
            return (p.max_left, p.max_right, p.bias)

        # Encontrar intervalo [lower, upper] que contiene speed
        # Búsqueda lineal - rápida para 10 elementos
        lower_idx = 0
        upper_idx = len(points) - 1

        for i in range(len(points) - 1):
            if points[i].pwm <= speed <= points[i + 1].pwm:
                lower_idx = i
                upper_idx = i + 1
                break

        # Interpolación lineal
        p_lower = points[lower_idx]
        p_upper = points[upper_idx]

        # Factor de interpolación (0.0 a 1.0)
        t = (speed - p_lower.pwm) / (p_upper.pwm - p_lower.pwm)

        # Interpolar linealmente cada parámetro
        max_left = p_lower.max_left + t * (p_upper.max_left - p_lower.max_left)
        max_right = p_lower.max_right + t * (p_upper.max_right - p_lower.max_right)
        bias = p_lower.bias + t * (p_upper.bias - p_lower.bias)

        return (max_left, max_right, bias)

    def apply_calibration(self, robot_id: int, left_speed: int, right_speed: int
                         ) -> Tuple[int, int]:
        """Aplica calibración multi-punto a las velocidades de motor.

        CALIBRACIÓN BIDIRECCIONAL POR RUEDA:
        Cada rueda usa su propia calibración según su dirección de giro.
        Ejemplo: Al girar, rueda derecha (+50) usa calibración adelante,
                 rueda izquierda (-50) usa calibración atrás.

        Args:
            robot_id: ID del robot
            left_speed: Velocidad motor izquierdo (-127 a 127)
            right_speed: Velocidad motor derecho (-127 a 127)

        Returns:
            Tupla (left_calibrated, right_calibrated) con valores calibrados
        """
        # Obtener calibración para cada rueda según su velocidad (con signo)
        max_left_l, _, bias_l = self.get_calibration_at_speed(robot_id, left_speed)
        _, max_right_r, bias_r = self.get_calibration_at_speed(robot_id, right_speed)

        # Aplicar factor de velocidad a cada rueda
        # Rueda izquierda usa max_left, rueda derecha usa max_right
        left_cal = left_speed * max_left_l
        right_cal = right_speed * max_right_r

        # Aplicar corrección de sesgo solo en movimiento recto (misma dirección)
        if abs(left_speed - right_speed) < 20 and left_speed * right_speed > 0:
            # Usar promedio de bias de ambas calibraciones
            bias_avg = (bias_l + bias_r) / 2
            bias_value = bias_avg * 127
            left_cal += bias_value
            right_cal -= bias_value

        # Limitar a rango válido
        left_cal = max(-127, min(127, int(left_cal)))
        right_cal = max(-127, min(127, int(right_cal)))

        return (left_cal, right_cal)

    def reset_robot(self, robot_id: int):
        """Resetea la calibración de un robot a valores neutros."""
        self._init_robot_calibration(robot_id)

    def get_pwm_range(self, robot_id: int) -> Tuple[int, int]:
        """Obtiene el rango PWM útil de un robot.

        Args:
            robot_id: ID del robot (0-3)

        Returns:
            Tupla (pwm_min, pwm_max)
        """
        robot_key = str(robot_id)
        if robot_key not in self.calibrations:
            logger.warning("Robot %d no tiene calibración, usando rango por defecto", robot_id)
            return (DEFAULT_PWM_MIN, DEFAULT_PWM_MAX)

        return (
            self.calibrations[robot_key].get('pwm_min', DEFAULT_PWM_MIN),
            self.calibrations[robot_key].get('pwm_max', DEFAULT_PWM_MAX)
        )

    def set_pwm_range(self, robot_id: int, pwm_min: int, pwm_max: int):
        """Establece el rango PWM útil de un robot y regenera puntos de calibración.

        IMPORTANTE: Esto regenera todos los puntos con valores neutros.
        Debes recalibrar el robot después de cambiar el rango.

        Args:
            robot_id: ID del robot (0-3)
            pwm_min: PWM mínimo útil (positivo)
            pwm_max: PWM máximo útil (positivo)
        """
        robot_key = str(robot_id)

        # Validar rango
        if pwm_min >= pwm_max:
            logger.error("pwm_min (%d) debe ser menor que pwm_max (%d)", pwm_min, pwm_max)
            return

        if pwm_min < 5 or pwm_max > 127:
            logger.error("Rango PWM debe estar entre 5 y 127")
            return

        # Generar nuevos puntos
        new_points_pwm = generate_calibration_points(pwm_min, pwm_max, num_points=5)
        new_points = [
            CalibrationPoint(pwm=pwm, max_left=1.0, max_right=1.0, bias=0.0)
            for pwm in new_points_pwm
        ]

        # Inicializar calibración si no existe
        if robot_key not in self.calibrations:
            self._init_robot_calibration(robot_id)

        # Actualizar rango y puntos
        self.calibrations[robot_key]['pwm_min'] = pwm_min
        self.calibrations[robot_key]['pwm_max'] = pwm_max
        self.calibrations[robot_key]['points'] = new_points

        logger.info("Robot %d: Rango PWM actualizado a [%d, %d], %d puntos regenerados",
                   robot_id, pwm_min, pwm_max, len(new_points))
        logger.warning("Robot %d: Puntos regenerados con valores neutros - RECALIBRAR", robot_id)

    def get_calibration_points_pwm(self, robot_id: int = None) -> List[int]:
        """Obtiene la lista de valores PWM de los puntos de calibración.

        Args:
            robot_id: ID del robot (0-3). Si es None, retorna puntos por defecto.

        Returns:
            Lista de valores PWM de los puntos de calibración
        """
        if robot_id is None:
            return DEFAULT_CALIBRATION_POINTS.copy()

        robot_key = str(robot_id)
        if robot_key not in self.calibrations:
            logger.warning("Robot %d no tiene calibración, usando puntos por defecto", robot_id)
            return DEFAULT_CALIBRATION_POINTS.copy()

        # Retornar puntos específicos del robot
        return [p.pwm for p in self.calibrations[robot_key]['points']]


# Instancia global singleton
_CALIBRATION_MANAGER_MULTIPOINT = None


def get_calibration_manager_multipoint() -> RobotCalibrationMultipoint:
    """Obtiene la instancia global del gestor de calibración multi-punto."""
    global _CALIBRATION_MANAGER_MULTIPOINT  # pylint: disable=global-statement
    if _CALIBRATION_MANAGER_MULTIPOINT is None:
        _CALIBRATION_MANAGER_MULTIPOINT = RobotCalibrationMultipoint()
    return _CALIBRATION_MANAGER_MULTIPOINT
