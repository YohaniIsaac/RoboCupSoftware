import logging
import numpy as np
import skfuzzy as fuzz
from skfuzzy import control as ctrl
from robot_soccer.ai.role_assignment.role_assigner import RoleAssigner
from robot_soccer.config import (LADO_DERECHO, LADO_IZQUIERDO, EQUIPO_ROJO,
                                ZONA_IZQUIERDA, ZONA_DERECHA, FIELD_SIM)
from robot_soccer.ai.fuzzy_logic.proximity_calculator import calcular_ventaja_proximidad

log = logging.getLogger(__name__)


class FuzzyRobotTeamManager:
    """Clase que implementa una máquina de estados difusa para gestionar los estados de cada equipo de robots.

    Atributos:
        team                (str): Equipo del robot ('red' o 'blue')
        robot_aliado_one    (int): Identificador del primer robot aliado
        robot_aliado_two    (int): Identificador del segundo robot aliado
        robot_rival_one     (int): Identificador del primer robot rival
        robot_rival_two     (int): Identificador del segundo robot rival
        ancho_campo         (int): Ancho del campo en píxeles
        lim_izquierdo       (float): Límite izquierdo del campo para determinar zonas aliada, neutral o rival
        lim_derecho         (float): Límite derecho del campo para determinar zonas aliada, neutral o rival.
    """

    def __init__(self, players, ball, team="red", zonas=(ZONA_IZQUIERDA, ZONA_DERECHA),
                 field=None):
        """Inicializa el gestor de equipos de forma dinámica.

        Args:
            players: Lista de jugadores
            ball: Objeto pelota
            team: Equipo ('red' o 'blue')
            zonas: Tupla con límites de zona
            field: FieldGeometry con geometría del campo. Defaults to FIELD_SIM.
        """
        self.team = team
        self.field = field if field is not None else FIELD_SIM
        self.side = LADO_IZQUIERDO if team == EQUIPO_ROJO else LADO_DERECHO
        self.ball = ball

        # Filtrar jugadores por equipo
        self.team_players = [p for p in players if p.team == team]
        self.opponents = [p for p in players if p.team != team]

        # Validar que tenemos suficientes jugadores
        if len(self.team_players) < 1:
            raise ValueError(f"No hay suficientes jugadores en el equipo {team}")

        # Zonas del campo
        self.lim_izquierdo, self.lim_derecho = [self.field.width * z for z in zonas]

        # Configuración del equipo
        # self.side = LADO_IZQUIERDO if team == EQUIPO_ROJO else LADO_DERECHO

        # self.team = team
        # # Pasando objetos Player directamente
        # if team == EQUIPO_ROJO:
        #     self.aliado_1 = player_1  # Objeto Player
        #     self.aliado_2 = player_2  # Objeto Player
        #     self.rival_1 = player_3  # Objeto Player
        #     self.rival_2 = player_4  # Objeto Player
        # else:
        #     self.aliado_1 = player_3  # Objeto Player
        #     self.aliado_2 = player_4  # Objeto Player
        #     self.rival_1 = player_1  # Objeto Player
        #     self.rival_2 = player_2  # Objeto Player
        #
        # self.ball = ball

        # Configuración de la cancha
        # self.lim_izquierdo, self.lim_derecho = [ANCHO_CAMPO * z for z in zonas]

        # Inicia el objeto para definir roles
        self.role_assigner = RoleAssigner(self.team_players, self.ball)

        # Inicializar atributos antes de configurar los sistemas
        self.reglas_posesion = None
        self.reglas_proximidad = None
        self.reglas_zona = None

        # Inicializar sistemas difusos
        self._init_posesion_system()
        self._init_proximidad_system()
        self._init_zona_system()

    def _init_posesion_system(self):
        """Inicializa el sistema de lógica difusa para determinar la posesión de la pelota."""
        # ENTRADA - Distancia (escalada al campo)
        dist_max = int(self.field.width * 1.17) + 1
        self.distancia_aliado1 = ctrl.Antecedent(
            np.arange(0, dist_max, 1), "distancia_aliado1"
        )
        self.distancia_aliado2 = ctrl.Antecedent(
            np.arange(0, dist_max, 1), "distancia_aliado2"
        )
        self.distancia_rival1 = ctrl.Antecedent(
            np.arange(0, dist_max, 1), "distancia_rival1"
        )
        self.distancia_rival2 = ctrl.Antecedent(
            np.arange(0, dist_max, 1), "distancia_rival2"
        )

        # ENTRADA - Orientación
        self.orientacion_aliado1 = ctrl.Antecedent(
            np.arange(0, 3.15, 0.01), "orientacion_aliado1"
        )
        self.orientacion_aliado2 = ctrl.Antecedent(
            np.arange(0, 3.15, 0.01), "orientacion_aliado2"
        )
        self.orientacion_rival1 = ctrl.Antecedent(
            np.arange(0, 3.15, 0.01), "orientacion_rival1"
        )
        self.orientacion_rival2 = ctrl.Antecedent(
            np.arange(0, 3.15, 0.01), "orientacion_rival2"
        )

        # ENTRADA - Pelota
        self.velocidad_pelota = ctrl.Antecedent(
            np.arange(0, 21, 1), "velocidad_pelota"
        )  # 0-20 px/frame
        self.direccion_movimiento = ctrl.Antecedent(
            np.arange(0, 3, 0.1), "direccion_movimiento"
        )  # 0-2

        # SALIDA - posesión de la pelota
        self.posesion_pelota = ctrl.Consequent(
            np.arange(0, 1.1, 0.1), "posesion_pelota"
        )

        # Definir funciones de membresía y reglas
        self._definir_funciones_membresia_posesion()
        self._definir_reglas_posesion()

        # Sistema difuso para posesión de la pelota
        self.control_posesion = ctrl.ControlSystem(self.reglas_posesion)
        self.sim_posesion = ctrl.ControlSystemSimulation(self.control_posesion)

    def _definir_funciones_membresia_posesion(self):
        """Define las funciones de membresía para las variables de entrada y salida del sistema de posesión."""
        f = self.field
        # ENTRADA - Distancias (escaladas al campo)
        # Breakpoints como proporción del ancho: cerca ~0-13%, media ~10-37%, lejos ~33-100%
        for dist_var in [self.distancia_aliado1, self.distancia_aliado2,
                         self.distancia_rival1, self.distancia_rival2]:
            dist_var["cerca"] = fuzz.trapmf(
                dist_var.universe,
                [0, 0, f.ratio_to_px(0.067), f.ratio_to_px(0.133)]
            )
            dist_var["media"] = fuzz.trapmf(
                dist_var.universe,
                [f.ratio_to_px(0.1), f.ratio_to_px(0.167), f.ratio_to_px(0.3), f.ratio_to_px(0.367)]
            )
            dist_var["lejos"] = fuzz.trapmf(
                dist_var.universe,
                [f.ratio_to_px(0.333), f.ratio_to_px(0.533), f.width, f.width]
            )

        # ENTRADA - Orientaciones (en radianes)
        self.orientacion_aliado1["apuntando"] = fuzz.trimf(
            self.orientacion_aliado1.universe, [0, 0, 0.52]  # 0-30 grados
        )
        self.orientacion_aliado1["medio_apuntando"] = fuzz.trimf(
            self.orientacion_aliado1.universe, [0.44, 0.79, 1.57]  # 25-90 grados
        )
        self.orientacion_aliado1["no_apuntando"] = fuzz.trimf(
            self.orientacion_aliado1.universe, [1.31, 2.36, 3.14]  # 75-180 grados
        )

        self.orientacion_aliado2["apuntando"] = fuzz.trimf(
            self.orientacion_aliado2.universe, [0, 0, 0.52]  # 0-30 grados
        )
        self.orientacion_aliado2["medio_apuntando"] = fuzz.trimf(
            self.orientacion_aliado2.universe, [0.44, 0.79, 1.57]  # 25-90 grados
        )
        self.orientacion_aliado2["no_apuntando"] = fuzz.trimf(
            self.orientacion_aliado2.universe, [1.31, 2.36, 3.14]  # 75-180 grados
        )

        self.orientacion_rival1["apuntando"] = fuzz.trimf(
            self.orientacion_rival1.universe, [0, 0, 0.52]  # 0-30 grados
        )
        self.orientacion_rival1["medio_apuntando"] = fuzz.trimf(
            self.orientacion_rival1.universe, [0.44, 0.79, 1.57]  # 25-90 grados
        )
        self.orientacion_rival1["no_apuntando"] = fuzz.trimf(
            self.orientacion_rival1.universe, [1.31, 2.36, 3.14]  # 75-180 grados
        )

        self.orientacion_rival2["apuntando"] = fuzz.trimf(
            self.orientacion_rival2.universe, [0, 0, 0.52]  # 0-30 grados
        )
        self.orientacion_rival2["medio_apuntando"] = fuzz.trimf(
            self.orientacion_rival2.universe, [0.44, 0.79, 1.57]  # 25-90 grados
        )
        self.orientacion_rival2["no_apuntando"] = fuzz.trimf(
            self.orientacion_rival2.universe, [1.31, 2.36, 3.14]  # 75-180 grados
        )

        # ENTRADA - Velocidad de la pelota
        self.velocidad_pelota["quieta"] = fuzz.trimf(
            self.velocidad_pelota.universe, [0, 0, 2]
        )
        self.velocidad_pelota["lenta"] = fuzz.trimf(
            self.velocidad_pelota.universe, [1, 5, 9]
        )
        self.velocidad_pelota["rapida"] = fuzz.trimf(
            self.velocidad_pelota.universe, [7, 15, 20]
        )

        # ENTRADA - Dirección de moviminento de la pelota
        self.direccion_movimiento["hacia_zona_aliada"] = fuzz.trimf(
            self.direccion_movimiento.universe, [0, 0, 0.8]
        )
        self.direccion_movimiento["neutral"] = fuzz.trimf(
            self.direccion_movimiento.universe, [0.6, 1, 1.4]
        )
        self.direccion_movimiento["hacia_zona_rival"] = fuzz.trimf(
            self.direccion_movimiento.universe, [1.2, 2, 2]
        )

        # SALIDAS
        self.posesion_pelota["posesion_aliada"] = fuzz.trimf(
            self.posesion_pelota.universe, [0, 0, 0.35]
        )
        self.posesion_pelota["libre"] = fuzz.trimf(
            self.posesion_pelota.universe, [0.25, 0.5, 0.75]
        )
        self.posesion_pelota["posesion_rival"] = fuzz.trimf(
            self.posesion_pelota.universe, [0.65, 1, 1]
        )

    def _definir_reglas_posesion(self):
        """Define las reglas para determinar el estado de la pelota."""
        # Reglas para cuando un aliado está cerca
        regla1 = ctrl.Rule(
            self.distancia_aliado1["cerca"] & self.orientacion_aliado1["apuntando"],
            self.posesion_pelota["posesion_aliada"],
        )

        regla2 = ctrl.Rule(
            self.distancia_aliado2["cerca"] & self.orientacion_aliado2["apuntando"],
            self.posesion_pelota["posesion_aliada"],
        )

        # Reglas para cuando un rival está cerca
        regla3 = ctrl.Rule(
            self.distancia_rival1["cerca"] & self.orientacion_rival1["apuntando"],
            self.posesion_pelota["posesion_rival"],
        )

        regla4 = ctrl.Rule(
            self.distancia_rival2["cerca"] & self.orientacion_rival2["apuntando"],
            self.posesion_pelota["posesion_rival"],
        )

        # Reglas para pelota libre
        regla5 = ctrl.Rule(
            (self.distancia_aliado1["media"] | self.distancia_aliado1["lejos"])
            & (self.distancia_aliado2["media"] | self.distancia_aliado2["lejos"])
            & (self.distancia_rival1["media"] | self.distancia_rival1["lejos"])
            & (self.distancia_rival2["media"] | self.distancia_rival2["lejos"]),
            self.posesion_pelota["libre"],
        )

        regla6 = ctrl.Rule(
            self.distancia_aliado1["cerca"]
            & self.orientacion_aliado1["no_apuntando"]
            & (self.distancia_rival1["media"] | self.distancia_rival1["lejos"])
            & (self.distancia_rival2["media"] | self.distancia_rival2["lejos"]),
            self.posesion_pelota["libre"],
        )

        regla7 = ctrl.Rule(
            self.distancia_aliado2["cerca"]
            & self.orientacion_aliado2["no_apuntando"]
            & (self.distancia_rival1["media"] | self.distancia_rival1["lejos"])
            & (self.distancia_rival2["media"] | self.distancia_rival2["lejos"]),
            self.posesion_pelota["libre"],
        )

        regla8 = ctrl.Rule(
            self.distancia_rival1["cerca"]
            & self.orientacion_rival1["no_apuntando"]
            & (self.distancia_aliado1["media"] | self.distancia_aliado1["lejos"])
            & (self.distancia_aliado2["media"] | self.distancia_aliado2["lejos"]),
            self.posesion_pelota["libre"],
        )

        regla9 = ctrl.Rule(
            self.distancia_rival2["cerca"]
            & self.orientacion_rival2["no_apuntando"]
            & (self.distancia_aliado1["media"] | self.distancia_aliado1["lejos"])
            & (self.distancia_aliado2["media"] | self.distancia_aliado2["lejos"]),
            self.posesion_pelota["libre"],
        )

        # Regla por defecto (si ninguna otra regla se activa)
        regla10 = ctrl.Rule(
            ~self.distancia_aliado1["cerca"]
            & ~self.distancia_aliado2["cerca"]
            & ~self.distancia_rival1["cerca"]
            & ~self.distancia_rival2["cerca"],
            self.posesion_pelota["libre"],
        )

       # REGLAS CON VELOCIDAD
        regla11 = ctrl.Rule(
            self.velocidad_pelota["quieta"] & self.distancia_aliado1["cerca"],
            self.posesion_pelota["posesion_aliada"]
        )

        regla12 = ctrl.Rule(
            self.velocidad_pelota["rapida"] & self.distancia_aliado1["cerca"] & self.orientacion_aliado1["apuntando"],
            self.posesion_pelota["posesion_aliada"]
        )

        regla13 = ctrl.Rule(
            self.velocidad_pelota["rapida"] & self.distancia_rival1["cerca"],
            self.posesion_pelota["libre"]
        )

        # REGLAS CON DIRECCIÓN
        regla14 = ctrl.Rule(
            self.direccion_movimiento["hacia_zona_aliada"] & self.distancia_aliado1["media"],
            self.posesion_pelota["posesion_aliada"]
        )

        regla15 = ctrl.Rule(
            self.direccion_movimiento["hacia_zona_rival"] & self.distancia_rival1["media"],
            self.posesion_pelota["posesion_rival"]
        )

        regla16 = ctrl.Rule(
            self.direccion_movimiento["neutral"] & self.velocidad_pelota["lenta"],
            self.posesion_pelota["libre"]
        )

        # REGLAS COMBINADAS
        regla17 = ctrl.Rule(
            self.velocidad_pelota["rapida"] & self.direccion_movimiento["hacia_zona_rival"] & self.distancia_rival1["cerca"],
            self.posesion_pelota["posesion_rival"]
        )

        regla18 = ctrl.Rule(
            self.velocidad_pelota["quieta"] & self.direccion_movimiento["neutral"],
            self.posesion_pelota["libre"]
        )

        # PROXIMIDAD MÚLTIPLE
        regla19 = ctrl.Rule(
            self.distancia_aliado1["cerca"] & self.distancia_aliado2["cerca"] &
            self.distancia_rival1["media"] & self.distancia_rival2["lejos"],
            self.posesion_pelota["posesion_aliada"]
        )

        regla20 = ctrl.Rule(
            self.distancia_rival1["cerca"] & self.distancia_rival2["cerca"] &
            self.distancia_aliado1["media"] & self.distancia_aliado2["lejos"],
            self.posesion_pelota["posesion_rival"]
        )

        # REGLAS DE INTERCEPTACIÓN (pelota rápida)
        regla21 = ctrl.Rule(
            self.velocidad_pelota["rapida"] &
            self.distancia_aliado1["cerca"] &
            self.orientacion_aliado1["apuntando"] &
            self.direccion_movimiento["hacia_zona_aliada"],
            self.posesion_pelota["posesion_aliada"]
        )

        regla22 = ctrl.Rule(
            self.velocidad_pelota["rapida"] &
            self.distancia_rival1["cerca"] &
            self.orientacion_rival1["apuntando"] &
            self.direccion_movimiento["hacia_zona_rival"],
            self.posesion_pelota["posesion_rival"]
        )

        # REGLAS DE CONFLICTO (cuando ambos equipos están cerca)
        regla23 = ctrl.Rule(
            self.distancia_aliado1["cerca"] & self.distancia_rival1["cerca"] &
            self.orientacion_aliado1["apuntando"] & self.orientacion_rival1["no_apuntando"],
            self.posesion_pelota["posesion_aliada"]
        )

        regla24 = ctrl.Rule(
            self.distancia_aliado1["cerca"] & self.distancia_rival1["cerca"] &
            self.orientacion_aliado1["no_apuntando"] & self.orientacion_rival1["apuntando"],
            self.posesion_pelota["posesion_rival"]
        )

        # REGLAS DE PELOTA ESTÁTICA
        regla25 = ctrl.Rule(
            self.velocidad_pelota["quieta"] &
            self.distancia_aliado1["cerca"] &
            self.distancia_rival1["lejos"],
            self.posesion_pelota["posesion_aliada"]
        )

        regla26 = ctrl.Rule(
            self.velocidad_pelota["quieta"] &
            self.distancia_rival1["cerca"] &
            self.distancia_aliado1["lejos"],
            self.posesion_pelota["posesion_rival"]
        )
        self.reglas_posesion = [
            regla1, regla2, regla3, regla4, regla5, regla6, regla7, regla8, regla9, regla10,
            regla11, regla12, regla13, regla14, regla15, regla16, regla17, regla18, regla19,
            regla20, #regla21, regla22, regla23, regla24, regla25
        ]

    def _init_proximidad_system(self):
        """Inicializa el sistema difuso para determinar la proximidad de la pelota."""
        # ENTRADA - posesión de la pelota (salida de sist. difuso)
        self.posesion_pelota_result = ctrl.Antecedent(
            np.arange(0, 1.1, 0.1), "posesion_pelota_result"
        )

        # ENTRADA - Ventaja de proximidad (escalada al campo)
        ventaja_max = int(self.field.width * 0.667)  # ~1000 para 1500, ~427 para 640
        self.ventaja_proximidad = ctrl.Antecedent(
            np.arange(-ventaja_max, ventaja_max + 1, 1), "ventaja_proximidad"
        )

        # ENTRADA - Velocidad de la pelota
        self.velocidad_pelota_prox = ctrl.Antecedent(
            np.arange(0, 21, 1), "velocidad_pelota_prox"
        )

        # SALIDA - proximidad del equipo a la pelota
        self.proximidad_equipo = ctrl.Consequent(
            np.arange(0, 2.1, 0.1), "proximidad_equipo"
        )

        # Definir funciones de membresía y reglas
        self._definir_funciones_membresia_proximidad()
        self._definir_reglas_proximidad()

        # Sistema difuso para proximidad del equipo
        self.control_proximidad = ctrl.ControlSystem(self.reglas_proximidad)
        self.sim_proximidad = ctrl.ControlSystemSimulation(self.control_proximidad)

    def _definir_funciones_membresia_proximidad(self):
        """Define las funciones de membresía para las variables de entrada y salida del sistema de proximidad."""
        # ENTRADA - posesión de la pelota
        self.posesion_pelota_result["posesion_aliada"] = fuzz.trimf(
            self.posesion_pelota.universe, [0, 0, 0.35]
        )
        self.posesion_pelota_result["libre"] = fuzz.trimf(
            self.posesion_pelota.universe, [0.2, 0.5, 0.8]
        )
        self.posesion_pelota_result["posesion_rival"] = fuzz.trimf(
            self.posesion_pelota.universe, [0.65, 1, 1]
        )

        # ENTRADA - Ventaja de proximidad (POSITIVO = ventaja aliada, NEGATIVO = ventaja rival)
        f = self.field
        self.ventaja_proximidad["ventaja_aliada_grande"] = fuzz.trimf(
            self.ventaja_proximidad.universe,
            [f.ratio_to_px(0.2), f.ratio_to_px(0.667), f.ratio_to_px(0.667)]
        )
        self.ventaja_proximidad["ventaja_aliada_media"] = fuzz.trimf(
            self.ventaja_proximidad.universe,
            [f.ratio_to_px(0.033), f.ratio_to_px(0.133), f.ratio_to_px(0.333)]
        )
        self.ventaja_proximidad["equilibrado"] = fuzz.trimf(
            self.ventaja_proximidad.universe,
            [-f.ratio_to_px(0.067), 0, f.ratio_to_px(0.067)]
        )
        self.ventaja_proximidad["ventaja_rival_media"] = fuzz.trimf(
            self.ventaja_proximidad.universe,
            [-f.ratio_to_px(0.333), -f.ratio_to_px(0.133), -f.ratio_to_px(0.033)]
        )
        self.ventaja_proximidad["ventaja_rival_grande"] = fuzz.trimf(
            self.ventaja_proximidad.universe,
            [-f.ratio_to_px(0.667), -f.ratio_to_px(0.667), -f.ratio_to_px(0.2)]
        )

        # ENTRADA - Velocidad de la pelota
        self.velocidad_pelota_prox["quieta"] = fuzz.trimf(
            self.velocidad_pelota_prox.universe, [0, 0, 2]
        )
        self.velocidad_pelota_prox["lenta"] = fuzz.trimf(
            self.velocidad_pelota_prox.universe, [1, 5, 9]
        )
        self.velocidad_pelota_prox["rapida"] = fuzz.trimf(
            self.velocidad_pelota_prox.universe, [7, 15, 20]
        )

        # SALIDAS
        self.proximidad_equipo["aliado"] = fuzz.trimf(
            self.proximidad_equipo.universe, [0, 0, 0.8]
        )
        self.proximidad_equipo["neutro"] = fuzz.trimf(
            self.proximidad_equipo.universe, [0.6, 1, 1.4]
        )
        self.proximidad_equipo["rival"] = fuzz.trimf(
            self.proximidad_equipo.universe, [1.2, 2, 2]
        )

    def _definir_reglas_proximidad(self):
        """Define las reglas para determinar la proximidad de la pelota con un conjunto más completo."""
        # Reglas básicas basadas en posesión
        regla1 = ctrl.Rule(
            self.posesion_pelota_result["posesion_aliada"],
            self.proximidad_equipo["aliado"],
        )

        regla2 = ctrl.Rule(
            self.posesion_pelota_result["posesion_rival"],
            self.proximidad_equipo["rival"],
        )

        # Reglas basadas en ventaja de proximidad cuando la pelota está libre
        regla3 = ctrl.Rule(
            self.posesion_pelota_result["libre"]
            & self.ventaja_proximidad["ventaja_aliada_grande"],
            self.proximidad_equipo["aliado"],
        )

        regla4 = ctrl.Rule(
            self.posesion_pelota_result["libre"]
            & self.ventaja_proximidad["ventaja_aliada_media"],
            self.proximidad_equipo["aliado"],
        )

        regla5 = ctrl.Rule(
            self.posesion_pelota_result["libre"]
            & self.ventaja_proximidad["equilibrado"],
            self.proximidad_equipo["neutro"],
        )

        regla6 = ctrl.Rule(
            self.posesion_pelota_result["libre"]
            & self.ventaja_proximidad["ventaja_rival_media"],
            self.proximidad_equipo["rival"],
        )

        regla7 = ctrl.Rule(
            self.posesion_pelota_result["libre"]
            & self.ventaja_proximidad["ventaja_rival_grande"],
            self.proximidad_equipo["rival"],
        )

        # Reglas con velocidad
        regla8 = ctrl.Rule(
            self.velocidad_pelota_prox["rapida"] & self.ventaja_proximidad["ventaja_aliada_media"],
            self.proximidad_equipo["aliado"]
        )

        regla9 = ctrl.Rule(
            self.velocidad_pelota_prox["quieta"] & self.ventaja_proximidad["equilibrado"],
            self.proximidad_equipo["neutro"]
        )

        # Casos de pelota muy rápida
        regla10 = ctrl.Rule(
            self.velocidad_pelota_prox["rapida"] &
            self.ventaja_proximidad["ventaja_rival_media"],
            self.proximidad_equipo["rival"]
        )

        regla11 = ctrl.Rule(
            self.velocidad_pelota_prox["rapida"] &
            self.ventaja_proximidad["ventaja_rival_grande"],
            self.proximidad_equipo["rival"]
        )

        # Casos de pelota lenta con diferentes ventajas
        regla12 = ctrl.Rule(
            self.velocidad_pelota_prox["lenta"] &
            self.ventaja_proximidad["ventaja_aliada_grande"],
            self.proximidad_equipo["aliado"]
        )

        regla13 = ctrl.Rule(
            self.velocidad_pelota_prox["lenta"] &
            self.ventaja_proximidad["ventaja_rival_grande"],
            self.proximidad_equipo["rival"]
        )

        # Casos especiales de equilibrio
        regla14 = ctrl.Rule(
            self.posesion_pelota_result["libre"] &
            self.velocidad_pelota_prox["lenta"] &
            self.ventaja_proximidad["equilibrado"],
            self.proximidad_equipo["neutro"]
        )

        self.reglas_proximidad = [
            regla1,regla2, regla3, regla4, regla5, regla6, regla7,
            regla8, regla9,
            regla10, regla11, regla12, regla13, regla14,
        ]

    def _init_zona_system(self):
        """Inicializa el sistema difuso para determinar la zona del campo donde se encuentra la pelota."""
        # ENTRADA - posición de la pelota (escalada al campo)
        self.posicion_x = ctrl.Antecedent(
            np.arange(0, self.field.width + 1, 1), "posicion_x"
        )

        # ENTRADA - dirección de movimiento
        self.direccion_movimiento_zona = ctrl.Antecedent(
            np.arange(0, 3, 0.1), "direccion_movimiento_zona"
        )

        # SALIDA - zona dónde se encuentra la pelota
        self.zona_pelota = ctrl.Consequent(np.arange(0, 2.1, 0.1), "zona_pelota")

        # Definir funciones de membresía y reglas
        self._definir_funciones_membresia_zona()
        self._definir_reglas_zona()

        # Sistema difuso para la zona del campo
        self.control_zona = ctrl.ControlSystem(self.reglas_zona)
        self.sim_zona = ctrl.ControlSystemSimulation(self.control_zona)

    def _definir_funciones_membresia_zona(self):
        """Define las funciones de membresía para las variables de entrada y salida del sistema de zona."""
        w = self.field.width
        overlap = self.field.ratio_to_px(0.053)  # ~80px en 1500, ~34px en 640
        # ENTRADA - posición de la pelota por equipo
        if self.side == "LEFT":
            self.posicion_x["defensiva"] = fuzz.trimf(
                self.posicion_x.universe, [0, 0, self.lim_izquierdo]
            )
            self.posicion_x["media"] = fuzz.trimf(
                self.posicion_x.universe,
                [self.lim_izquierdo - overlap, w // 2, self.lim_derecho + overlap],
            )
            self.posicion_x["ofensiva"] = fuzz.trimf(
                self.posicion_x.universe, [self.lim_derecho, w, w]
            )
        else:
            self.posicion_x["defensiva"] = fuzz.trimf(
                self.posicion_x.universe, [self.lim_derecho, w, w]
            )
            self.posicion_x["media"] = fuzz.trimf(
                self.posicion_x.universe,
                [self.lim_izquierdo - overlap, w // 2, self.lim_derecho + overlap],
            )
            self.posicion_x["ofensiva"] = fuzz.trimf(
                self.posicion_x.universe, [0, 0, self.lim_izquierdo]
            )

        # ENTRADA - Dirección para zona
        self.direccion_movimiento_zona["hacia_zona_aliada"] = fuzz.trimf(
            self.direccion_movimiento_zona.universe, [0, 0, 0.8]
        )
        self.direccion_movimiento_zona["neutral"] = fuzz.trimf(
            self.direccion_movimiento_zona.universe, [0.6, 1, 1.4]
        )
        self.direccion_movimiento_zona["hacia_zona_rival"] = fuzz.trimf(
            self.direccion_movimiento_zona.universe, [1.2, 2, 2]
        )

        # SALIDAS
        self.zona_pelota["defensiva"] = fuzz.trimf(
            self.zona_pelota.universe, [0, 0, 0.5]
        )
        self.zona_pelota["media"] = fuzz.trimf(
            self.zona_pelota.universe, [0.4, 1, 1.6]
        )
        self.zona_pelota["ofensiva"] = fuzz.trimf(
            self.zona_pelota.universe, [1.5, 2, 2]
        )

    def _definir_reglas_zona(self):
        """Define las reglas para determinar la zona del campo donde se encuentra la pelota."""
        regla1 = ctrl.Rule(
            self.posicion_x["defensiva"], self.zona_pelota["defensiva"]
        )
        regla2 = ctrl.Rule(
            self.posicion_x["media"], self.zona_pelota["media"]
        )
        regla3 = ctrl.Rule(
            self.posicion_x["ofensiva"], self.zona_pelota["ofensiva"]
        )

        # Reglas con dirección de la pelota
        regla4 = ctrl.Rule(
            self.posicion_x["media"] & self.direccion_movimiento_zona["hacia_zona_aliada"],
            self.zona_pelota["defensiva"]
        )

        regla5 = ctrl.Rule(
            self.posicion_x["media"] & self.direccion_movimiento_zona["hacia_zona_rival"],
            self.zona_pelota["ofensiva"]
        )


        self.reglas_zona = [regla1, regla2, regla3, regla4, regla5,]

    def _calcular_velocidad_y_direccion(self):
        """Calcula la velocidad y dirección de movimiento de la pelota."""
        velocity_data = self.ball.get_velocity()  # [v_x, v_y, speed]
        v_x, v_y, speed = velocity_data

        # Velocidad (magnitud)
        velocidad = speed

        # Dirección basada en componentes de velocidad
        if abs(v_x) < 1 and abs(v_y) < 1:  # Pelota quieta
            direccion = 1.0  # neutral
        else:
            # Determinar dirección según lado del equipo y velocidad X
            if self.side == "LEFT":
                if v_x < -2:  # Se mueve hacia izquierda (zona aliada)
                    direccion = 0.0  # hacia_zona_aliada
                elif v_x > 2:  # Se mueve hacia derecha (zona rival)
                    direccion = 2.0  # hacia_zona_rival
                else:
                    direccion = 1.0  # neutral
            else:  # RIGHT
                if v_x > 2:  # Se mueve hacia derecha (zona aliada)
                    direccion = 0.0  # hacia_zona_aliada
                elif v_x < -2:  # Se mueve hacia izquierda (zona rival)
                    direccion = 2.0  # hacia_zona_rival
                else:
                    direccion = 1.0  # neutral

        return velocidad, direccion

    def evaluar_ms_logic_difusse(self):
        """Evalúa el estado del sistema basado en la posición de la pelota y los robots.

        Returns:
            dict: Un diccionario con los resultados de la evaluación:
                - 'estado_pelota': Estado de la pelota (posesión aliada, libre o posesión rival) (0; 0.5; 1).
                - 'equipo_cercano': Proximidad de la pelota (aliado, neutro o rival) (0; 1; 2).
                - 'zona_pelota': Zona del campo donde se encuentra la pelota (defensiva, media u ofensiva) (0; 1; 2).
        """

        # Obtener velocidad y dirección de la pelota
        velocidad, direccion = self._calcular_velocidad_y_direccion()

        # Calcular distancias y orientaciones
        distancias_aliados = []
        distancias_rivales = []
        orientaciones_aliados = []
        orientaciones_rivales = []

        max_dist = self.field.width
        for player in self.team_players[:2]:  # Máximo 2 aliados
            distancias_aliados.append(min(max_dist, player.distance_to_ball(self.ball)))
            orientaciones_aliados.append(min(np.pi, player.angle_difference_ball(self.ball)))

        for player in self.opponents[:2]:  # Máximo 2 rivales
            distancias_rivales.append(min(max_dist, player.distance_to_ball(self.ball)))
            orientaciones_rivales.append(min(np.pi, player.angle_difference_ball(self.ball)))

        # Completar listas si faltan jugadores
        while len(distancias_aliados) < 2:
            distancias_aliados.append(max_dist)
        while len(distancias_rivales) < 2:
            distancias_rivales.append(max_dist)
        while len(orientaciones_aliados) < 2:
            orientaciones_aliados.append(3.14)
        while len(orientaciones_rivales) < 2:
            orientaciones_rivales.append(3.14)

        # ==================== POSESION ====================

        # Input para la posesión de la pelota
        self.sim_posesion.input['distancia_aliado1'] = distancias_aliados[0]
        self.sim_posesion.input['distancia_aliado2'] = distancias_aliados[1]
        self.sim_posesion.input['distancia_rival1'] = distancias_rivales[0]
        self.sim_posesion.input['distancia_rival2'] = distancias_rivales[1]
        self.sim_posesion.input['orientacion_aliado1'] = orientaciones_aliados[0]
        self.sim_posesion.input['orientacion_aliado2'] = orientaciones_aliados[1]
        self.sim_posesion.input['orientacion_rival1'] = orientaciones_rivales[0]
        self.sim_posesion.input['orientacion_rival2'] = orientaciones_rivales[1]

        # NUEVAS ENTRADAS
        self.sim_posesion.input["velocidad_pelota"] = min(velocidad, 20)  # Limitar a rango
        self.sim_posesion.input["direccion_movimiento"] = direccion

        try:
            # posesion_resultado = self.sim_posesion.compute()
            self.sim_posesion.compute()
            log.debug(
                "POSESION completado %.2f", self.sim_posesion.output["posesion_pelota"]
            )
        except Exception as e:
            log.error("Error en sim_posesion: %s", e)

            # Fallback basado en distancia y orientación
            if (distancia_aliados[0] < 150 and orientaciones_aliados[0] < 0.4) or (
                distancia_aliados[1] < 150 and orientaciones_aliados[1] < 0.4
            ):
                posesion_resultado = 0.2  # Posesión aliada
            elif (distancias_rivales[0] < 150 and orientaciones_rivales[0] < 0.4) or (
                distancias_rivales[1] < 150 and orientaciones_rivales[1] < 0.4
            ):
                posesion_resultado = 0.8  # Posesión rival
            else:
                posesion_resultado = 0.5  # Libre

            # Actualizar manualmente el output del simulador para que esté disponible
            if not hasattr(self.sim_posesion, "output"):
                self.sim_posesion.output = {}
            self.sim_posesion.output["posesion_pelota"] = posesion_resultado

        # ==================== PROXIMIDAD ====================
        # Calcular ventaja
        ventaja_valor = calcular_ventaja_proximidad(distancias_aliados, distancias_rivales,
                                                    orientaciones_aliados, orientaciones_rivales,
                                                    self.team_players,self.opponents, self.ball)

        self.sim_proximidad.input["posesion_pelota_result"] = self.sim_posesion.output[
            "posesion_pelota"
        ]
        self.sim_proximidad.input["ventaja_proximidad"] = ventaja_valor
        self.sim_proximidad.input["velocidad_pelota_prox"] = min(velocidad, 20)  # Limitar a rango

        try:
            self.sim_proximidad.compute()
            log.debug("PROXIMIDAD completado")
        except Exception as e:
            log.error("Error en sim_proximidad: %s", e)
            # Fallback para proximidad basado en simple comparación
            if ventaja_valor < -50:  # Ventaja aliada
                proximidad_resultado = 0.3
            elif ventaja_valor > 50:  # Ventaja rival
                proximidad_resultado = 1.7
            else:  # Equilibrado
                proximidad_resultado = 1.0
            self.sim_proximidad.output = {"proximidad_equipo": proximidad_resultado}
            log.debug("Asignando proximidad de la pelota por defecto basando")

        # ==================== ZONA ====================

        # Inputs para determinar la zona de la pelota
        self.sim_zona.input["posicion_x"] = self.ball.get_position()[0]
        self.sim_zona.input["direccion_movimiento_zona"] = direccion

        try:
            self.sim_zona.compute()
            log.debug("ZONA completado")
        except Exception as e:
            log.error("Error en sim_zona: %s", e)
            log.debug(
                "Entradas: \nposición x de la pelota: %d", self.ball.get_position()[0]
            )

        # if self.side == "LEFT":
        # self.rol_atacante.view(sim=self.sim_rol)
        #     self.proximidad_equipo.view(sim=self.sim_proximidad)
        #     self.posesion_pelota.view(sim=self.sim_posesion)
        #     self.distancia_aliado1.view(sim=self.sim_proximidad)
        #     self.orientacion_aliado1.view(sim=self.sim_posesion)
        #     print(f"aliado 1 - distancia: {distancia_aliado1}  orientacion: {orientacion_aliado1}\n"
        #           f"aliado 2 - distancia: {distancia_aliado2}  orientacion: {orientacion_aliado2}\n"
        #           f"rival 1 - distancia: {distancia_rival1}  orientacion: {orientacion_rival1}\n"
        #           f"rival 2 - distancia: {distancia_rival2}  orientacion: {orientacion_rival2}\n")

        posesion = round(self.sim_posesion.output["posesion_pelota"], 1)
        proximidad = round(self.sim_proximidad.output["proximidad_equipo"], 1)
        zona = round(self.sim_zona.output["zona_pelota"], 1)

        self.role_assigner.assign_roles()

        # Retornar resultados
        return posesion, proximidad, zona
