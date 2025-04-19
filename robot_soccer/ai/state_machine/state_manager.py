import numpy as np
import skfuzzy as fuzz
from skfuzzy import control as ctrl
import logging


class StateManager:
    def __init__(self):
        # ENTRADAS
        self.posesion_pelota = ctrl.Antecedent(np.arange(0, 1.1, 0.1), 'posesion_pelota')
        self.proximidad_equipo = ctrl.Antecedent(np.arange(0, 2.1, 0.1), 'proximidad_equipo')
        self.zona_pelota = ctrl.Antecedent(np.arange(0, 2.1, 0.1), 'zona_pelota')

        # SALIDAS
        self.estado_robot_cercano = ctrl.Consequent(np.arange(0, 1.1, 0.1), 'estado_robot_cercano')
        self.estado_robot_lejano = ctrl.Consequent(np.arange(0, 1.1, 0.1), 'estado_robot_lejano')

        self._func_membresia()
        self._rules()

        # Sistema difuso para la zona del campo
        self.control = ctrl.ControlSystem(self.reglas)
        self.simulation = ctrl.ControlSystemSimulation(self.control)

    def _func_membresia(self):
        # ENTRADAS
        self.posesion_pelota['posesion_aliada'] = fuzz.trimf(self.posesion_pelota.universe, [0, 0, 0.3])
        self.posesion_pelota['libre'] = fuzz.trimf(self.posesion_pelota.universe, [0.2, 0.5, 0.8])
        self.posesion_pelota['posesion_rival'] = fuzz.trimf(self.posesion_pelota.universe, [0.7, 1, 1])

        self.proximidad_equipo['aliado'] = fuzz.trimf(self.proximidad_equipo.universe, [0, 0, 1])
        self.proximidad_equipo['neutro'] = fuzz.trimf(self.proximidad_equipo.universe, [0.3, 1, 1.7])
        self.proximidad_equipo['rival'] = fuzz.trimf(self.proximidad_equipo.universe, [1, 2, 2])

        self.zona_pelota['defensiva'] = fuzz.trimf(self.zona_pelota.universe, [0, 0, 0.5])
        self.zona_pelota['neutral'] = fuzz.trimf(self.zona_pelota.universe, [0.4, 1, 1.6])
        self.zona_pelota['ofensiva'] = fuzz.trimf(self.zona_pelota.universe, [1.5, 2, 2])

        # SALIDAS
        # Funciones de membresía para acciones del robot cercano
        self.estado_robot_cercano['presionar'] = fuzz.trimf(self.estado_robot_cercano.universe,
                                                            [0, 0, 0.2])
        self.estado_robot_cercano['interceptar'] = fuzz.trimf(self.estado_robot_cercano.universe,
                                                              [0.1, 0.3, 0.5])
        self.estado_robot_cercano['capturar_pelota'] = fuzz.trimf(self.estado_robot_cercano.universe,
                                                                  [0.4, 0.6, 0.8])
        self.estado_robot_cercano['adelantar_lanzar'] = fuzz.trimf(self.estado_robot_cercano.universe,
                                                                   [0.7, 1, 1])

        # Funciones de membresía para acciones del robot lejano
        self.estado_robot_lejano['preparar_pase'] = fuzz.trimf(self.estado_robot_lejano.universe,
                                                               [0, 0, 0.2])
        self.estado_robot_lejano['marcar'] = fuzz.trimf(self.estado_robot_lejano.universe,
                                                        [0.1, 0.3, 0.5])
        self.estado_robot_lejano['posicion_defensiva'] = fuzz.trimf(self.estado_robot_lejano.universe,
                                                                    [0.4, 0.6, 0.8])
        self.estado_robot_lejano['bloquear_tiro'] = fuzz.trimf(self.estado_robot_lejano.universe,
                                                               [0.7, 1, 1])

    def _rules(self):
        # Si la pelota está libre y más cerca de un aliado
        regla1 = ctrl.Rule(self.posesion_pelota['libre'] & self.proximidad_equipo['aliado'] &
                           self.zona_pelota['defensiva'],
                           self.estado_robot_lejano['posicion_defensiva'])

        regla2 = ctrl.Rule((self.posesion_pelota['libre'] & self.proximidad_equipo['aliado'] &
                            (self.zona_pelota['neutral'] | self.zona_pelota['defensiva'] |
                             self.zona_pelota['ofensiva'])),
                           self.estado_robot_cercano['capturar_pelota'])

        regla3 = ctrl.Rule((self.posesion_pelota['libre'] & self.proximidad_equipo['aliado'] &
                            self.zona_pelota['ofensiva']),
                           self.estado_robot_lejano['preparar_pase'])

        regla4 = ctrl.Rule((self.posesion_pelota['libre'] & self.proximidad_equipo['rival'] &
                            (self.zona_pelota['neutral'] | self.zona_pelota['defensiva'])),
                           (self.estado_robot_cercano['interceptar'],
                            self.estado_robot_lejano['bloquear_tiro']))

        regla5 = ctrl.Rule((self.posesion_pelota['libre'] & self.proximidad_equipo['rival'] &
                            self.zona_pelota['ofensiva']),
                           (self.estado_robot_cercano['capturar_pelota'],
                            self.estado_robot_lejano['marcar']))

        regla6 = ctrl.Rule((self.posesion_pelota['posesion_aliada'] & self.proximidad_equipo['aliado'] &
                            (self.zona_pelota['neutral'] | self.zona_pelota['defensiva'])),
                           (self.estado_robot_cercano['adelantar_lanzar'],
                            self.estado_robot_lejano['preparar_pase']))

        regla7 = ctrl.Rule((self.posesion_pelota['posesion_aliada'] & self.proximidad_equipo['aliado'] &
                            self.zona_pelota['ofensiva']),
                           (self.estado_robot_cercano['adelantar_lanzar'],
                            self.estado_robot_lejano['preparar_pase']))

        regla8 = ctrl.Rule((self.posesion_pelota['posesion_rival'] & self.proximidad_equipo['rival'] &
                            (self.zona_pelota['neutral'] | self.zona_pelota['defensiva'])),
                           (self.estado_robot_cercano['presionar'],
                            self.estado_robot_lejano['bloquear_tiro']))

        regla9 = ctrl.Rule((self.posesion_pelota['posesion_rival'] & self.proximidad_equipo['rival'] &
                            self.zona_pelota['ofensiva']),
                           (self.estado_robot_cercano['presionar'],
                            self.estado_robot_lejano['posicion_defensiva']))

        self.reglas = [regla1, regla2, regla3, regla4, regla5, regla6, regla7, regla8, regla9]

    def evaluar_admEstados(self, posesion, proximidad, zona):
        self.simulation.input['posesion_pelota'] = posesion
        self.simulation.input['proximidad_equipo'] = proximidad
        self.simulation.input['zona_pelota'] = zona

        try:
            self.simulation.compute()
            # print(f"estado robot cercano/Ataque: {self.simulation.output['estado_robot_cercano']}\n"
            #       f"estado robot lejano/Defensa: {self.simulation.output['estado_robot_lejano']}")
        except Exception as e:
            logging.error(f"Error en sim_posesion: {e}")
            logging.debug(
                f"Entradas: \n"
                f"posición: {posesion}\n"
                f"proximida: {proximidad}\n"
                f"zona. \t{zona}")
        return {
            'estado robot cercano/Ataque': self.simulation.output['estado_robot_cercano'],
            'estado robot lejano/Defensa': self.simulation.output['estado_robot_lejano']
        }