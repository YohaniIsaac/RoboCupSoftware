from skfuzzy import control as ctrl
import numpy as np

# Variables de entrada
distancia = ctrl.Antecedent(np.arange(0, 100, 1), 'distancia')
angulo = ctrl.Antecedent(np.arange(-180, 180, 1), 'angulo')

# Variables de salida
velocidad = ctrl.Consequent(np.arange(0, 10, 1), 'velocidad')
direccion = ctrl.Consequent(np.arange(-180, 180, 1), 'direccion')

# Funciones de membresía para la distancia
distancia['cerca'] = fuzz.trimf(distancia.universe, [0, 0, 50])
distancia['media'] = fuzz.trimf(distancia.universe, [0, 50, 100])
distancia['lejos'] = fuzz.trimf(distancia.universe, [50, 100, 100])

# Funciones de membresía para el ángulo
angulo['alineado'] = fuzz.trimf(angulo.universe, [-10, 0, 10])
angulo['desviado'] = fuzz.trimf(angulo.universe, [-90, -10, 10, 90])
angulo['opuesto'] = fuzz.trimf(angulo.universe, [-180, -90, 90, 180])

# Funciones de membresía para la velocidad
velocidad['lenta'] = fuzz.trimf(velocidad.universe, [0, 0, 5])
velocidad['media'] = fuzz.trimf(velocidad.universe, [0, 5, 10])
velocidad['rapida'] = fuzz.trimf(velocidad.universe, [5, 10, 10])

# Funciones de membresía para la dirección
direccion['hacia_pelota'] = fuzz.trimf(direccion.universe, [-10, 0, 10])
direccion['ajustar_angulo'] = fuzz.trimf(direccion.universe, [-90, -10, 10, 90])

# Reglas difusas
regla1 = ctrl.Rule(distancia['cerca'] & angulo['alineado'], velocidad['rapida'])
regla2 = ctrl.Rule(distancia['media'] & angulo['desviado'], velocidad['media'])
regla3 = ctrl.Rule(distancia['lejos'] & angulo['opuesto'], velocidad['lenta'])

# Sistema de control
sistema_control = ctrl.ControlSystem([regla1, regla2, regla3])
simulador = ctrl.ControlSystemSimulation(sistema_control)

# Ejemplo de uso
simulador.input['distancia'] = 30  # Distancia a la pelota
simulador.input['angulo'] = 15     # Ángulo hacia la pelota
simulador.compute()

print("Velocidad:", simulador.output['velocidad'])
print("Dirección:", simulador.output['direccion'])