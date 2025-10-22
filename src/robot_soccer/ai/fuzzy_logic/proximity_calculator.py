import logging
import numpy as np

log = logging.getLogger(__name__)

# Constantes para mejor legibilidad
DISTANCIA_MAX_NORMALIZACION = 1200  # Aumentado de 800 para campo más grande
VELOCIDAD_MAX_ROBOT = 10
VELOCIDAD_BASE_ROBOT = 5
VELOCIDAD_MAX_PELOTA = 20
PENALIZACION_PELOTA_RAPIDA = 0.3

# Pesos de factores (DISTANCIA es el factor más importante)
PESO_DISTANCIA = 0.6    # Aumentado de 0.4 para dar más peso a la distancia
PESO_ORIENTACION = 0.15  # Reducido de 0.3 (orientación es secundaria)
PESO_VELOCIDAD = 0.15    # Reducido de 0.2
PESO_TIEMPO = 0.1        # Mantener igual

# Factores de rol
FACTOR_ROL_ATACANTE = 1.2
FACTOR_ROL_DEFENSOR = 0.9

# Combinación de ventajas
PESO_MEJOR_JUGADOR = 0.7
PESO_EQUIPO_COMPLETO = 0.3


def _calcular_efectividad_jugador(player, distancia, orientacion):
    """Calcula la efectividad de un jugador para alcanzar la pelota.

    Args:
        player: Objeto jugador con atributos de posición y estado
        distancia: Distancia euclidiana a la pelota (píxeles)
        orientacion: Diferencia angular hacia la pelota (radianes)

    Returns:
        tuple: (efectividad, tiempo_llegada)
            - efectividad (float): Valor entre 0-1 indicando qué tan efectivo
              es el jugador para alcanzar la pelota.
            - tiempo_llegada (float): Tiempo estimado en unidades para
              llegar a la pelota.
    """
    # 1. Factor de distancia (0-1, 1 = muy cerca)
    factor_distancia = max(0, 1 - distancia / DISTANCIA_MAX_NORMALIZACION)

    # 2. Factor de orientación mejorado (0-1, 1 = perfectamente orientado)
    # Penalizar más los ángulos malos: usar coseno al cuadrado para amplificar diferencias
    cos_orientacion = np.cos(orientacion)
    if cos_orientacion > 0:
        factor_orientacion = cos_orientacion ** 1.5  # Amplifica diferencias
    else:
        factor_orientacion = 0.05  # Ángulos > 90° son muy malos

    # 3. Factor de velocidad del robot (más robusto)
    factor_velocidad = 0.7  # Valor por defecto

    # Verificar si el player tiene información de velocidad
    if hasattr(player, 'velocity'):
        try:
            if hasattr(player.velocity, 'magnitude'):
                velocidad_robot = min(player.velocity.magnitude, VELOCIDAD_MAX_ROBOT)
                factor_velocidad = 0.5 + (velocidad_robot / (VELOCIDAD_MAX_ROBOT * 2))
            elif isinstance(player.velocity, (int, float)):
                velocidad_robot = min(player.velocity, VELOCIDAD_MAX_ROBOT)
                factor_velocidad = 0.5 + (velocidad_robot / (VELOCIDAD_MAX_ROBOT * 2))
        except (AttributeError, TypeError):
            pass  # Mantener valor por defecto

    # 4. Factor de rol (más robusto)
    factor_rol = 1.0  # Valor por defecto

    if hasattr(player, 'rol'):
        if player.rol in ['atacante', 'ATACANTE', 'attacker']:
            factor_rol = FACTOR_ROL_ATACANTE
        elif player.rol in ['defensor', 'DEFENSOR', 'defender']:
            factor_rol = FACTOR_ROL_DEFENSOR

    # 5. Tiempo estimado de llegada
    velocidad_efectiva = factor_velocidad * VELOCIDAD_BASE_ROBOT
    tiempo_llegada = distancia / max(1, velocidad_efectiva)
    tiempo_llegada *= (1 + orientacion)  # Penalizar mala orientación

    # 6. Efectividad combinada
    efectividad = (
        factor_distancia * PESO_DISTANCIA +
        factor_orientacion * PESO_ORIENTACION +
        factor_velocidad * PESO_VELOCIDAD +
        (1 / max(tiempo_llegada, 1)) * PESO_TIEMPO
    ) * factor_rol

    return efectividad, tiempo_llegada


def calcular_ventaja_proximidad(distancias_aliados, distancias_rivales,
                               orientaciones_aliados, orientaciones_rivales,
                               team_players, opponents, ball):
    """Calcula la ventaja de proximidad considerando múltiples factores.

    Evalúa qué equipo tiene mayor ventaja para alcanzar la pelota basándose
    en las distancias, orientaciones y características de los jugadores.

    Args:
        distancias_aliados (list): Lista de distancias de jugadores aliados
            a la pelota en píxeles.
        distancias_rivales (list): Lista de distancias de jugadores rivales
            a la pelota en píxeles.
        orientaciones_aliados (list): Lista de orientaciones de jugadores
            aliados hacia la pelota en radianes.
        orientaciones_rivales (list): Lista de orientaciones de jugadores
            rivales hacia la pelota en radianes.
        team_players (list): Lista de objetos jugadores del equipo.
        opponents (list): Lista de objetos jugadores rivales.
        ball: Objeto pelota con método get_velocity().

    Returns:
        int: Ventaja de proximidad entre -1000 y 1000.
            - Valores POSITIVOS indican ventaja del equipo aliado (más cerca).
            - Valores NEGATIVOS indican ventaja del equipo rival (aliado más lejos).
            - Cero indica equilibrio entre equipos.
    """
    # Obtener equipo para logging
    team = team_players[0].team if team_players else "unknown"

    # Calcular efectividades para aliados
    efectividades_aliados = []
    tiempos_aliados = []

    for i, player in enumerate(team_players[:2]):
        if i < len(distancias_aliados):
            efectividad, tiempo = _calcular_efectividad_jugador(
                player, distancias_aliados[i], orientaciones_aliados[i]
            )
            efectividades_aliados.append(efectividad)
            tiempos_aliados.append(tiempo)

    # Calcular efectividades para rivales
    efectividades_rivales = []
    tiempos_rivales = []

    for i, player in enumerate(opponents[:2]):
        if i < len(distancias_rivales):
            efectividad, tiempo = _calcular_efectividad_jugador(
                player, distancias_rivales[i], orientaciones_rivales[i]
            )
            efectividades_rivales.append(efectividad)
            tiempos_rivales.append(tiempo)

    # Completar listas si faltan jugadores (valores que indican ausencia)
    while len(efectividades_aliados) < 2:
        efectividades_aliados.append(0.1)
        tiempos_aliados.append(1000)
    while len(efectividades_rivales) < 2:
        efectividades_rivales.append(0.1)
        tiempos_rivales.append(1000)

    # Estrategias de cálculo de ventaja

    # 1. Mejor jugador de cada equipo
    mejor_aliado = max(efectividades_aliados)
    mejor_rival = max(efectividades_rivales)

    # 2. Suma total de efectividades
    suma_efectividad_aliada = sum(efectividades_aliados)
    suma_efectividad_rival = sum(efectividades_rivales)

    # 3. Factor de velocidad de pelota (más difícil interceptar si va rápido)
    try:
        _, _, velocidad_pelota = ball.get_velocity()
        factor_pelota_rapida = 1 + (velocidad_pelota / VELOCIDAD_MAX_PELOTA) * PENALIZACION_PELOTA_RAPIDA
    except (AttributeError, TypeError, ValueError):
        log.warning("No se pudo obtener velocidad de la pelota, usando factor por defecto")
        factor_pelota_rapida = 1.0

    # 4. Ventaja combinada (70% mejor jugador + 30% equipo completo)
    # Valores POSITIVOS = aliado más cerca, NEGATIVOS = rival más cerca
    # Esto luego se transforma para que coincida con la escala de proximidad
    ventaja_mejor_jugador = (mejor_aliado - mejor_rival) * 150  # Aumentado de 100 a 150
    ventaja_equipo = (suma_efectividad_aliada - suma_efectividad_rival) * 75  # Aumentado de 50 a 75

    ventaja_total = (ventaja_mejor_jugador * PESO_MEJOR_JUGADOR +
                    ventaja_equipo * PESO_EQUIPO_COMPLETO)

    # 5. Aplicar factor de pelota rápida
    ventaja_total /= factor_pelota_rapida

    # 6. Escalar y limitar resultado (amplificado para mayor sensibilidad)
    ventaja_final = max(-1000, min(1000, int(ventaja_total * 15)))  # Aumentado de 10 a 15

    efectividades_aliados_fmt = [f'{e:.2f}' for e in efectividades_aliados]
    efectividades_rivales_fmt = [f'{e:.2f}' for e in efectividades_rivales]
    # Log detallado para debug
    log.debug("Equipo%s: "
             "Efectividades aliados=%s, "
             "rivales=%s, "
             "vel_pelota=%.1f, ventaja=%s",
             team, efectividades_aliados_fmt, efectividades_rivales_fmt,
             velocidad_pelota, ventaja_final)

    return ventaja_final
