# Módulo de Percepción - Robot Soccer

Sistema de visión computacional para detección y seguimiento en tiempo real de jugadores y pelota en el campo de fútbol robot.

## Arquitectura

```
perception/
├── ball_tracking.py      # Seguimiento de pelota por color
└── player_tracking.py    # Detección de jugadores con ArUco tags
```

## Componentes Principales

### 1. Seguimiento de Pelota (`ball_tracking.py`)

**Propósito**: Detección y seguimiento de pelota mediante segmentación por color en espacio HSV.

**Clase principal**: `Ball`

**Algoritmo**:
1. **Detección inicial**: Círculos Hough en imagen HSV filtrada
2. **Seguimiento optimizado**: ROI (Región de Interés) alrededor de posición previa
3. **Validación**: Filtros de tamaño y forma para eliminar falsos positivos

**Funcionalidades**:
- Detección automática por color configurable
- Seguimiento con ROI para optimización de rendimiento
- Detección de goles automática
- Contador de puntuación por equipo

### 2. Detección de Jugadores (`player_tracking.py`)

**Propósito**: Identificación de robots mediante marcadores ArUco con cálculo de posición y orientación.

**Función principal**: `deteccion_jugadores_aruco_tag(frame)`

**Algoritmo**:
1. **Conversión a escala de grises**
2. **Detección ArUco**: Diccionario DICT_7X7_1000
3. **Cálculo de orientación**: Vector desde esquinas del marcador
4. **Generación de rectángulo**: Representación del robot (104x140 px)

**Datos extraídos por jugador**:
- ID único del marcador
- Coordenadas del centro (x, y)
- Ángulo de orientación en grados
- Esquinas del rectángulo rotado

## Visualización y Debug

### Elementos Visuales Dibujados

**Pelota**:
- Círculo negro en el centro detectado
- Marcador de goles en pantalla
- ROI (región de interés) resaltada

**Jugadores**:
- Contornos verdes del rectángulo del robot
- Círculos azules en las esquinas
- Línea verde indicando orientación
- ID del marcador como texto


## Optimizaciones de Rendimiento

### ROI (Región de Interés)

```python
# Recorte optimizado para seguimiento de pelota
vecindad = 40  # píxeles alrededor de última posición
```

**Beneficios**:
- Reduce área de procesamiento en ~95%
- Mejora FPS significativamente
- Mantiene precisión de detección

### Filtros de Validación

```python
# Filtros para círculos válidos
if radius > min_radius and radius < max_radius:
    # Círculo válido para pelota
    return x, y, radius
```

## Extensiones Futuras

- Predicción de posición futura de pelota
- Calibración automática de color
- Detección de líneas del campo para corrección de perspectiva
