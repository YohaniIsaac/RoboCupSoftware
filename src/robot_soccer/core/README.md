# Módulo Core - Robot Soccer

Motor central del sistema que maneja la simulación, física del juego y coordinación de procesos múltiples.

## Arquitectura

```
core/
├── game_controller.py      # Controlador principal multiproceso
├── physics.py              # Motor físico y colisiones
└── process/                # Procesos independientes
    ├── ball_search.py          # Proceso de búsqueda de pelota
    ├── main_simulation.py      # Simulación principal con pygame
    ├── path.py                 # Proceso de planificación de rutas
    └── search_for_players.py   # Proceso de búsqueda de jugadores
```

## Componentes Principales

### 1. Controlador de Juego (`game_controller.py`)

**Propósito**: Orquestador principal que gestiona la ejecución multiproceso del sistema completo.

**Función principal**: `execute_multiprocessing()`

**Procesos creados**:
- **p1**: Simulación principal (pygame + renderizado)
- **p2**: Búsqueda de pelota (visión + tracking)
- **p3**: Búsqueda de jugadores (ArUco + detección)
- **p4**: Planificación de trayectorias (RRT* + pathfinding)

**Comunicación entre procesos**:
```python
# Pipes para intercambio de frames
fr2ball_env, fr2ball_recv = multiprocessing.Pipe()
fr2player_env, fr2player_recv = multiprocessing.Pipe() 
fr2traj_env, fr2traj_recv = multiprocessing.Pipe()

# Cola para rutas planificadas
env_ruta = multiprocessing.Queue()
```

### 2. Motor Físico (`physics.py`)

**Propósito**: Sistema de detección y resolución de colisiones con respuesta física realista.

**Función principal**: `detectar_colisiones()`

**Características**:
- Detección de colisiones entre todos los objetos
- Resolución de impulsos basada en conservación de momento
- Coeficiente de restitución configurable
- Separación automática para evitar superposición
- Sistema de cooldown para colisiones

### 3. Simulación Principal (`process/main_simulation.py`)

**Propósito**: Motor de renderizado y física principal usando pygame.

**Función principal**: `simulacion_principal()`

**Responsabilidades**:
- Inicialización de pygame y ventana de juego
- Creación de jugadores (`Player4Simulation`) y pelota (`Ball4Simulation`)
- Bucle principal de renderizado y actualización
- Conversión de frames pygame a OpenCV
- Distribución de frames a procesos de visión

### 4. Procesos de Visión

#### Búsqueda de Pelota (`process/ball_search.py`)

**Función**: `busqueda_ball(fr2ball_recv, ball_send)`

**Algoritmo**:
1. Recibir frame desde simulación
2. Convertir a espacio HSV  
3. Aplicar detección/seguimiento de pelota
4. Enviar coordenadas (x, y) al sistema principal

#### Búsqueda de Jugadores (`process/search_for_players.py`)

**Función**: `busqueda_player(fr2player_recv, player_send)`

**Algoritmo**:
1. Recibir frame desde simulación
2. Detectar marcadores ArUco
3. Calcular posición y orientación
4. Enviar datos estructurados al sistema principal

#### Planificación de Rutas (`process/path.py`)

**Función**: `trayectoria(env_ruta, ...)`

**Responsabilidades**:
- Recibir solicitudes de planificación
- Ejecutar algoritmo RRT*
- Enviar rutas optimizadas
- Coordinar con sistema de IA

## Configuración de Simulación

### Parámetros del Campo

```python
# En config.py
ANCHO_CAMPO = 1500      # Ancho del campo
ALTO_CAMPO = 900        # Alto del campo  
ANCHO_TOTAL = 1600      # Incluyendo márgenes
ALTO_TOTAL = 1000       # Incluyendo márgenes
FPS = 60                # Frames por segundo
```

### Parámetros de Física

```python
# Colisiones
COEF_RESTITUCION = 1.0  # Elasticidad (0=inelástico, 1=elástico)
COOLDOWN_COLISION = 8   # Frames entre colisiones
FACTOR_SEPARACION = 0.7 # Factor de separación

# Movimiento
MAX_VELOCIDAD = 5.0     # Velocidad máxima
FACTOR_ROCE = 0.5       # Desaceleración por fricción
```

## Comunicación entre Procesos

### Tipos de Datos Intercambiados

**Frames de video**:
```python
# Formato: numpy.ndarray (BGR)
frame = np.array([altura, anchura, 3], dtype=np.uint8)
```

**Coordenadas de pelota**:
```python
ball_coords = (x, y)  # tupla de enteros
```

**Datos de jugadores**:
```python
player_data = [
    {'id': 1, 'x': 200, 'y': 300, 'angulo': 45.0, 'esquinas': [...]},
    # ... más jugadores
]
```

**Rutas planificadas**:
```python
route_data = {
    'player_id': 1,
    'path': [(x1, y1), (x2, y2), ..., (x_goal, y_goal)],
    'timestamp': time.time()
}
```

### Sincronización

```python
# Uso de pipes para comunicación bidireccional
send_pipe.send(data)
received_data = recv_pipe.recv()

# Uso de colas para datos asíncronos
queue.put(route_request)
route_response = queue.get()
```

## Optimización de Rendimiento

### Técnicas Implementadas

- **Multiprocesing**: Distribución de carga en múltiples cores
- **ROI tracking**: Reducción de área de procesamiento
- **Frame skipping**: Saltar frames si el procesamiento es lento
- **Object pooling**: Reutilización de objetos pygame


## Troubleshooting

### Problemas Comunes

**Procesos no se comunican**:
- Verificar que pipes estén correctamente conectados
- Comprobar que ambos extremos estén activos
- Validar formato de datos enviados

**Simulación lenta**:
- Reducir FPS objetivo
- Disminuir resolución de ventana
- Optimizar algoritmos de física

**Errores de pygame**:
- Verificar inicialización correcta
- Comprobar drivers de video
- Validar resolución de pantalla soportada
