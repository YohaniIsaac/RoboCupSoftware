# Módulo de Inteligencia Artificial - Robot Soccer

Sistema híbrido de IA que combina múltiples enfoques para el control inteligente de robots de fútbol en enfrentamientos 2vs2.

## Arquitectura General

```
ai/                                 # Inteligencia artificial
├── behavior_tree/                  # Árbol de comportamiento
│    ├── base.py
│    ├── manager.py
│    ├── soccer_behaviors.py
│    └── utils.py
│
├── fuzzy_logic  			        # Sistema lógico difuso
│    └── game_context.py            # Entrega el contexto del juego, donde está la pelota, equipo cercano
│
├── path_planning/                  # Algoritmos de planificación
│    ├── rrt_star_smart.py		    # Algoritmo RRT*
│    └── tools_for_path_planning.py # Herramientas para el algoritmo RRT
│
└── role_assignment/
     └── role_assigner.py
```

## Componentes Principales

### 1. Árboles de Comportamiento (behavior_tree/)

**Propósito**: Sistema jerárquico de toma de decisiones basado en behavior trees.

**Archivos clave**:

- `base.py`: Clases base (`BehaviorNode`, `NodeStatus`, `SequenceNode`, `SelectorNode`)
- `manager.py`: `BehaviorManager` - Gestor principal que integra todo el sistema
- `soccer_behaviors.py`: Comportamientos específicos de fútbol (atacante/defensor)
- `utils.py`: Utilidades para cálculos de posicionamiento

**Funcionalidades**:

- Estructura modular de decisiones jerárquicas
- Composición de comportamientos complejos (secuencias, selectores)
- Estados: RUNNING, SUCCESS, FAILURE
- Blackboard para compartir datos entre nodos

### 2. Sistema de Lógica Difusa (`fuzzy_logic/`)

**Propósito**: Evaluación del contexto del juego y toma de decisiones estratégicas.

**Archivos clave**:
- `game_context.py`: `FuzzyRobotTeamManager` - Gestión de contexto por equipos

**Funcionalidades**:
- Evaluación de proximidad a pelota
- Análisis de posición relativa de equipos
- Determinación de roles dinámicos (atacante/defensor)
- Cálculo de prioridades de acción

**Variables difusas**:
- Distancia a pelota (cerca/medio/lejos)
- Posición en campo (defensiva/neutral/ofensiva)
- Presión rival (baja/media/alta)

### 3. Planificación de Rutas (`path_planning/`)

**Propósito**: Generación de trayectorias libres de colisiones usando RRT*.

**Archivos clave**:
- `rrt_star_smart.py`: Implementación del algoritmo RRT* optimizado
- `tools_for_path_planning.py`: Utilidades geométricas y de validación

**Características**:
- Evitación dinámica de obstáculos (robots y límites de campo)
- Optimización de rutas en tiempo real
- Validación de trayectorias factibles
- Suavizado de caminos generados

**Parámetros configurables**:
```python
MAX_ITERATIONS = 1000      # Iteraciones máximas RRT*
STEP_SIZE = 30             # Tamaño de paso
GOAL_RADIUS = 25           # Radio de llegada a objetivo
```

### 4. Asignación de Roles (`role_assignment/`)

**Propósito**: Gestión dinámica de roles de robots según contexto del juego.

**Archivos clave**:
- `role_assigner.py`: `RoleAssigner` - Asignación inteligente atacante/defensor

**Funcionalidades**:
- Asignación automática basada en múltiples factores
- Intercambio dinámico de roles con cooldown
- Puntuación por distancia, orientación, posesión
- Factor de inercia para estabilidad

**Flujo de decisión**:
1. Percepción (entities/vision) → 
2. Contexto (fuzzy_logic) → 
3. Asignación de roles (role_assignment) → 
4. Ejecución de comportamientos (behavior_tree) → 
5. Planificación de rutas (path_planning) → 
6. Comandos a robots (controllers externos)

## Integración de Componentes

### Ciclo Principal de IA:

```python
# 1. Análisis de contexto difuso
context = fuzzy_manager.evaluar_ms_logic_difusse()

# 2. Asignación dinámica de roles
role_assignments = role_assigner.assign_roles()

# 3. Actualización de contexto en behavior trees
behavior_manager.update_game_context(context)

# 4. Ejecución de árboles de comportamiento
behavior_manager.execute_behaviors()

# 5. Planificación de rutas (si es necesario)
if target_position:
    path = rrt_planner.plan_path(current_pos, target_pos)
```
### Comportamientos Definidos

**Atacante (`create_attacker_tree()`):**

- **Con pelota**: Tiro → Pase → Avance con dribble
- **Sin pelota**: Captura → Intercepción → Apoyo ofensivo

**Defensor (`create_defender_tree()`):**

- **Zona defensiva**: Capturar pelota libre → Bloquear rival → Defender portería
- **Zona neutral/ofensiva**: Pasar si tiene pelota → Capturar en neutral → Posición defensiva

## Configuración de Roles

**Criterios de Asignación (RoleAssigner):**

```python
score = (
    0.35 * distancia_normalizada +      # Cercanía a pelota
    0.25 * orientacion_normalizada +    # Orientación hacia pelota
    0.20 * factor_posesion +            # Tiene la pelota
    0.10 * posicion_estrategica +       # Geometría del campo
    0.10 * factor_inercia               # Estabilidad de roles
)
```

**Estabilidad**:

- Cooldown mínimo: 10 frames entre cambios
- Factor de inercia: +0.2 puntos al atacante actual

## Parámetros de Configuración

### Configuración en `config.py`:

```python
# Lógica difusa
BALL_CAPTURE_DISTANCE = 25
ZONA_IZQUIERDA = 0.3
ZONA_DERECHA = 0.7

# Planificación
MAX_VELOCIDAD = 5.0
ROBOT_RADIO = 30

# Estados
TIMEOUT_SEARCH = 3.0  # segundos
COOLDOWN_KICK = 1.0   # segundos
```

## Extensión y Personalización

### Agregar nuevo estado:
1. Definir en `state_manager.py`
2. Implementar lógica de transición
3. Configurar comportamiento específico

### Modificar lógica difusa:
1. Ajustar variables lingüísticas en `game_context.py`
2. Redefinir reglas de inferencia
3. Calibrar funciones de membresía

### Nuevo algoritmo de planificación:
1. Implementar en `path_planning/`
2. Heredar de interfaz base
3. Integrar en controlador principal

## Optimización de Rendimiento

- **Caché de rutas**: Reutilización de trayectorias similares
- **Evaluación lazy**: Cálculo diferido de contexto
- **Paralelización**: Planificación concurrente por robot
- **Predicción**: Anticipación de movimientos rivales
