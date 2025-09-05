# Módulo Entities - Robot Soccer

Clases fundamentales que representan los objetos del juego: jugadores y pelota, con versiones específicas para lógica de IA y simulación visual.

## Arquitectura

```
entities/
├── ball.py             # Clase Ball para lógica de juego
├── player.py           # Clase Player para lógica de IA y decisiones
└── simulation/         # Versiones específicas para simulación pygame
    ├── ball_sim.py        # Ball4Simulation para renderizado y física
    └── player_sim.py      # Player4Simulation para renderizado y física
```

## Dualidad de Clases

El módulo implementa **dos versiones** de cada entidad:

### **Versión Lógica** (IA y Decisiones)
- `Player` - Para sistema de IA, behavior trees y toma de decisiones
- `Ball` - Para cálculos de posición y estado del juego

### **Versión Simulación** (Renderizado y Física)  
- `Player4Simulation` - Para renderizado pygame y física de movimiento
- `Ball4Simulation` - Para renderizado pygame y simulación física

## Entidades Principales

### 1. Player (`player.py`)

**Propósito**: Representación lógica del jugador para sistema de IA.

**Características principales**:
- Almacenamiento de estado (posición, orientación, rol)
- Métodos para toma de decisiones de IA
- Cálculos geométricos (distancias, ángulos)
- Interfaz con behavior trees y fuzzy logic

### 2. Ball (`ball.py`)

**Propósito**: Representación lógica de la pelota para cálculos de juego.

**Características principales**:
- Estado y posición de la pelota
- Cálculos de trayectoria y velocidad
- Métodos para predicción de movimiento

### 3. Player4Simulation (`simulation/player_sim.py`)

**Propósito**: Jugador para simulación física y renderizado pygame.

**Características principales**:
- Física de movimiento realista
- Renderizado visual con pygame
- Colisiones con otros objetos
- Interacción con pelota (captura y disparo)

### 4. Ball4Simulation (`simulation/ball_sim.py`)

**Propósito**: Pelota para simulación física y renderizado pygame.

**Características principales**:
- Física de movimiento con inercia y fricción
- Colisiones realistas con jugadores y bordes
- Renderizado visual
- Sistema de posesión con jugadores

## Flujo de Datos

```
[Simulación pygame] → [Visión OpenCV] → [Entidades lógicas] → [Sistema IA] → [Comandos] → [Simulación pygame]
     ↓                      ↓                    ↓                ↓              ↓              ↓
Player4Simulation    → detección ArUco → Player.update() → BehaviorTree → move_to() → Player4Simulation
Ball4Simulation      → seguimiento     → Ball.update()   → FuzzyLogic  → actions   → Ball4Simulation
```
