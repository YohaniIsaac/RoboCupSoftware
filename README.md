# Robot Soccer - Sistema de Enfrentamiento Autónomo 2vs2

Sistema completo para enfrentamientos autónomos de fútbol robot 2 contra 2, con capacidad de simulación y control de robots reales mediante Arduino y comunicación RF.

## Características Principales

- **Enfrentamientos autónomos**: 2 equipos de 2 robots cada uno
- **Doble modalidad**: Simulación visual y control de robots físicos
- **Detección visual**: Seguimiento de jugadores y pelota usando ArUco markers y OpenCV
- **IA**: Sistema híbrido con lógica difusa, árboles de comportamiento y planificación de rutas
- **Comunicación RF**: Control remoto via Arduino Nano RF

## Requisitos del Sistema

### Software
- Python >= 3.10
- OpenCV >= 4.9.0
- Pygame >= 2.5.2
- NumPy, Matplotlib, SciPy
- PySerial (para comunicación con Arduino)

### Hardware
- Arduino Nano con módulo RF (NRF24L01)
- Cámara para captura de video
- Marcadores ArUco impresos
- PlatformIO Core (para compilar firmware Arduino)

## Instalación

1. **Clonar repositorio:**
```bash
git clone <tu-repositorio>
cd robot_soccer
```

2. **Instalar dependencias:**
```bash
pip install -r requirements.txt
```

3. **Configurar ArUco markers:**
   - Los marcadores deben ubicarse en `arucoMarkers/`
   - Configura las IDs de robots en `src/robot_soccer/config.py`

4. **Compilar firmware (opcional, para robots físicos):**
   - Ver la guía completa en `firmware/README.md`
   ```bash
   cd firmware/msr
   pio run -t upload --upload-port /dev/ttyUSB0
   ```

## Uso Rápido

### Simulación
```bash
python -m robot_soccer
```

### Scripts de Prueba
```bash
# Pruebas de comportamiento
python test/test_behavior_commands.py

```
## Estructura del Proyecto

```
robot_soccer/                           # Directorio raíz del proyecto
│
├── main.py                             # Punto de entrada principal
├── config.py                           # Configuración centralizada. Constantes globales
│
├── core/                               # Módulo core con funcionalidad central
│   ├── game_controller.py              # Inicialización y control de cada uno de los juegos
│   ├── process/   					    # Funciones de cada uno de los procesos por separado
│   │    ├── __init__.py
│   │    ├── main_simulation.py  		# Función para iniciar la simulación principal
│   │    ├── ball_search.py          	# Función para iniciar y realizar la búsqueda de la pelota
│   │    ├── search_for_players.py      # Función para iniciar y realizar la búsqueda de los jugadores
│   │    └── path.py            		# Función para iniciar y realizar la creación de rutas
│   └── physics.py                      # Motor físico y colisiones
│
├── perception/                         # Módulo de percepción
│   ├── ball_tracking.py                # Seguimiento de pelota
│   └── player_tracking.py              # Seguimiento de jugadores
│
├──entities/				            # Almacena los archivos de las Clases de los participantes
│   ├── ball.py                         # Clase Ball para la toma de decisiones
│   ├── player.py                       # Clase Player para la toma de decisiones
│   └── simulation/			            # Almacena los archivos de las Clases para crear la simulación
│        ├── player_sim.py              # Player4Simulation. Clase para crear a los jugadores para simulación
│        └── ball_sim.py                # Ball4Simulation. Clase para crear a la pelota para la simulación
│
├── ai/                                 # Inteligencia artificial
│   ├── behavior_tree/
│   │    ├── base.py
│   │    ├── manager.py
│   │    ├── soccer_behaviors.py
│   │    └── utils.py
│   │
│   ├── fuzzy_logic  			        # Sistema lógico difuso
│   │    └── game_context.py            # Entrega el contexto del juego, donde está la pelota, equipo cercano,
│   │
│   ├── path_planning/                  # Algoritmos de planificación
│   │    ├── rrt_star_smart.py		    # Algoritmo RRT*
│   │    └── tools_for_path_planning.py # Herramientas para el algoritmo RRT
│   │
│   └── role_assignment/
│       └── role_assigner.py
│
├── controllers/                    # Controladores específicos
│    └── robot_controller.py        # Controlador de robot
│
├── utils/                              # Utilidades
│    └── tools.py                       # Herramientas generales
│
└── firmware/                           # Firmware para Arduino (ver firmware/README.md)
    ├── msr/                            # Robot MSR (control de motores, RF)
    │   ├── src/                        # Código fuente
    │   │   ├── main.cpp
    │   │   └── robot_control.cpp
    │   ├── include/                    # Headers
    │   │   ├── config.h
    │   │   └── robot_control.h
    │   └── platformio.ini              # Configuración PlatformIO
    └── tablero/                        # Tablero de marcador/cronómetro
        ├── src/
        │   ├── main.cpp
        │   ├── display.cpp
        │   └── game_control.cpp
        ├── include/
        │   ├── config.h
        │   ├── display.h
        │   └── game_control.h
        └── platformio.ini
```

## Configuración

### Campo de Juego
Modifica `config.py` para ajustar:
- Dimensiones del campo (`ANCHO_CAMPO`, `ALTO_CAMPO`)
- Parámetros de robots (`ROBOT_RADIO`, `MAX_VELOCIDAD`)
- Configuración de pelota (`PELOTA_RADIO`, `PELOTA_MASA`)

## Modos de Operación

### 1. Simulación Pura
- Visualización con Pygame
- Physics engine completo
- Sin hardware requerido

### 2. Robots Reales
- Detección via cámara
- Comandos enviados por RF
- Feedback visual en tiempo real

### 3. Modo Híbrido
- Simulación para desarrollo
- Switch fácil a hardware real



