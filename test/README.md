# Módulo de Pruebas - Robot Soccer

Suite de pruebas para validación del sistema de fútbol robot en modos simulación y hardware real.

## Estructura de Pruebas

```
test/
├── test_behavior_commands.py     # Pruebas de comportamiento y comandos
└── test_boardgame.py             # Prueba general
```

## Tipos de Pruebas

### 1. Pruebas de Comportamiento (`test_behavior_commands.py`)

**Propósito**: Validar árboles de comportamiento, toma de decisiones y comandos de robot.

**Funcionalidades**:
- Simulación visual interactiva con Matplotlib
- Visualización de estados y transiciones
- Monitoreo de acciones planificadas
- Control manual para debugging

**Ejecución**:
```bash
python test/test_behavior_commands.py
```

**Controles disponibles**:
- **ESC**: Salir de la simulación
- **R**: Reiniciar posiciones
- **SPACE**: Pausar/reanudar
- **Click**: Enfocar robot específico

### 2. Pruebas de Árboles de Decisión (`test/test_boardgame.py`)

**Propósito**:  Test interactivo para árboles de comportamiento y lógica difusa.

**Funcionalidades**:

- Interfaz gráfica interactiva con Matplotlib
- Configuración manual de posiciones de jugadores y pelota
- Evaluación en tiempo real del sistema difuso
- Visualización de roles dinámicos (atacante/defensor)
- Logging detallado de decisiones de IA

**Ejecución**:

```bash
python test/test_boardgame.py
```
**Controles disponibles**:

- **Click en jugador/pelota**: Abrir cuadro de diálogo para cambiar posición
- **Modificar X, Y, Ángulo**: Campos de texto editables
- **Enter**: Tecla para confirmar cambios
