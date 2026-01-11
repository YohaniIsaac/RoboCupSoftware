#!/usr/bin/env python3
"""Script de calibración multi-punto BIDIRECCIONAL de motores por robot.

Este script permite calibrar motores con CALIBRACIÓN BIDIRECCIONAL PERSONALIZADA:
- Cada dirección (adelante/atrás) tiene su propia curva de calibración
- Cada robot tiene sus PUNTOS PERSONALIZADOS basados en su rango PWM útil
- Compensa la respuesta asimétrica de motores DC montados en orientación opuesta

PUNTOS PERSONALIZADOS POR ROBOT:
Los puntos de calibración se generan automáticamente basados en el rango PWM
determinado con 'calibrate_robot_pwm_range.py'. Por ejemplo:

Robot 0: Rango [15, 60] → Puntos: -60, -48, -37, -26, -15, 15, 26, 37, 48, 60
Robot 1: Rango [20, 75] → Puntos: -75, -60, -45, -32, -20, 20, 32, 45, 60, 75

Cada robot tiene 10 puntos (5 adelante + 5 atrás) distribuidos uniformemente
en SU RANGO ÚTIL, garantizando:
✓ Todos los puntos son útiles (no hay desperdicio)
✓ Cobertura completa del rango operativo
✓ Mejor distribución de puntos de calibración

CALIBRACIÓN POR RUEDA:
- Cada rueda usa su calibración según su dirección de giro
- Ejemplo: Al girar, rueda derecha (+50) usa calibración ADELANTE,
           rueda izquierda (-50) usa calibración ATRÁS
- Esto garantiza la misma velocidad angular en ambas ruedas

DEAD-ZONE INDIVIDUAL:
- Cada motor tiene su propio valor de PWM mínimo para empezar a moverse
- Se calibra independientemente para motor izquierdo y derecho
- Ajustable con z/x (motor izq) y c/v (motor der)
- Modo de prueba dedicado para encontrar el valor exacto (tecla 't')

Uso:
    python scripts/calibrate_robot_motors_multipoint.py --robot-id 0

Controles:
    NAVEGACIÓN ENTRE PUNTOS:
    PgUp/PgDn: Cambiar entre los 5 puntos de calibración

    CALIBRACIÓN DEL PUNTO ACTUAL:
    q/a: max_speed_left   (±0.05) - Ajuste GRUESO
    w/s: max_speed_right  (±0.05) - Ajuste GRUESO
    e/d: bias_correction  (±0.01) - Ajuste GRUESO
    1/2: max_speed_left   (±0.001) - Ajuste FINO
    3/4: max_speed_right  (±0.001) - Ajuste FINO
    5/6: bias_correction  (±0.001) - Ajuste FINO

    AJUSTE DE DEAD-ZONE:
    z/x: Motor izquierdo  (±1 PWM)
    c/v: Motor derecho    (±1 PWM)
    t: Activar modo de prueba de dead-zone

    MODO PRUEBA DEAD-ZONE (presiona 't' para activar):
    7/8: Control directo motor izquierdo (±1 PWM)
    9/0: Control directo motor derecho   (±1 PWM)
    - Encuentra el PWM mínimo donde el motor empieza a moverse
    - Usa z/x/c/v para guardar los valores encontrados
    - Presiona 't' de nuevo para volver a modo multi-punto

    PARÁMETROS DE MOVIMIENTO:
    [/]: Duración del movimiento (±0.05s, ajuste grueso)
    -/=: Duración del movimiento (±0.01s, ajuste fino)
    Rango: 0.05s - 5.0s (default: 0.3s)

    MOVIMIENTO:
    Flechas: Mover robot a la velocidad del punto actual
        ↑: Adelante
        ↓: Atrás
        ←: Girar izquierda
        →: Girar derecha

    OTROS:
    r: Reset punto actual a neutro (1.0, 1.0, 0.0)
    ESPACIO: Stop emergencia
    ENTER: Guardar TODA la calibración (10 puntos + dead-zone)
    ESC: Salir sin guardar

Workflow recomendado:
    PASO PREVIO - DETERMINAR RANGO PWM (OBLIGATORIO):
    0a. Ejecuta: python scripts/calibrate_robot_pwm_range.py --robot-id X
    0b. Encuentra PWM_min: Más bajo donde el robot se mueve bien
    0c. Encuentra PWM_max: Más alto donde la cámara detecta al robot
    0d. Ajusta rango con n/m (min) y ,/. (max), presiona 'g' para guardar
    0e. Los 10 puntos de calibración se generan automáticamente

    PASO 1 - CALIBRAR DEAD-ZONE (opcional pero recomendado):
       - Presiona 't' para entrar en modo de prueba de dead-zone
       - Motor izquierdo: Usa 7/8 hasta encontrar PWM mínimo de movimiento
       - Motor derecho: Usa 9/0 hasta encontrar PWM mínimo de movimiento
       - Anota estos valores y ajústalos con z/x (izq) y c/v (der)
       - Presiona 't' para volver a modo multi-punto

    PASO 2 - CALIBRACIÓN ADELANTE (últimos 5 puntos):
    1. Primer punto adelante (ej: +15 PWM) - Velocidad mínima adelante
       - Presiona ↑ para mover adelante
       - Ajusta bias (e/d) hasta que vaya recto
       - Ajusta max_left/max_right (q/a/w/s) si un motor va más rápido

    2. Siguientes puntos adelante (usa PgDn para avanzar)
       - Calibra cada punto moviéndote hacia adelante
       - Ajusta bias si se desvía
       - Verifica que vaya recto en TODOS los puntos

    PASO 3 - CALIBRACIÓN ATRÁS (primeros 5 puntos):
    3. Último punto atrás (ej: -15 PWM) - Velocidad mínima atrás
       - Presiona ↓ para mover atrás
       - Ajusta bias (e/d) hasta que vaya recto
       - La calibración es INDEPENDIENTE de adelante

    4. Siguientes puntos atrás (usa PgUp para retroceder)
       - Calibra cada punto moviéndote hacia atrás
       - Ajusta bias si se desvía
       - Verifica que vaya recto en TODOS los puntos

    PASO 4 - PROBAR GIROS:
       - En cada punto, presiona ← o → para girar
       - El giro debe ser simétrico (cada rueda usa su calibración)
       - Rueda que va adelante usa calibración ADELANTE
       - Rueda que va atrás usa calibración ATRÁS

    PASO 5 - GUARDAR:
    5. Presionar ENTER para guardar los 10 puntos + dead-zone

Objetivo:
    - Robot va recto ADELANTE a cualquier velocidad en su rango personalizado
    - Robot va recto ATRÁS a cualquier velocidad en su rango personalizado
    - Giros son simétricos gracias a calibración bidireccional por rueda
    - Todos los robots tienen comportamiento normalizado dentro de sus rangos
"""
import sys
import logging
import argparse
from pathlib import Path

# Configurar logging
logging.basicConfig(
    level=logging.INFO,
    format='%(levelname)s [%(name)s]: %(message)s'
)

# Control de niveles por módulo
logging.getLogger('robot_soccer.core').setLevel(logging.ERROR)
logging.getLogger('robot_soccer.perception').setLevel(logging.ERROR)
logging.getLogger('robot_soccer.entities').setLevel(logging.ERROR)
logging.getLogger('robot_soccer.ai').setLevel(logging.ERROR)
logging.getLogger('robot_soccer.utils').setLevel(logging.ERROR)
logging.getLogger('robot_soccer.communication.rf_controller').setLevel(logging.INFO)
logging.getLogger('robot_soccer.communication.serial_manager').setLevel(logging.INFO)

# Agregar scripts al path
sys.path.insert(0, str(Path(__file__).parent))

# Importar módulo de calibración multiprocess
# pylint: disable=import-error,wrong-import-position
from multiprocess_calibration.calibrate_motors_multipoint_mp import run_motors_calibration_multipoint


def main():
    """Función principal."""
    parser = argparse.ArgumentParser(
        description='Calibración multi-punto de motores (5 puntos + dead-zone)'
    )
    parser.add_argument(
        '--robot-id',
        type=int,
        default=0,
        choices=[0, 1, 2, 3],
        help='ID del robot a calibrar (0-3)'
    )
    parser.add_argument(
        '--camera-id',
        type=int,
        default=None,
        help='ID de la cámara (None = auto-detectar DroidCam)'
    )
    parser.add_argument(
        '--serial-port',
        type=str,
        default='/dev/ttyUSB0',
        help='Puerto serial del transmisor RF'
    )

    args = parser.parse_args()

    print("\n" + "=" * 70)
    print("CALIBRACIÓN BIDIRECCIONAL - COMPENSACIÓN DE ASIMETRÍA")
    print("=" * 70)
    print("\nCaracterísticas:")
    print("  ✅ 10 puntos de calibración (5 ADELANTE + 5 ATRÁS)")
    print("  ✅ Calibración bidireccional por rueda")
    print("  ✅ Adelante: +20, +35, +50, +65, +80 PWM")
    print("  ✅ Atrás:    -20, -35, -50, -65, -80 PWM")
    print("  ✅ Interpolación lineal independiente por dirección")
    print("  ✅ Dead-zone individual por motor (ajustable)")
    print("  ✅ Modo de prueba para encontrar dead-zone exacto")
    print("\nVentajas vs calibración unidireccional:")
    print("  → Robot va recto ADELANTE y ATRÁS a cualquier velocidad")
    print("  → Compensa asimetría inherente de motores DC")
    print("  → Giros simétricos (cada rueda usa su calibración)")
    print("  → Al girar: rueda adelante usa cal. ADELANTE,")
    print("              rueda atrás usa cal. ATRÁS")
    print("  → Usado en RoboCup SSL profesional")
    print("\nModo de prueba de dead-zone:")
    print("  → Presiona 't' para activar/desactivar")
    print("  → Control directo de cada motor (7/8 izq, 9/0 der)")
    print("  → Encuentra el PWM mínimo de movimiento")
    print("\n" + "=" * 70 + "\n")

    # Ejecutar calibración multiprocess
    run_motors_calibration_multipoint(
        robot_id=args.robot_id,
        serial_port=args.serial_port,
        camera_id=args.camera_id
    )


if __name__ == "__main__":
    main()
