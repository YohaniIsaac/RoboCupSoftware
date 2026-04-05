#!/usr/bin/env python3
"""Control manual simple del robot via RF.

Permite controlar motores, dribbler y kicker de forma interactiva
sin necesidad de cámara ni comportamiento autónomo.

Uso:
    python scripts/robot_manual_control.py --robot-id 0
    python scripts/robot_manual_control.py --robot-id 1 --port /dev/ttyUSB0

Controles:
    MOTORES:
    ↑ / W      : Adelante
    ↓ / S      : Atrás
    ← / A      : Giro izquierda (en sitio)
    → / D      : Giro derecha (en sitio)
    X          : Detener motores

    PWM DE CONDUCCIÓN:
    + / =      : PWM +1
    -          : PWM -1

    DRIBBLER:
    F          : Activar / Desactivar
    Q / E      : Dribbler PWM -1 / +1

    KICKER (solenoide):
    K / ESPACIO : Disparar
    1-9         : Potencia del disparo (1=10% … 9=90%)

    ESC         : Salir
"""

import sys
import time
import logging
import argparse
from pathlib import Path

ROOT_DIR = Path(__file__).parent.parent
sys.path.insert(0, str(ROOT_DIR / "src"))

import cv2
import numpy as np
from robot_soccer.communication.rf_controller import RFController

logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s %(levelname)-8s %(message)s',
    datefmt='%H:%M:%S'
)
log = logging.getLogger(__name__)

KEEPALIVE_INTERVAL = 0.08  # 80 ms — firmware apaga motores/dribbler si no recibe en 100 ms


# ---------------------------------------------------------------------------
# Panel de visualización
# ---------------------------------------------------------------------------

def _color_active(active):
    return (0, 220, 80) if active else (60, 60, 60)


def draw_panel(drive_pwm, direction, dribbler_on, dribbler_pwm, kick_power, last_event, robot_id):
    w, h = 510, 395
    img = np.zeros((h, w, 3), dtype=np.uint8)
    img[:] = (30, 30, 30)

    # Título
    cv2.putText(img, f"CONTROL MANUAL  -  Robot {robot_id}",
                (15, 33), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (200, 200, 200), 2)
    cv2.line(img, (0, 46), (w, 46), (80, 80, 80), 1)

    # Motores
    dir_labels = {
        'adelante':   'ADELANTE',
        'atras':      'ATRAS',
        'izquierda':  'GIRO IZQ',
        'derecha':    'GIRO DER',
    }
    dir_label = dir_labels.get(direction, 'DETENIDO')
    m_color = _color_active(direction is not None)
    cv2.putText(img, f"MOTORES:   {dir_label}   PWM {drive_pwm}",
                (15, 78), cv2.FONT_HERSHEY_SIMPLEX, 0.62, m_color, 2)

    # Dribbler
    d_color = _color_active(dribbler_on)
    d_label = 'ON' if dribbler_on else 'OFF'
    cv2.putText(img, f"DRIBBLER:  {d_label}   PWM {dribbler_pwm}",
                (15, 118), cv2.FONT_HERSHEY_SIMPLEX, 0.62, d_color, 2)

    # Kicker
    cv2.putText(img, f"KICKER:    potencia {int(kick_power * 100):3d}%",
                (15, 158), cv2.FONT_HERSHEY_SIMPLEX, 0.62, (0, 150, 255), 2)
    bar_x, bar_y, bar_w, bar_h = 15, 170, 480, 16
    cv2.rectangle(img, (bar_x, bar_y), (bar_x + bar_w, bar_y + bar_h), (55, 55, 55), -1)
    cv2.rectangle(img, (bar_x, bar_y), (bar_x + int(kick_power * bar_w), bar_y + bar_h),
                  (0, 150, 255), -1)

    # Último evento
    cv2.line(img, (0, 200), (w, 200), (60, 60, 60), 1)
    cv2.putText(img, last_event, (15, 228),
                cv2.FONT_HERSHEY_SIMPLEX, 0.52, (255, 220, 80), 1)

    # Controles
    cv2.line(img, (0, 248), (w, 248), (60, 60, 60), 1)
    controls = [
        "↑/W adelante   ↓/S atras   ←/A giro izq   →/D giro der   X stop",
        "+ / -  PWM motores ±1      F = dribbler ON/OFF",
        "Q / E = dribbler PWM -/+   1-9 = potencia kicker",
        "ESC = salir",
    ]
    for i, line in enumerate(controls):
        cv2.putText(img, line, (15, 272 + i * 22),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.38, (130, 130, 130), 1)

    return img


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description='Control manual simple del robot via RF')
    parser.add_argument('--robot-id', type=int, choices=[0, 1, 2, 3], required=True,
                        help='ID del robot (0-3)')
    parser.add_argument('--port', type=str, default='/dev/ttyUSB0',
                        help='Puerto serial RF (default: /dev/ttyUSB0)')
    args = parser.parse_args()

    robot_id = args.robot_id
    firmware_id = robot_id + 1  # RF usa IDs 1-based

    log.info("Conectando a %s...", args.port)
    rf = RFController(port=args.port, enable_calibration=True)
    if not rf.initialize():
        log.error("No se pudo inicializar RF en %s", args.port)
        sys.exit(1)

    connections = rf.test_connections()
    if not connections.get(f'robot_{firmware_id}', False):
        log.warning("Robot %d no responde via RF — verificar que esté encendido", robot_id)
    else:
        log.info("Robot %d disponible via RF", robot_id)

    # Estado
    drive_pwm = 30
    direction = None       # None | 'adelante' | 'atras' | 'izquierda' | 'derecha'
    dribbler_on = False
    dribbler_pwm = 200
    kick_power = 0.5
    last_event = "Esperando comando..."
    last_keepalive = 0.0

    window_name = f"Control Manual  -  Robot {robot_id}"
    cv2.namedWindow(window_name, cv2.WINDOW_AUTOSIZE)

    log.info("Ventana abierta. Usa WASD/flechas=motores, F=dribbler, K=kicker, ESC=salir")

    try:
        while True:
            now = time.time()

            # Keepalive: reenviar comando activo cada 80 ms
            if now - last_keepalive >= KEEPALIVE_INTERVAL:
                if direction == 'adelante':
                    rf.set_motors(firmware_id, drive_pwm, drive_pwm)
                elif direction == 'atras':
                    rf.set_motors(firmware_id, -drive_pwm, -drive_pwm)
                elif direction == 'izquierda':
                    rf.set_motors(firmware_id, -drive_pwm, drive_pwm)
                elif direction == 'derecha':
                    rf.set_motors(firmware_id, drive_pwm, -drive_pwm)
                if dribbler_on:
                    rf.set_dribbler(firmware_id, dribbler_pwm)
                last_keepalive = now

            img = draw_panel(drive_pwm, direction, dribbler_on, dribbler_pwm,
                             kick_power, last_event, robot_id)
            cv2.imshow(window_name, img)
            key = cv2.waitKey(30) & 0xFF

            if key == 255:  # Sin tecla
                continue

            # --- Salir ---
            if key == 27:
                break

            # --- Motores ---
            elif key in (82, ord('w'), ord('W')):       # ↑ / W
                direction = 'adelante'
                rf.set_motors(firmware_id, drive_pwm, drive_pwm)
                last_event = f"Adelante  PWM {drive_pwm}"
                last_keepalive = now

            elif key in (84, ord('s'), ord('S')):       # ↓ / S
                direction = 'atras'
                rf.set_motors(firmware_id, -drive_pwm, -drive_pwm)
                last_event = f"Atrás  PWM {drive_pwm}"
                last_keepalive = now

            elif key in (81, ord('a'), ord('A')):       # ← / A
                direction = 'izquierda'
                rf.set_motors(firmware_id, -drive_pwm, drive_pwm)
                last_event = f"Giro izquierda  PWM {drive_pwm}"
                last_keepalive = now

            elif key in (83, ord('d'), ord('D')):       # → / D
                direction = 'derecha'
                rf.set_motors(firmware_id, drive_pwm, -drive_pwm)
                last_event = f"Giro derecha  PWM {drive_pwm}"
                last_keepalive = now

            elif key in (ord('x'), ord('X')):
                direction = None
                rf.stop_robot(firmware_id)
                last_event = "Motores detenidos"

            # --- PWM motores ---
            elif key in (ord('+'), ord('=')):
                drive_pwm = min(127, drive_pwm + 1)
                last_event = f"PWM motores → {drive_pwm}"

            elif key == ord('-'):
                drive_pwm = max(1, drive_pwm - 1)
                last_event = f"PWM motores → {drive_pwm}"

            # --- Dribbler ---
            elif key in (ord('f'), ord('F')):
                dribbler_on = not dribbler_on
                if dribbler_on:
                    rf.set_dribbler(firmware_id, dribbler_pwm)
                    last_event = f"Dribbler ON  PWM {dribbler_pwm}"
                else:
                    rf.set_dribbler(firmware_id, 0)
                    last_event = "Dribbler OFF"
                last_keepalive = now

            elif key in (ord('q'), ord('Q')):
                dribbler_pwm = max(0, dribbler_pwm - 1)
                last_event = f"Dribbler PWM → {dribbler_pwm}"
                if dribbler_on:
                    rf.set_dribbler(firmware_id, dribbler_pwm)
                    last_keepalive = now

            elif key in (ord('e'), ord('E')):
                dribbler_pwm = min(255, dribbler_pwm + 1)
                last_event = f"Dribbler PWM → {dribbler_pwm}"
                if dribbler_on:
                    rf.set_dribbler(firmware_id, dribbler_pwm)
                    last_keepalive = now

            # --- Kicker ---
            elif key in (ord('k'), ord('K'), ord(' ')):
                rf.kick(firmware_id, kick_power)
                last_event = f"DISPARO  potencia {int(kick_power * 100)}%"
                log.info("Kick  potencia=%.0f%%", kick_power * 100)

            elif ord('1') <= key <= ord('9'):
                kick_power = (key - ord('0')) / 10.0
                last_event = f"Potencia kicker → {int(kick_power * 100)}%"

    finally:
        if dribbler_on:
            rf.set_dribbler(firmware_id, 0)
        rf.stop_robot(firmware_id)
        rf.shutdown()
        cv2.destroyAllWindows()
        log.info("Robot detenido. Fin del control manual.")


if __name__ == '__main__':
    main()
