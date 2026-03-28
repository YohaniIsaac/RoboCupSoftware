#!/usr/bin/env python3
"""Test simple: dribbler y kicker via RF.

Permite activar/desactivar el dribbler y disparar el kicker
de forma interactiva para verificar que ambos mecanismos funcionan.

Uso:
    python scripts/integration/test_dribbler_kicker.py --robot-id 0
    python scripts/integration/test_dribbler_kicker.py --robot-id 0 --serial-port /dev/ttyUSB0

Controles (en la ventana):
    D        : Activar / Desactivar dribbler
    K        : Disparar (kicker) a la potencia actual
    ESPACIO  : Disparar (igual que K)
    1..9     : Cambiar potencia (1=10%, 5=50%, 9=90%, etc.)
    +  / =   : Subir potencia +10%
    -        : Bajar potencia -10%
    ESC      : Salir
"""

import sys
import time
import logging
import argparse
from pathlib import Path

ROOT_DIR = Path(__file__).parent.parent.parent
sys.path.insert(0, str(ROOT_DIR / "src"))
sys.path.insert(0, str(ROOT_DIR / "scripts"))

import cv2
import numpy as np
from robot_soccer.communication.rf_controller import RFController
from robot_soccer.utils.camera_utils import get_camera_index

logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s %(levelname)-8s %(message)s',
    datefmt='%H:%M:%S'
)
log = logging.getLogger(__name__)


def draw_panel(dribbler_on, dribbler_power, kick_power, last_event, robot_id):
    """Dibuja el panel de estado."""
    w, h = 480, 340
    img = np.zeros((h, w, 3), dtype=np.uint8)
    img[:] = (30, 30, 30)

    # Título
    cv2.putText(img, f"TEST DRIBBLER / KICKER  -  Robot {robot_id}",
                (15, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (200, 200, 200), 2)
    cv2.line(img, (0, 50), (w, 50), (80, 80, 80), 1)

    # Estado dribbler
    d_color = (0, 255, 80) if dribbler_on else (60, 60, 60)
    d_label = "ON " if dribbler_on else "OFF"
    cv2.rectangle(img, (20, 70), (220, 140), d_color, -1 if dribbler_on else 2)
    cv2.putText(img, "DRIBBLER", (35, 98),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0) if dribbler_on else (150, 150, 150), 2)
    cv2.putText(img, d_label, (90, 128),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0) if dribbler_on else (150, 150, 150), 2)
    cv2.putText(img, f"{int(dribbler_power * 100)}%", (148, 128),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0) if dribbler_on else (120, 120, 120), 1)

    # Estado kicker
    k_color = (0, 140, 255)
    cv2.rectangle(img, (250, 70), (460, 140), k_color, 2)
    cv2.putText(img, "KICKER", (285, 98),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, k_color, 2)
    cv2.putText(img, f"Potencia: {int(kick_power * 100)}%", (265, 128),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, k_color, 1)

    # Barra de potencia (compartida: aplica a kicker)
    bar_x, bar_y = 20, 165
    bar_w = 440
    bar_h = 22
    cv2.rectangle(img, (bar_x, bar_y), (bar_x + bar_w, bar_y + bar_h), (80, 80, 80), -1)
    fill = int(kick_power * bar_w)
    cv2.rectangle(img, (bar_x, bar_y), (bar_x + fill, bar_y + bar_h), (0, 180, 255), -1)
    cv2.rectangle(img, (bar_x, bar_y), (bar_x + bar_w, bar_y + bar_h), (120, 120, 120), 1)
    cv2.putText(img, f"Potencia kicker: {int(kick_power * 100)}%",
                (bar_x + 5, bar_y + 16), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)

    # Último evento
    cv2.line(img, (0, 205), (w, 205), (60, 60, 60), 1)
    cv2.putText(img, "Ultimo evento:", (15, 228),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (150, 150, 150), 1)
    cv2.putText(img, last_event, (15, 252),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 220, 80), 1)

    # Controles
    cv2.line(img, (0, 270), (w, 270), (60, 60, 60), 1)
    controls = [
        "D = Dribbler ON/OFF   K / ESPACIO = Disparar",
        "1-9 = Potencia kicker  +/- = +/-10%",
        "ESC = Salir",
    ]
    for i, line in enumerate(controls):
        cv2.putText(img, line, (15, 292 + i * 18),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.38, (130, 130, 130), 1)

    return img


def main():
    parser = argparse.ArgumentParser(description='Test dribbler y kicker via RF')
    parser.add_argument('--robot-id', type=int, choices=[0, 1, 2, 3],
                        required=True, help='ID del robot (0-3)')
    parser.add_argument('--serial-port', type=str, default='/dev/ttyUSB0',
                        help='Puerto serial (default: /dev/ttyUSB0)')
    args = parser.parse_args()

    robot_id = args.robot_id
    firmware_id = robot_id + 1  # RF usa IDs 1-based

    # Conectar RF
    log.info("Conectando a %s...", args.serial_port)
    rf = RFController(port=args.serial_port, enable_calibration=True)
    if not rf.initialize():
        log.error("No se pudo inicializar el controlador RF en %s", args.serial_port)
        sys.exit(1)

    connections = rf.test_connections()
    robot_key = f'robot_{firmware_id}'
    if not connections.get(robot_key, False):
        log.warning("Robot %d no responde via RF — verificar que esté encendido", robot_id)
    else:
        log.info("Robot %d disponible via RF", robot_id)

    # Estado
    dribbler_on = False
    dribbler_power = 1.0   # 100%
    kick_power = 0.5       # 50%
    last_event = "Esperando comando..."

    window_name = f"Dribbler / Kicker Test  -  Robot {robot_id}"
    cv2.namedWindow(window_name, cv2.WINDOW_AUTOSIZE)

    log.info("Ventana abierta. Usa D=dribbler, K/ESPACIO=kicker, 1-9=potencia, ESC=salir")

    last_dribbler_keepalive = 0.0
    DRIBBLER_KEEPALIVE = 0.08  # 80ms < timeout firmware 100ms

    try:
        while True:
            now = time.time()

            # Keepalive: reenviar 'D' cada 80ms cuando dribbler está activo.
            # El firmware apaga el dribbler si no recibe señal en 100ms
            # (mismo mecanismo que las ruedas). Sin esto, el dribbler se apagaría solo.
            if dribbler_on and now - last_dribbler_keepalive >= DRIBBLER_KEEPALIVE:
                rf.set_dribbler(firmware_id, dribbler_power)
                last_dribbler_keepalive = now

            img = draw_panel(dribbler_on, dribbler_power, kick_power, last_event, robot_id)
            cv2.imshow(window_name, img)

            key = cv2.waitKey(30) & 0xFF  # 33fps — menor que 80ms para no perderse keepalive

            if key == 27:  # ESC
                log.info("Saliendo...")
                break

            elif key == ord('d') or key == ord('D'):
                dribbler_on = not dribbler_on
                if dribbler_on:
                    ok = rf.set_dribbler(firmware_id, dribbler_power)
                    last_event = f"Dribbler ACTIVADO ({int(dribbler_power*100)}%)" + (" OK" if ok else " FALLO")
                    log.info("Dribbler ON  potencia=%.0f%%  %s", dribbler_power * 100, "OK" if ok else "FALLO")
                else:
                    ok = rf.set_dribbler(firmware_id, 0.0)
                    last_event = "Dribbler DESACTIVADO" + (" OK" if ok else " FALLO")
                    log.info("Dribbler OFF  %s", "OK" if ok else "FALLO")

            elif key == ord('k') or key == ord('K') or key == ord(' '):
                ok = rf.kick(firmware_id, kick_power)
                last_event = f"DISPARO  potencia={int(kick_power*100)}%" + (" OK" if ok else " FALLO")
                log.info("Kick  potencia=%.0f%%  %s", kick_power * 100, "OK" if ok else "FALLO")

            elif ord('1') <= key <= ord('9'):
                kick_power = (key - ord('0')) / 10.0
                last_event = f"Potencia kicker → {int(kick_power*100)}%"
                log.info("Potencia kicker: %.0f%%", kick_power * 100)

            elif key in (ord('+'), ord('=')):
                kick_power = min(1.0, kick_power + 0.1)
                last_event = f"Potencia kicker → {int(kick_power*100)}%"
                log.info("Potencia kicker: %.0f%%", kick_power * 100)

            elif key == ord('-'):
                kick_power = max(0.1, kick_power - 0.1)
                last_event = f"Potencia kicker → {int(kick_power*100)}%"
                log.info("Potencia kicker: %.0f%%", kick_power * 100)

    finally:
        # Apagar dribbler al salir
        if dribbler_on:
            rf.set_dribbler(firmware_id, 0.0)
            log.info("Dribbler apagado al salir")
        rf.stop_robot(firmware_id)
        rf.shutdown()
        cv2.destroyAllWindows()
        log.info("Fin del test")


if __name__ == '__main__':
    main()
