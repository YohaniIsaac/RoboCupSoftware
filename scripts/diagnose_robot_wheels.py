#!/usr/bin/env python3
"""Diagnóstico de ruedas: enviar PWM independiente a cada motor.

Herramienta para diagnosticar asimetría entre la rueda izquierda y la derecha
de un robot (p. ej. el Robot 3, que muestra una diferencia de fuerza grande).

A diferencia de calibrate_robot_pwm_range.py — que SIEMPRE envía el mismo PWM a
ambas ruedas — aquí cada rueda se controla por separado, lo que permite:

  - Aislar una rueda (la otra en 0) y ver si gira y cuánto.
  - Encontrar el PWM de arranque (dead-zone) de CADA motor por separado.
  - Comparar ambas ruedas al MISMO PWM para cuantificar la asimetría.

IMPORTANTE: se envía con enable_calibration=False, o sea PWM CRUDO. El firmware
recibe exactamente los valores que ves en pantalla: ni bias ni dead-zone los
modifican. Así la asimetría que observas es física (motor/rueda/fricción), no
enmascarada por software. El bias guardado se muestra solo como referencia.

Uso:
    python scripts/diagnose_robot_wheels.py --robot-id 3
    python scripts/diagnose_robot_wheels.py --robot-id 3 --port /dev/ttyUSB0

Controles:
    RUEDA IZQUIERDA:
    q / a       : Left PWM +1 / -1
    RUEDA DERECHA:
    o / l       : Right PWM +1 / -1
    AMBAS:
    w / s       : Ambas +1 / -1 (mantiene la diferencia actual)
    ↑ / ↓       : Ambas +5 / -5

    PRESETS DE DIAGNÓSTICO (usan el PWM 'base'):
    1           : Solo IZQUIERDA  (L=base, R=0)
    2           : Solo DERECHA    (L=0,   R=base)
    3           : AMBAS iguales   (L=R=base)  -> ¿va recto?
    + / -       : base +/- 1
    z           : invertir sentido (negar L, R y base)

    OTROS:
    x / ESPACIO : STOP (0, 0)
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
from robot_soccer.controllers.robot_calibration_multipoint import RobotCalibrationMultipoint

logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s %(levelname)-8s %(message)s',
    datefmt='%H:%M:%S'
)
log = logging.getLogger(__name__)

KEEPALIVE_INTERVAL = 0.08  # 80 ms — firmware apaga motores si no recibe en 100 ms
PWM_LIMIT = 127            # rango int8_t del firmware


# ---------------------------------------------------------------------------
# Panel de visualización
# ---------------------------------------------------------------------------

def _draw_wheel_bar(img, y, label, pwm):
    """Dibuja una barra bidireccional [-127, 127] centrada para una rueda."""
    cx, half = 270, 220  # centro x y semiancho de la barra
    bar_y, bar_h = y, 20
    # Pista
    cv2.rectangle(img, (cx - half, bar_y), (cx + half, bar_y + bar_h), (55, 55, 55), -1)
    cv2.line(img, (cx, bar_y - 3), (cx, bar_y + bar_h + 3), (110, 110, 110), 1)
    # Relleno proporcional al PWM
    fill = int((pwm / PWM_LIMIT) * half)
    color = (0, 220, 80) if pwm >= 0 else (60, 120, 255)
    if fill >= 0:
        cv2.rectangle(img, (cx, bar_y), (cx + fill, bar_y + bar_h), color, -1)
    else:
        cv2.rectangle(img, (cx + fill, bar_y), (cx, bar_y + bar_h), color, -1)
    cv2.putText(img, f"{label}", (15, bar_y + 16),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (210, 210, 210), 1)
    cv2.putText(img, f"{pwm:+4d}", (cx + half + 8, bar_y + 16),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2)


def draw_panel(left_pwm, right_pwm, base, bias_ref, last_event, robot_id, firmware_id):
    w, h = 560, 470
    img = np.zeros((h, w, 3), dtype=np.uint8)
    img[:] = (30, 30, 30)

    cv2.putText(img, f"DIAGNOSTICO DE RUEDAS  -  Robot {robot_id} (firmware {firmware_id})",
                (15, 32), cv2.FONT_HERSHEY_SIMPLEX, 0.58, (200, 200, 200), 2)
    cv2.line(img, (0, 46), (w, 46), (80, 80, 80), 1)

    _draw_wheel_bar(img, 64, "IZQUIERDA", left_pwm)
    _draw_wheel_bar(img, 104, "DERECHA", right_pwm)

    # Diferencia L-R: la magnitud de la asimetría que se está enviando
    diff = left_pwm - right_pwm
    diff_color = (0, 220, 80) if diff == 0 else (80, 200, 255)
    cv2.putText(img, f"DIFERENCIA L-R: {diff:+d}        base={base:+d}",
                (15, 158), cv2.FONT_HERSHEY_SIMPLEX, 0.55, diff_color, 2)

    # Referencia: calibración OFF, bias guardado solo informativo
    cv2.putText(img, f"calibracion=OFF (PWM crudo)   bias guardado={bias_ref:+.3f} (NO aplicado)",
                (15, 186), cv2.FONT_HERSHEY_SIMPLEX, 0.44, (150, 150, 150), 1)

    cv2.line(img, (0, 202), (w, 202), (60, 60, 60), 1)
    cv2.putText(img, last_event, (15, 226),
                cv2.FONT_HERSHEY_SIMPLEX, 0.50, (255, 220, 80), 1)

    cv2.line(img, (0, 244), (w, 244), (60, 60, 60), 1)
    controls = [
        "RUEDA IZQ:  q/a = +/-1        RUEDA DER:  o/l = +/-1",
        "AMBAS:      w/s = +/-1        flechas U/D = +/-5",
        "",
        "PRESETS (usan 'base'):",
        "  1 = solo IZQ (L=base,R=0)   2 = solo DER (L=0,R=base)",
        "  3 = AMBAS iguales (recto)   +/- = base +/-1   z = invertir",
        "",
        "x / ESPACIO = STOP            ESC = salir",
    ]
    for i, line in enumerate(controls):
        cv2.putText(img, line, (15, 268 + i * 24),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.44, (130, 130, 130), 1)

    return img


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(
        description='Diagnostico de ruedas: PWM independiente por motor'
    )
    parser.add_argument('--robot-id', type=int, choices=[0, 1, 2, 3], default=3,
                        help='ID del robot (0-3). Default: 3')
    parser.add_argument('--port', type=str, default='/dev/ttyUSB0',
                        help='Puerto serial RF (default: /dev/ttyUSB0)')
    args = parser.parse_args()

    robot_id = args.robot_id
    firmware_id = robot_id + 1  # RF usa IDs 1-based

    # Bias guardado: solo para mostrarlo como referencia (NO se aplica)
    bias_ref = RobotCalibrationMultipoint().get_bias(robot_id)

    log.info("Conectando a %s...", args.port)
    # enable_calibration=False -> PWM crudo, sin bias ni dead-zone: queremos ver
    # la respuesta fisica real de cada motor para diagnosticar la asimetria.
    rf = RFController(port=args.port, enable_calibration=False, min_command_interval=0.005)
    if not rf.initialize():
        log.error("No se pudo inicializar RF en %s", args.port)
        sys.exit(1)

    connections = rf.test_connections()
    if not connections.get(f'robot_{firmware_id}', False):
        log.warning("Robot %d no responde via RF — verificar que este encendido", robot_id)
    else:
        log.info("Robot %d disponible via RF", robot_id)

    log.info("Bias guardado del Robot %d: %+.3f (referencia, NO se aplica)", robot_id, bias_ref)

    # Estado
    left_pwm = 0
    right_pwm = 0
    base = 25                       # PWM de referencia para los presets 1/2/3
    last_event = "Listo. Usa los presets 1/2/3 o ajusta cada rueda."
    last_keepalive = 0.0

    def clamp(v):
        return max(-PWM_LIMIT, min(PWM_LIMIT, int(v)))

    window_name = f"Diagnostico de ruedas  -  Robot {robot_id}"
    cv2.namedWindow(window_name, cv2.WINDOW_AUTOSIZE)
    log.info("Ventana abierta. q/a=IZQ, o/l=DER, w/s=ambas, 1/2/3=presets, x=stop, ESC=salir")

    try:
        while True:
            now = time.time()

            # Keepalive: reenviar el comando actual mientras alguna rueda gira
            if now - last_keepalive >= KEEPALIVE_INTERVAL:
                if left_pwm != 0 or right_pwm != 0:
                    rf.set_motors(firmware_id, left_pwm, right_pwm)
                last_keepalive = now

            img = draw_panel(left_pwm, right_pwm, base, bias_ref,
                             last_event, robot_id, firmware_id)
            cv2.imshow(window_name, img)
            key = cv2.waitKey(30) & 0xFF

            if key == 255:  # Sin tecla
                continue

            send_now = False

            # --- Salir ---
            if key == 27:
                break

            # --- STOP ---
            elif key in (ord('x'), ord('X'), ord(' ')):
                left_pwm = 0
                right_pwm = 0
                rf.set_motors(firmware_id, 0, 0)
                last_event = "STOP (0, 0)"

            # --- Rueda izquierda ---
            elif key in (ord('q'), ord('Q')):
                left_pwm = clamp(left_pwm + 1)
                last_event = f"Izquierda -> {left_pwm:+d}"
                send_now = True
            elif key in (ord('a'), ord('A')):
                left_pwm = clamp(left_pwm - 1)
                last_event = f"Izquierda -> {left_pwm:+d}"
                send_now = True

            # --- Rueda derecha ---
            elif key in (ord('o'), ord('O')):
                right_pwm = clamp(right_pwm + 1)
                last_event = f"Derecha -> {right_pwm:+d}"
                send_now = True
            elif key in (ord('l'), ord('L')):
                right_pwm = clamp(right_pwm - 1)
                last_event = f"Derecha -> {right_pwm:+d}"
                send_now = True

            # --- Ambas ±1 ---
            elif key in (ord('w'), ord('W')):
                left_pwm = clamp(left_pwm + 1)
                right_pwm = clamp(right_pwm + 1)
                last_event = f"Ambas +1 -> L{left_pwm:+d} R{right_pwm:+d}"
                send_now = True
            elif key in (ord('s'), ord('S')):
                left_pwm = clamp(left_pwm - 1)
                right_pwm = clamp(right_pwm - 1)
                last_event = f"Ambas -1 -> L{left_pwm:+d} R{right_pwm:+d}"
                send_now = True

            # --- Ambas ±5 (flechas ↑/↓) ---
            elif key == 82:  # ↑
                left_pwm = clamp(left_pwm + 5)
                right_pwm = clamp(right_pwm + 5)
                last_event = f"Ambas +5 -> L{left_pwm:+d} R{right_pwm:+d}"
                send_now = True
            elif key == 84:  # ↓
                left_pwm = clamp(left_pwm - 5)
                right_pwm = clamp(right_pwm - 5)
                last_event = f"Ambas -5 -> L{left_pwm:+d} R{right_pwm:+d}"
                send_now = True

            # --- Presets de diagnóstico ---
            elif key == ord('1'):
                left_pwm, right_pwm = base, 0
                last_event = f"Solo IZQUIERDA  L{left_pwm:+d} R0"
                send_now = True
            elif key == ord('2'):
                left_pwm, right_pwm = 0, base
                last_event = f"Solo DERECHA  L0 R{right_pwm:+d}"
                send_now = True
            elif key == ord('3'):
                left_pwm, right_pwm = base, base
                last_event = f"AMBAS iguales  L{left_pwm:+d} R{right_pwm:+d} (mismo PWM)"
                send_now = True

            # --- Ajustar base ---
            elif key in (ord('+'), ord('=')):
                base = clamp(base + 1)
                last_event = f"base -> {base:+d}"
            elif key == ord('-'):
                base = clamp(base - 1)
                last_event = f"base -> {base:+d}"

            # --- Invertir sentido ---
            elif key in (ord('z'), ord('Z')):
                left_pwm = clamp(-left_pwm)
                right_pwm = clamp(-right_pwm)
                base = clamp(-base)
                last_event = f"Invertido -> L{left_pwm:+d} R{right_pwm:+d} base{base:+d}"
                send_now = True

            if send_now:
                rf.set_motors(firmware_id, left_pwm, right_pwm)
                last_keepalive = now

    finally:
        rf.set_motors(firmware_id, 0, 0)
        rf.stop_robot(firmware_id)
        rf.shutdown()
        cv2.destroyAllWindows()
        log.info("Robot detenido. Fin del diagnostico de ruedas.")


if __name__ == '__main__':
    main()
