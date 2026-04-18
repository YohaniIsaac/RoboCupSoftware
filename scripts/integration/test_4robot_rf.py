#!/usr/bin/env python3
"""Test RF simultáneo para 4 robots: valida comandos de movimiento y mide latencia serial.

Pre-requisito del PASO 7 (partido 2v2). Verifica que:
  - Los 4 robots responden via RF desde un único puerto serial
  - El rate limiter no bloquea comandos cuando se envían a 4 robots en round-robin
  - Los comandos de movimiento llegan con latencia aceptable a cada robot

Uso:
    python scripts/integration/test_4robot_rf.py
    python scripts/integration/test_4robot_rf.py --serial-port /dev/ttyUSB0
    python scripts/integration/test_4robot_rf.py --robots 0 1 2 3

Controles (ventana OpenCV):
    1 / 2 / 3 / 4  : Activar/detener robot individual (Python IDs 0-3)
    A               : Mover todos los robots activos simultáneamente
    S               : Detener todos (stop de emergencia)
    F               : Ciclar patrón de movimiento (adelante / rotar-izq / rotar-der / atrás)
    ESC             : Salir
"""

import sys
import time
import logging
import argparse
import collections
from pathlib import Path

ROOT_DIR = Path(__file__).parent.parent.parent
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

# Patrones de movimiento: (left_pwm, right_pwm)
PATTERNS = [
    ("Adelante",   50,  50),
    ("Rotar Izq", -40,  40),
    ("Rotar Der",  40, -40),
    ("Atras",     -50, -50),
]

# Colores por equipo (BGR)
TEAM_COLORS = {
    1: (80,  80, 220),   # Robot 0 (firmware 1) — rojo
    2: (60, 100, 200),   # Robot 1 (firmware 2) — rojo claro
    3: (220, 80,  80),   # Robot 2 (firmware 3) — azul
    4: (200, 100, 60),   # Robot 3 (firmware 4) — azul claro
}


def draw_panel(robot_states, pattern_idx, total_cmds_s, window_w=640, window_h=420):
    img = np.zeros((window_h, window_w, 3), dtype=np.uint8)
    img[:] = (25, 25, 25)

    # Título
    cv2.putText(img, "TEST RF - 4 ROBOTS",
                (window_w // 2 - 120, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (200, 200, 200), 2)
    pat_name, pat_l, pat_r = PATTERNS[pattern_idx]
    cv2.putText(img, f"Patron: {pat_name}  L={pat_l:+d}  R={pat_r:+d}",
                (window_w // 2 - 140, 55), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (160, 160, 160), 1)
    cv2.line(img, (0, 65), (window_w, 65), (60, 60, 60), 1)

    # Cajas por robot
    box_w = (window_w - 40) // 4
    box_h = 200
    box_y = 80

    for i, (fid, state) in enumerate(sorted(robot_states.items())):
        bx = 10 + i * (box_w + 6)
        color = TEAM_COLORS[fid]
        moving = state['moving']
        connected = state['connected']

        # Fondo
        fill_color = (int(color[0] * 0.35), int(color[1] * 0.35), int(color[2] * 0.35))
        cv2.rectangle(img, (bx, box_y), (bx + box_w, box_y + box_h), fill_color, -1)
        border_color = color if moving else (80, 80, 80)
        cv2.rectangle(img, (bx, box_y), (bx + box_w, box_y + box_h), border_color,
                      3 if moving else 1)

        # ID y equipo
        team = "ROJO" if fid <= 2 else "AZUL"
        python_id = fid - 1
        cv2.putText(img, f"R{python_id}  [{team}]",
                    (bx + 8, box_y + 24), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

        # Estado conexion
        if connected:
            cv2.putText(img, "RF: OK", (bx + 8, box_y + 46),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0, 220, 80), 1)
        else:
            cv2.putText(img, "RF: --", (bx + 8, box_y + 46),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.42, (80, 80, 80), 1)

        # Estado movimiento
        status_label = "MOVIENDO" if moving else "DETENIDO"
        status_color = color if moving else (100, 100, 100)
        cv2.putText(img, status_label,
                    (bx + 8, box_y + 68), cv2.FONT_HERSHEY_SIMPLEX, 0.45, status_color, 1)

        # PWM enviado
        l_pwm, r_pwm = state.get('pwm', (0, 0))
        cv2.putText(img, f"L={l_pwm:+3d}", (bx + 8, box_y + 90),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.42, (180, 180, 180), 1)
        cv2.putText(img, f"R={r_pwm:+3d}", (bx + 8, box_y + 108),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.42, (180, 180, 180), 1)

        # Barra de comandos/s
        cmds_s = state.get('cmds_s', 0.0)
        cv2.putText(img, f"{cmds_s:.1f} cmd/s", (bx + 8, box_y + 130),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.42, (160, 200, 160), 1)

        bar_h_px = 28
        bar_y_px = box_y + 143
        max_rate = 70.0  # comandos/s máximos esperables
        fill_frac = min(cmds_s / max_rate, 1.0)
        cv2.rectangle(img, (bx + 8, bar_y_px), (bx + box_w - 8, bar_y_px + bar_h_px),
                      (50, 50, 50), -1)
        fill_px = int(fill_frac * (box_w - 16))
        bar_fill_color = color if moving else (60, 80, 60)
        cv2.rectangle(img, (bx + 8, bar_y_px), (bx + 8 + fill_px, bar_y_px + bar_h_px),
                      bar_fill_color, -1)

        # Latencia media serial
        avg_lat = state.get('avg_lat_ms', None)
        lat_str = f"lat={avg_lat:.1f}ms" if avg_lat is not None else "lat=---"
        cv2.putText(img, lat_str, (bx + 8, box_y + 190),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (140, 140, 200), 1)

    # Métricas globales
    y_metrics = box_y + box_h + 20
    cv2.line(img, (0, y_metrics - 8), (window_w, y_metrics - 8), (50, 50, 50), 1)
    cv2.putText(img, f"Total comandos/s (todos los robots): {total_cmds_s:.1f}",
                (10, y_metrics + 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)

    # Controles
    y_ctrl = y_metrics + 32
    cv2.line(img, (0, y_ctrl - 4), (window_w, y_ctrl - 4), (40, 40, 40), 1)
    controls = "1/2/3/4=Robot  A=Todos  S=Stop  F=Patron  ESC=Salir"
    cv2.putText(img, controls, (10, y_ctrl + 14),
                cv2.FONT_HERSHEY_SIMPLEX, 0.4, (120, 120, 120), 1)

    return img


def main():
    parser = argparse.ArgumentParser(description='Test RF simultáneo: 4 robots')
    parser.add_argument('--serial-port', default='/dev/ttyUSB0',
                        help='Puerto serial del Arduino transmisor (default: /dev/ttyUSB0)')
    parser.add_argument('--robots', nargs='+', type=int, default=[0, 1, 2, 3],
                        choices=[0, 1, 2, 3],
                        help='IDs Python de los robots a probar (default: 0 1 2 3)')
    args = parser.parse_args()

    firmware_ids = [rid + 1 for rid in args.robots]

    log.info("Conectando a %s...", args.serial_port)
    rf = RFController(port=args.serial_port, enable_calibration=False)
    if not rf.initialize():
        log.error("No se pudo abrir %s — verifica que el Arduino esté conectado", args.serial_port)
        sys.exit(1)
    log.info("Serial OK. Probando conexiones RF (ping)...")

    connections = rf.test_connections()
    robot_states = {}
    for fid in firmware_ids:
        key = f'robot_{fid}'
        connected = connections.get(key, False)
        robot_states[fid] = {
            'connected': connected,
            'moving': False,
            'pwm': (0, 0),
            'cmds_s': 0.0,
            'avg_lat_ms': None,
            '_send_times': collections.deque(maxlen=60),  # ventana 1s a 60Hz
            '_lat_samples': collections.deque(maxlen=20),
        }
        status = "disponible" if connected else "SIN RESPUESTA"
        log.info("  Robot %d (firmware %d): %s", fid - 1, fid, status)

    log.info("")
    log.info("Controles: 1/2/3/4=robot individual, A=todos, S=stop, F=patron, ESC=salir")

    pattern_idx = 0
    window_name = "Test RF 4 Robots"
    cv2.namedWindow(window_name, cv2.WINDOW_AUTOSIZE)

    LOOP_SLEEP = 0.010  # 100 Hz — bien por debajo del rate limit (15ms/robot)

    try:
        while True:
            now = time.time()
            pat_name, pat_l, pat_r = PATTERNS[pattern_idx]

            # Enviar comandos de movimiento a robots activos y medir transmisión real
            total_sent_this_tick = 0
            for fid in firmware_ids:
                state = robot_states[fid]
                if not state['moving']:
                    continue

                prev_send_time = rf.last_send_time.get(fid, 0.0)
                t_call = time.time()
                rf.set_motors(fid, pat_l, pat_r)
                t_after = time.time()

                new_send_time = rf.last_send_time.get(fid, 0.0)
                if new_send_time > prev_send_time:
                    # El rate limiter dejó pasar este comando
                    state['_send_times'].append(now)
                    lat_ms = (t_after - t_call) * 1000.0
                    state['_lat_samples'].append(lat_ms)
                    total_sent_this_tick += 1

                state['pwm'] = (pat_l, pat_r)

            # Actualizar métricas por robot (ventana deslizante de 1s)
            total_cmds_s = 0.0
            for fid in firmware_ids:
                state = robot_states[fid]
                cutoff = now - 1.0
                while state['_send_times'] and state['_send_times'][0] < cutoff:
                    state['_send_times'].popleft()
                state['cmds_s'] = len(state['_send_times'])
                total_cmds_s += state['cmds_s']
                if state['_lat_samples']:
                    state['avg_lat_ms'] = sum(state['_lat_samples']) / len(state['_lat_samples'])

            # Dibujar panel
            panel = draw_panel(robot_states, pattern_idx, total_cmds_s)
            cv2.imshow(window_name, panel)

            key = cv2.waitKey(int(LOOP_SLEEP * 1000)) & 0xFF

            if key == 27:  # ESC
                break

            elif key == ord('s') or key == ord('S'):
                for fid in firmware_ids:
                    rf.stop_robot(fid)
                    robot_states[fid]['moving'] = False
                    robot_states[fid]['pwm'] = (0, 0)
                log.info("STOP: todos los robots detenidos")

            elif key == ord('a') or key == ord('A'):
                # Toggle todos
                any_moving = any(robot_states[fid]['moving'] for fid in firmware_ids)
                target = not any_moving
                for fid in firmware_ids:
                    robot_states[fid]['moving'] = target
                    if not target:
                        rf.stop_robot(fid)
                        robot_states[fid]['pwm'] = (0, 0)
                log.info("TODOS: %s | patron='%s'", "MOVIENDO" if target else "DETENIDOS", pat_name)

            elif key == ord('f') or key == ord('F'):
                pattern_idx = (pattern_idx + 1) % len(PATTERNS)
                new_name = PATTERNS[pattern_idx][0]
                log.info("Patron: %s", new_name)

            elif ord('1') <= key <= ord('4'):
                python_id = key - ord('1')
                fid = python_id + 1
                if fid in robot_states:
                    state = robot_states[fid]
                    state['moving'] = not state['moving']
                    if not state['moving']:
                        rf.stop_robot(fid)
                        state['pwm'] = (0, 0)
                    log.info("Robot %d: %s | patron='%s'",
                             python_id,
                             "MOVIENDO" if state['moving'] else "DETENIDO",
                             pat_name)

            time.sleep(LOOP_SLEEP)

    except KeyboardInterrupt:
        pass
    finally:
        log.info("Deteniendo todos los robots...")
        for fid in firmware_ids:
            rf.stop_robot(fid)
        time.sleep(0.1)
        rf.shutdown()
        cv2.destroyAllWindows()

        # Resumen final
        log.info("")
        log.info("=== RESUMEN ===")
        for fid in firmware_ids:
            state = robot_states[fid]
            lat = f"{state['avg_lat_ms']:.1f}ms" if state['avg_lat_ms'] else "---"
            ok = "OK" if state['connected'] else "SIN RESPUESTA"
            log.info("  Robot %d: RF=%s | Comandos enviados=%d | Latencia media=%s",
                     fid - 1, ok, len(state['_send_times']), lat)


if __name__ == '__main__':
    main()
