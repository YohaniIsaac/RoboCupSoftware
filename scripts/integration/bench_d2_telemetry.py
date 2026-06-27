#!/usr/bin/env python3
"""Bench-test del firmware D2 (telemetría por ACK-payload + toggle DPL del tablero).

Manda comandos CRUDOS al transmisor por serial y muestra TODO lo que responde
(OK / ERROR / TELEM). Corre en ciclo hasta Ctrl+C. A propósito NO usa las capas de
`src/` (rf_controller, serial_manager): prueba el protocolo de cable directo, sin
rate-limiting ni abstracciones que oculten un problema de RF.

Verifica los 3 puntos del bench-test de D2:
  1. Robots responden CON DPL: comandos M (mover) y D (dribbler) → el robot se mueve
     y mueve el rodillo. Si NO responde, el problema es DPL robot↔transmisor.
  2. Tablero intacto: comandos T (gol/pausa/reset) → el marcador responde. Es la prueba
     del toggle disableDynamicPayloads alrededor del write al tablero.
  3. Telemetría: '?,id' (consulta) y 'G,id,1' (nivel eventos) → aparecen líneas
     'TELEM R.. dbg=.. cfg=on/off/wdt eng=.. pwr=.. ev=.. m=.. d=..'.

Uso:
    python scripts/integration/bench_d2_telemetry.py --port /dev/ttyUSB0 --robot-id 1
    # Saltar fases si hace falta (robot que no puede moverse, no tocar el marcador):
    python scripts/integration/bench_d2_telemetry.py --skip-motor --skip-tablero
    Ctrl+C para salir (deja motores y dribbler en 0).
"""
import argparse
import threading
import time

import serial


def reader_loop(ser: serial.Serial, stop: threading.Event) -> None:
    """Imprime TODO lo que el transmisor manda por serial (OK / ERROR / TELEM)."""
    while not stop.is_set():
        try:
            raw = ser.readline()
        except Exception:
            break
        if not raw:
            continue
        line = raw.decode(errors="replace").strip()
        if line:
            mark = "  «" if line.startswith("TELEM") else "  <"
            print(f"{mark} {line}")


def send(ser: serial.Serial, cmd: str, note: str = "") -> None:
    """Envía un comando de texto (el transmisor agrega lo demás) y lo loguea."""
    ser.write((cmd + "\n").encode())
    ser.flush()
    print(f"> {cmd}" + (f"   ({note})" if note else ""))


def hold(ser: serial.Serial, cmd: str, duration_s: float, interval_s: float = 0.05) -> None:
    """Repite un comando para refrescar el watchdog del firmware durante duration_s.

    Motor: watchdog 100ms → interval 50ms. Dribbler: watchdog 150ms → 50ms también sirve.
    """
    print(f"> HOLD {cmd}  ({duration_s:.1f}s, cada {int(interval_s*1000)}ms)")
    t_end = time.time() + duration_s
    while time.time() < t_end:
        ser.write((cmd + "\n").encode())
        time.sleep(interval_s)


def phase_robots(ser: serial.Serial, rid: int) -> None:
    print("\n=== FASE 1: ROBOTS RESPONDEN (DPL) ===")
    print("    Mirá el robot: debe AVANZAR, GIRAR, RETROCEDER y mover el RODILLO.")
    hold(ser, f"M,{rid},45,45", 1.5)            # adelante
    send(ser, f"M,{rid},0,0", "stop")
    hold(ser, f"M,{rid},45,-45", 1.0)           # girar en el lugar
    send(ser, f"M,{rid},0,0", "stop")
    hold(ser, f"M,{rid},-45,-45", 1.5)          # atrás (vuelve aprox.)
    send(ser, f"M,{rid},0,0", "stop")
    hold(ser, f"D,{rid},50", 2.0)               # dribbler ON (refresca; firmware oscila)
    send(ser, f"D,{rid},0", "dribbler OFF")     # apagado explícito e inmediato


def phase_combo(ser: serial.Serial, rid: int) -> None:
    print("\n=== FASE 1b: MOTOR + DRIBBLER SIMULTÁNEOS ===")
    print("    El robot debe AVANZAR y mover el RODILLO A LA VEZ (ambos watchdogs vivos).")
    # Intercalar M y D en el mismo loop de refresco: no hay un paquete que mande ambos, se
    # refrescan los dos < su watchdog (motor 100ms, dribbler 150ms) y el firmware corre los dos.
    t_end = time.time() + 2.5
    print(f"> HOLD M,{rid},45,45 + D,{rid},50  (2.5s, cada 50ms)")
    while time.time() < t_end:
        ser.write((f"M,{rid},45,45\n").encode())
        ser.write((f"D,{rid},50\n").encode())
        time.sleep(0.05)
    send(ser, f"M,{rid},0,0", "stop")
    send(ser, f"D,{rid},0", "dribbler OFF")


def phase_tablero(ser: serial.Serial) -> None:
    print("\n=== FASE 2: TABLERO (toggle DPL — lo crítico) ===")
    print("    Mirá el MARCADOR: debe registrar goles, pausa y resets.")
    for cmd, desc in [("T2", "gol equipo 1"), ("T3", "gol equipo 2"),
                      ("T1", "toggle pausa"), ("T4", "reset goles"),
                      ("T5", "reset tiempo")]:
        send(ser, cmd, desc)
        time.sleep(1.2)


def phase_telemetry(ser: serial.Serial, rid: int) -> None:
    print("\n=== FASE 3: TELEMETRÍA ===")
    print("    Esperá líneas 'TELEM R..': consulta directa y eventos del dribbler.")
    send(ser, f"?,{rid}", "consulta de estado")
    time.sleep(0.6)
    hold(ser, f"D,{rid},50", 1.5)               # engage → evento ev=1 (eng=1)
    send(ser, f"D,{rid},0", "off -> ev=2")
    time.sleep(0.4)
    send(ser, f"?,{rid}", "consulta de nuevo")
    time.sleep(0.6)


def main() -> None:
    ap = argparse.ArgumentParser(description="Bench-test D2: DPL + telemetría + tablero.")
    ap.add_argument("--port", default="/dev/ttyUSB0", help="Puerto serial del transmisor.")
    ap.add_argument("--robot-id", type=int, default=1, help="ID de robot (1-4).")
    ap.add_argument("--skip-motor", action="store_true", help="No mover el robot (fase 1).")
    ap.add_argument("--skip-tablero", action="store_true", help="No tocar el marcador (fase 2).")
    args = ap.parse_args()
    rid = args.robot_id

    ser = serial.Serial(args.port, 115200, timeout=0.1)
    print(f"Abriendo {args.port} @115200 (el transmisor se reinicia al abrir)...")
    time.sleep(2.0)  # esperar el reboot del Arduino transmisor

    stop = threading.Event()
    rt = threading.Thread(target=reader_loop, args=(ser, stop), daemon=True)
    rt.start()

    print("Conectado. Nivel de debug -> 1 (eventos). Ctrl+C para salir.\n")
    send(ser, f"G,{rid},1", "nivel telemetría = eventos (en partido iría G,id,0)")
    time.sleep(0.5)

    cycle = 0
    try:
        while True:
            cycle += 1
            print(f"\n########################  CICLO {cycle}  ########################")
            if not args.skip_motor:
                phase_robots(ser, rid)
                phase_combo(ser, rid)
            if not args.skip_tablero:
                phase_tablero(ser)
            phase_telemetry(ser, rid)
            print("\n... (pausa 3s antes del próximo ciclo; Ctrl+C para terminar)")
            time.sleep(3.0)
    except KeyboardInterrupt:
        print("\nSaliendo: deteniendo robot y apagando dribbler...")
    finally:
        try:
            send(ser, f"M,{rid},0,0", "stop")
            send(ser, f"D,{rid},0", "dribbler OFF")
            send(ser, f"G,{rid},0", "nivel telemetría = 0 (partido)")
            time.sleep(0.3)
        finally:
            stop.set()
            time.sleep(0.2)
            ser.close()
            print("Puerto cerrado.")


if __name__ == "__main__":
    main()
