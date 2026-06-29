"""Grabador de video de sesión para los scripts de prueba del Cap. 4.

Añade grabación opcional a un script de medición sin tocar su lógica: el mismo
frame anotado que se muestra en pantalla se vuelca a un .mp4. Reusa el patrón
validado en ``integration/test_behavior_2v2.py`` (temporal oculto con extensión
.mp4 que se renombra al conservar, códec mp4v, comprobación de ``isOpened()``) y
abre el writer de forma diferida con el tamaño real del primer frame.

Toda la E/S va envuelta en ``try/except``: si la grabación falla por códec o por
disco, el writer queda deshabilitado y el script de medición continúa intacto.
La medición y el control nunca dependen de esta clase.

Salida: ``recordings/<script_stem>/<script_stem>_<timestamp>.mp4``.
Para una corrida sin video: ``RECORD_DISABLED=1 python ...``.
"""
import logging
import os
import time
from pathlib import Path

import cv2

log = logging.getLogger(__name__)

ROOT_DIR = Path(__file__).parent.parent.parent
RECORD_DIR = ROOT_DIR / "recordings"
DEFAULT_FPS = 20.0


class SessionRecorder:
    """Graba a video los frames anotados de una sesión de prueba.

    Uso típico::

        rec = SessionRecorder("mi_script")
        while ...:
            rec.write(frame)        # abre el writer en el primer frame
        path = rec.close()          # cierra y conserva; ruta final o None

    El writer se abre con el tamaño del primer frame recibido, de modo que la
    clase funciona igual con la imagen rectificada (640x480) o con cualquier
    otra resolución de cámara sin configuración previa.
    """

    def __init__(self, script_stem, fps=DEFAULT_FPS, enabled=None):
        self.stem = script_stem
        self.fps = fps
        # Opt-out por entorno; por defecto graba.
        if enabled is None:
            enabled = os.environ.get("RECORD_DISABLED") != "1"
        self.enabled = enabled
        self._writer = None
        self._tmp_path = None
        self._final_path = None
        self._frames = 0
        self._opened = False
        self._failed = False

    def _open(self, width, height):
        self._opened = True  # se intenta una sola vez
        try:
            rec_dir = RECORD_DIR / self.stem
            rec_dir.mkdir(parents=True, exist_ok=True)
            ts = time.strftime("%Y%m%d_%H%M%S")
            # El temporal DEBE terminar en .mp4: OpenCV/FFMPEG elige el
            # contenedor por la extensión y una desconocida deja isOpened() en
            # False. Se oculta con un punto inicial y se renombra al cerrar.
            self._tmp_path = rec_dir / f".{self.stem}_{ts}.mp4"
            self._final_path = rec_dir / f"{self.stem}_{ts}.mp4"
            writer = cv2.VideoWriter(
                str(self._tmp_path), cv2.VideoWriter_fourcc(*"mp4v"),
                self.fps, (int(width), int(height)))
            if not writer.isOpened():
                log.warning("VideoWriter no abrió (códec mp4v); grabación deshabilitada")
                self._failed = True
                self._tmp_path = None
                return
            self._writer = writer
            log.info("Grabando sesión → %s", self._tmp_path)
        except Exception as e:
            log.warning("No se pudo iniciar la grabación: %s", e)
            self._failed = True
            self._writer = None
            self._tmp_path = None

    def write(self, frame):
        """Escribe un frame BGR. Abre el writer en la primera llamada."""
        if not self.enabled or self._failed or frame is None:
            return
        if not self._opened:
            h, w = frame.shape[:2]
            self._open(w, h)
        if self._writer is None:
            return
        try:
            self._writer.write(frame)
            self._frames += 1
        except Exception as e:
            log.warning("Fallo al escribir frame de video: %s", e)
            self._failed = True

    @property
    def frames(self):
        return self._frames

    def close(self, keep=True):
        """Cierra el writer. Con ``keep`` y al menos un frame, renombra el
        temporal al .mp4 final y devuelve su ruta; en caso contrario descarta el
        temporal y devuelve ``None``."""
        if self._writer is not None:
            try:
                self._writer.release()
            except Exception:
                pass
        self._writer = None
        if self._tmp_path is None:
            return None
        final = None
        try:
            if keep and self._frames > 0:
                self._tmp_path.rename(self._final_path)
                final = self._final_path
                log.info("Video de sesión guardado en %s (%d frames)",
                         self._final_path, self._frames)
            else:
                self._tmp_path.unlink(missing_ok=True)
                log.info("Grabación descartada (sin frames o keep=False)")
        except Exception as e:
            log.warning("No se pudo finalizar el video: %s", e)
        self._tmp_path = None
        return final
