"""Shared memory con double buffering para distribución de frames.

Permite que un productor (cámara o simulación) escriba frames y múltiples
consumidores los lean sin serialización pickle. El double buffering garantiza
que los lectores nunca leen un buffer que está siendo escrito.

Flujo:
    1. Writer escribe en buffer inactivo via np.copyto (~0.5ms)
    2. Writer hace swap atómico de active_index
    3. Writer incrementa frame_counter
    4. Readers detectan nuevo frame via frame_counter
    5. Readers copian localmente el buffer activo (nunca siendo escrito)
"""

import logging
from multiprocessing import Value, shared_memory

import numpy as np

log = logging.getLogger(__name__)


class SharedFrameWriter:
    """Productor de frames con double buffering.

    Crea dos SharedMemory buffers y alterna entre ellos. Los consumidores
    siempre leen del buffer activo mientras el productor escribe en el inactivo.

    Args:
        height: Alto del frame en píxeles.
        width: Ancho del frame en píxeles.
        channels: Canales de color (default: 3 para BGR).
    """

    def __init__(self, height, width, channels=3):
        self.shape = (height, width, channels)
        self.nbytes = int(np.prod(self.shape))

        # Crear 2 buffers de shared memory
        self._shm = [
            shared_memory.SharedMemory(create=True, size=self.nbytes),
            shared_memory.SharedMemory(create=True, size=self.nbytes),
        ]

        # Arrays numpy que apuntan a los buffers
        self._arrays = [
            np.ndarray(self.shape, dtype=np.uint8, buffer=self._shm[i].buf)
            for i in range(2)
        ]

        # Inicializar buffers a cero
        for arr in self._arrays:
            arr[:] = 0

        # Índice del buffer activo (el que leen los consumidores)
        self._active_index = Value('i', 0)
        # Contador de frames para que los readers detecten actualizaciones
        self._frame_counter = Value('i', 0)

        log.info(
            "SharedFrameWriter creado: %dx%dx%d (%d bytes x2 buffers, shm=%s/%s)",
            height, width, channels, self.nbytes,
            self._shm[0].name, self._shm[1].name,
        )

    def write(self, frame):
        """Escribe un frame al buffer inactivo y hace swap atómico.

        Args:
            frame: numpy array de shape (height, width, channels), dtype uint8.
        """
        # Escribir en el buffer INACTIVO (el opuesto al que leen los consumers)
        inactive = 1 - self._active_index.value
        np.copyto(self._arrays[inactive], frame)

        # Swap atómico: ahora los readers leen el buffer recién escrito
        with self._active_index.get_lock():
            self._active_index.value = inactive

        # Señalar nuevo frame disponible
        with self._frame_counter.get_lock():
            self._frame_counter.value += 1

    def config(self):
        """Retorna dict serializable para pasar a procesos consumidores.

        Returns:
            dict con nombres de shared memory, Values y dimensiones.
        """
        return {
            'shm_names': (self._shm[0].name, self._shm[1].name),
            'shape': self.shape,
            'active_index': self._active_index,
            'frame_counter': self._frame_counter,
        }

    def cleanup(self):
        """Cierra y elimina los buffers de shared memory."""
        for shm in self._shm:
            try:
                shm.close()
                shm.unlink()
            except Exception as e:
                log.warning("Error limpiando shared memory %s: %s", shm.name, e)
        log.info("SharedFrameWriter limpiado")


class SharedFrameReader:
    """Consumidor de frames desde shared memory con double buffering.

    Se conecta a los buffers creados por SharedFrameWriter y lee frames
    sin serialización. Siempre lee del buffer activo (no siendo escrito).

    Args:
        frame_config: dict retornado por SharedFrameWriter.config().
    """

    def __init__(self, frame_config):
        shm_names = frame_config['shm_names']
        self.shape = frame_config['shape']
        self._active_index = frame_config['active_index']
        self._frame_counter = frame_config['frame_counter']
        self._last_counter = 0

        # Conectar a shared memory existente
        self._shm = [
            shared_memory.SharedMemory(name=shm_names[0], create=False),
            shared_memory.SharedMemory(name=shm_names[1], create=False),
        ]

        self._arrays = [
            np.ndarray(self.shape, dtype=np.uint8, buffer=self._shm[i].buf)
            for i in range(2)
        ]

        log.info(
            "SharedFrameReader conectado: shm=%s/%s",
            shm_names[0], shm_names[1],
        )

    def read(self, blocking_timeout=5.0):
        """Lee el frame activo más reciente.

        Espera hasta que haya un frame nuevo (frame_counter cambie) y retorna
        una copia local del buffer activo.

        Args:
            blocking_timeout: Tiempo máximo de espera en segundos.

        Returns:
            numpy array (copia local) del frame, o None si timeout.
        """
        import time
        deadline = time.monotonic() + blocking_timeout

        # Esperar a que haya un frame nuevo
        while self._frame_counter.value == self._last_counter:
            if time.monotonic() >= deadline:
                return None
            time.sleep(0.001)  # 1ms polling

        # Leer del buffer activo (el writer nunca lo toca mientras es activo)
        active = self._active_index.value
        frame = self._arrays[active].copy()
        self._last_counter = self._frame_counter.value
        return frame

    def cleanup(self):
        """Cierra la conexión a shared memory (sin unlink)."""
        for shm in self._shm:
            try:
                shm.close()
            except Exception as e:
                log.warning("Error cerrando shared memory: %s", e)
