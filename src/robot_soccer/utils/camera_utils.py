"""Utilidades para detección y configuración de cámaras.

Este módulo proporciona funciones para detectar cámaras disponibles,
identificar cámaras específicas (como DroidCam) y validar dispositivos de video.
"""

import logging
import subprocess
from pathlib import Path
import cv2

log = logging.getLogger(__name__)


def find_droidcam_device():
    """Encuentra automáticamente el dispositivo de video de DroidCam.

    Busca en todos los dispositivos /dev/videoX para encontrar cuál
    está usando el driver v4l2loopback (DroidCam).

    Returns:
        int or None: Índice del dispositivo DroidCam (ej: 2 para /dev/video2),
                     o None si no se encuentra.

    Example:
        >>> camera_id = find_droidcam_device()
        >>> if camera_id is not None:
        ...     cap = cv2.VideoCapture(camera_id)
    """
    # Buscar todos los dispositivos /dev/videoX
    video_devices = sorted(Path("/dev").glob("video*"))

    for device_path in video_devices:
        device_num = device_path.name.replace("video", "")

        # Verificar si es un número (evitar video4linux, etc.)
        if not device_num.isdigit():
            continue

        device_index = int(device_num)

        try:
            # Usar v4l2-ctl para obtener información del driver
            result = subprocess.run(
                ["v4l2-ctl", "--device", str(device_path), "--info"],
                capture_output=True,
                text=True,
                timeout=2
            )

            # Buscar "v4l2 loopback" en la salida
            if "v4l2 loopback" in result.stdout.lower():
                log.info("📹 DroidCam encontrado en /dev/video%d", device_index)
                return device_index

        except (subprocess.SubprocessError, subprocess.TimeoutExpired, FileNotFoundError):
            # Si v4l2-ctl no está disponible o falla, continuar
            continue

    log.warning("⚠️  No se encontró DroidCam. Usando cámara por defecto (0).")
    return None


def find_camera_by_name(name_pattern):
    """Busca una cámara por nombre o patrón en la descripción.

    Args:
        name_pattern (str): Patrón a buscar en el nombre del dispositivo
                           (case-insensitive).

    Returns:
        int or None: Índice del dispositivo encontrado, o None si no se encuentra.

    Example:
        >>> camera_id = find_camera_by_name("loopback")
        >>> camera_id = find_camera_by_name("droidcam")
    """
    video_devices = sorted(Path("/dev").glob("video*"))

    for device_path in video_devices:
        device_num = device_path.name.replace("video", "")

        if not device_num.isdigit():
            continue

        device_index = int(device_num)

        try:
            result = subprocess.run(
                ["v4l2-ctl", "--device", str(device_path), "--info"],
                capture_output=True,
                text=True,
                timeout=2
            )

            if name_pattern.lower() in result.stdout.lower():
                log.info("📹 Cámara '%s' encontrada en /dev/video%d", name_pattern, device_index)
                return device_index

        except (subprocess.SubprocessError, subprocess.TimeoutExpired, FileNotFoundError):
            continue

    return None


def list_available_cameras(max_cameras=10):
    """Lista todas las cámaras disponibles y sus propiedades.

    Args:
        max_cameras (int): Número máximo de índices a verificar (0 a max_cameras-1).

    Returns:
        list: Lista de diccionarios con información de cámaras disponibles.
              Cada diccionario contiene:
              - 'index': Índice del dispositivo
              - 'path': Ruta del dispositivo (/dev/videoX)
              - 'name': Nombre del dispositivo (si está disponible)
              - 'driver': Driver utilizado (si está disponible)
              - 'working': True si OpenCV puede abrir la cámara

    Example:
        >>> cameras = list_available_cameras()
        >>> for cam in cameras:
        ...     print(f"Camera {cam['index']}: {cam['name']} - Working: {cam['working']}")
    """
    cameras = []

    for i in range(max_cameras):
        device_path = Path(f"/dev/video{i}")

        if not device_path.exists():
            continue

        cam_info = {
            'index': i,
            'path': str(device_path),
            'name': 'Unknown',
            'driver': 'Unknown',
            'working': False
        }

        # Intentar obtener información con v4l2-ctl
        try:
            result = subprocess.run(
                ["v4l2-ctl", "--device", str(device_path), "--info"],
                capture_output=True,
                text=True,
                timeout=2
            )

            # Parsear nombre del dispositivo
            for line in result.stdout.split('\n'):
                if 'Card type' in line or 'card' in line.lower():
                    cam_info['name'] = line.split(':')[-1].strip()
                elif 'Driver name' in line or 'driver' in line.lower():
                    cam_info['driver'] = line.split(':')[-1].strip()

        except (subprocess.SubprocessError, subprocess.TimeoutExpired, FileNotFoundError):
            pass

        # Verificar si OpenCV puede abrir la cámara
        try:
            cap = cv2.VideoCapture(i)
            if cap.isOpened():
                cam_info['working'] = True
                cap.release()
        except Exception:
            pass

        cameras.append(cam_info)

    return cameras


def get_camera_index(prefer_droidcam=True, fallback_index=0):
    """Obtiene el índice de cámara a usar, con detección automática de DroidCam.

    Args:
        prefer_droidcam (bool): Si True, intenta encontrar DroidCam primero.
                                Si False, usa fallback_index directamente.
        fallback_index (int): Índice a usar si no se encuentra DroidCam o
                              prefer_droidcam=False. Por defecto 0.

    Returns:
        int: Índice de la cámara a usar.

    Example:
        >>> camera_id = get_camera_index(prefer_droidcam=True, fallback_index=0)
        >>> cap = cv2.VideoCapture(camera_id)
    """
    if prefer_droidcam:
        droidcam_index = find_droidcam_device()
        if droidcam_index is not None:
            return droidcam_index

        log.info("ℹ️  DroidCam no encontrado, usando cámara %d", fallback_index)

    return fallback_index
