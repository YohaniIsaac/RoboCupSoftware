import numpy as np

# ==========================================
# LOG
# ==========================================
from robot_soccer.utils.logger import get_logger
from robot_soccer.utils.logger import set_level, disable_module, enable_module
module_name = "entities"

logger = get_logger(module_name)

# Activar depuración detallada para un módulo
set_level(module_name, "WARNING")  # DEBUG, INFO, WARNING, ERROR, CRITICAL, DISABLED
# # Desactivar registro para un módulo que está generando demasiados mensajes
# disable_module("core.physics")
# # Reactivar registro para un módulo previamente desactivado
# enable_module("core.physics", "INFO")
# ==========================================


class Ball:
    def __init__(self, x, y):
        self.x = x
        self.y = y

    def get_position(self):
        return np.array([self.x, self.y])

    def set_position(self, x, y):
        self.x = x
        self.y = y
