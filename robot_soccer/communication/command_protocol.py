"""
Protocolo específico para comunicarse con los robots
"""

class RobotCommandProtocol:
    """
    Define el protocolo de comandos para comunicarse con los robots.
    Genera cadenas de comandos en el formato esperado por el Arduino.
    """

    @staticmethod
    def format_motor_command(robot_id, left_speed, right_speed):
        """
        Formatea un comando de control de motores.

        Args:
            robot_id: ID del robot (1-4)
            left_speed: Velocidad del motor izquierdo (-255 a 255)
            right_speed: Velocidad del motor derecho (-255 a 255)

        Returns:
            str: Comando formateado
        """
        # Formato: M,id,left,right\n
        # Ejemplo: M,1,100,-100\n

        # Limitar valores entre -255 y 255
        left = max(-255, min(255, int(left_speed)))
        right = max(-255, min(255, int(right_speed)))

        return f"M,{robot_id},{left},{right}"

    @staticmethod
    def format_kick_command(robot_id, power=255):
        """
        Formatea un comando para activar el mecanismo de pateo.

        Args:
            robot_id: ID del robot (1-4)
            power: Potencia del pateo (0-255)

        Returns:
            str: Comando formateado
        """
        # Formato: K,id,power\n
        # Ejemplo: K,1,255\n

        power_val = max(0, min(255, int(power)))

        return f"K,{robot_id},{power_val}"

    @staticmethod
    def format_dribbler_command(robot_id, power):
        """
        Formatea un comando para activar/desactivar el dribbler.

        Args:
            robot_id: ID del robot (1-4)
            power: Potencia del dribbler (0-255)

        Returns:
            str: Comando formateado
        """
        # Formato: D,id,power\n
        # Ejemplo: D,1,200\n

        power_val = max(0, min(255, int(power)))

        return f"D,{robot_id},{power_val}"

    @staticmethod
    def format_stop_command(robot_id):
        """
        Formatea un comando para detener todos los motores.

        Args:
            robot_id: ID del robot (1-4)

        Returns:
            str: Comando formateado
        """
        # Formato: S,id\n
        # Ejemplo: S,1\n

        return f"S,{robot_id}"