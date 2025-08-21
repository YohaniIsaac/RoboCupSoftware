"""Protocolo de comunicación para robots de fútbol.

Este módulo define el protocolo de comandos para comunicarse con los robots
a través de Arduino. Genera cadenas de comandos en el formato específico
esperado por el firmware de los robots.

El protocolo soporta comandos para:
- Control de motores diferenciales
- Activación del mecanismo de pateo
- Control del dribbler
- Detención de emergencia

Todos los comandos siguen el formato: COMANDO,ID,PARAMETROS
"""


class RobotCommandProtocol:
    """Define el protocolo de comandos para comunicación con robots de fútbol.

    Esta clase proporciona métodos estáticos para formatear comandos
    que serán enviados al Arduino que controla los robots. Cada comando
    sigue un formato específico reconocido por el firmware.

    El protocolo utiliza comandos de texto plano separados por comas,
    lo que facilita el debugging y la implementación en el Arduino.
    """

    @staticmethod
    def format_motor_command(robot_id, left_speed, right_speed):
        """Formatea un comando de control de motores diferenciales.

        Genera un comando para controlar independientemente las velocidades
        de los motores izquierdo y derecho del robot. Los valores se limitan
        automáticamente al rango válido.

        Args:
            robot_id (int): Identificador único del robot (1-4).
            left_speed (int): Velocidad del motor izquierdo (-255 a 255).
                Valores negativos para retroceso, positivos para avance.
            right_speed (int): Velocidad del motor derecho (-255 a 255).
                Valores negativos para retroceso, positivos para avance.

        Returns:
            str: Comando formateado en formato "M,id,left,right".

        Example:
            >>> RobotCommandProtocol.format_motor_command(1, 100, -100)
            'M,1,100,-100'
        """
        # Formato: M,id,left,right\n
        # Ejemplo: M,1,100,-100\n

        # Limitar valores entre -255 y 255
        left = max(-255, min(255, int(left_speed)))
        right = max(-255, min(255, int(right_speed)))

        return f"M,{robot_id},{left},{right}"

    @staticmethod
    def format_kick_command(robot_id, power=255):
        """Formatea un comando para activar el mecanismo de pateo.

        Genera un comando para activar el solenoides o motor de pateo
        del robot con la potencia especificada. La potencia se limita
        automáticamente al rango válido.

        Args:
            robot_id (int): Identificador único del robot (1-4).
            power (int, optional): Potencia del pateo (0-255).
                Defaults to 255 (potencia máxima).

        Returns:
            str: Comando formateado en formato "K,id,power".

        Example:
            >>> RobotCommandProtocol.format_kick_command(1, 200)
            'K,1,200'
        """
        # Formato: K,id,power\n
        # Ejemplo: K,1,255\n

        power_val = max(0, min(255, int(power)))

        return f"K,{robot_id},{power_val}"

    @staticmethod
    def format_dribbler_command(robot_id, power):
        """Formatea un comando para controlar el dribbler.

        Genera un comando para activar o desactivar el mecanismo dribbler
        que mantiene la pelota controlada. Una potencia de 0 desactiva
        el dribbler, valores mayores lo activan con diferentes intensidades.

        Args:
            robot_id (int): Identificador único del robot (1-4).
            power (int): Potencia del dribbler (0-255).
                0 para desactivar, valores mayores para diferentes intensidades.

        Returns:
            str: Comando formateado en formato "D,id,power".

        Example:
            >>> RobotCommandProtocol.format_dribbler_command(1, 150)
            'D,1,150'
        """
        # Formato: D,id,power\n
        # Ejemplo: D,1,200\n

        power_val = max(0, min(255, int(power)))

        return f"D,{robot_id},{power_val}"

    @staticmethod
    def format_stop_command(robot_id):
        """Formatea un comando de detención de emergencia.

        Genera un comando para detener inmediatamente todos los motores
        del robot especificado. Esto incluye motores de tracción, dribbler
        y cualquier otro actuador activo.

        Args:
            robot_id (int): Identificador único del robot (1-4).

        Returns:
            str: Comando formateado en formato "S,id".

        Example:
            >>> RobotCommandProtocol.format_stop_command(1)
            'S,1'
        """
        # Formato: S,id\n
        # Ejemplo: S,1\n

        return f"S,{robot_id}"
