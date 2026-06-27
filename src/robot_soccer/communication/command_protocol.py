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

        IMPORTANTE: El firmware acepta valores int8_t (-127 a 127), NO -255 a 255.

        Args:
            robot_id (int): Identificador único del robot (1-4).
            left_speed (int): Velocidad del motor izquierdo (-127 a 127).
                Valores negativos para retroceso, positivos para avance.
            right_speed (int): Velocidad del motor derecho (-127 a 127).
                Valores negativos para retroceso, positivos para avance.

        Returns:
            str: Comando formateado en formato "M,id,left,right".

        Example:
            >>> RobotCommandProtocol.format_motor_command(1, 100, -100)
            'M,1,100,-100'
        """
        # Formato: M,id,left,right\n
        # Ejemplo: M,1,100,-100\n

        # Limitar valores entre -127 y 127 (rango int8_t del firmware)
        left = max(-127, min(127, int(left_speed)))
        right = max(-127, min(127, int(right_speed)))

        return f"M,{robot_id},{left},{right}"

    @staticmethod
    def format_kick_command(robot_id, power=255):
        """Formatea un comando para activar el mecanismo de pateo.

        El firmware activa el solenoide durante TIEMPO_PATEO_MS ms (fijo en config.h).
        El parámetro power se ignora — el hardware no soporta potencia variable.

        Protocolo transmisor: "R[id]P"  →  ejecutarComando('P') en el robot.

        Args:
            robot_id (int): Identificador único del robot (1-4).
            power (int, optional): Ignorado. Solo existe por compatibilidad de interfaz.

        Returns:
            str: Comando formateado "R{id}P".

        Example:
            >>> RobotCommandProtocol.format_kick_command(1)
            'R1P'
        """
        return f"R{robot_id}P"

    @staticmethod
    def format_dribbler_command(robot_id, power):
        """Formatea un comando para controlar el dribbler con potencia variable.

        Protocolo transmisor: "D,id,power" → paquete RF 5 bytes ['D', id, pwm, 0, 0]
        El firmware del robot usa SoftPWM para controlar la velocidad del motor DC.

        Args:
            robot_id (int): Identificador único del robot (1-4).
            power (int): Potencia PWM (0-255). 0 = apagado, 255 = máximo.

        Returns:
            str: Comando formateado "D,{id},{power}".

        Example:
            >>> RobotCommandProtocol.format_dribbler_command(1, 255)
            'D,1,255'
            >>> RobotCommandProtocol.format_dribbler_command(1, 0)
            'D,1,0'
        """
        pwm = max(0, min(255, int(power)))
        return f"D,{robot_id},{pwm}"

    @staticmethod
    def format_dribbler_config(robot_id, on_ms, off_ms, wdt_ms):
        """Formatea la config de oscilación del dribbler (persiste en EEPROM del robot).

        Protocolo transmisor: "C,id,onMs,offMs,wdtMs" → paquete RF 5 bytes
        ['C', id, onMs, offMs, wdtMs]. El robot la guarda en EEPROM y oscila el rodillo con
        ese duty de forma autónoma; wdtMs es su watchdog propio (sin refresco 'D' → apaga).
        Cada valor es un byte (0-255).

        Args:
            robot_id (int): Identificador del robot (1-4).
            on_ms (int): Duración de la fase encendida (ms, 0-255).
            off_ms (int): Duración de la fase apagada (ms, 0-255).
            wdt_ms (int): Watchdog del dribbler (ms, 0-255).

        Returns:
            str: Comando formateado "C,{id},{on},{off},{wdt}".

        Example:
            >>> RobotCommandProtocol.format_dribbler_config(1, 65, 15, 150)
            'C,1,65,15,150'
        """
        on  = max(0, min(255, int(on_ms)))
        off = max(0, min(255, int(off_ms)))
        wdt = max(0, min(255, int(wdt_ms)))
        return f"C,{robot_id},{on},{off},{wdt}"

    @staticmethod
    def format_stop_command(robot_id):
        """Formatea un comando de detención de motores de tracción.

        Envía velocidad 0,0 vía el comando M para detener ambos motores.
        Usa el formato motor porque el transmisor solo acepta M y R[id][cmd].

        Args:
            robot_id (int): Identificador único del robot (1-4).

        Returns:
            str: Comando formateado "M,{id},0,0".

        Example:
            >>> RobotCommandProtocol.format_stop_command(1)
            'M,1,0,0'
        """
        return f"M,{robot_id},0,0"

    @staticmethod
    def format_tablero_command(cmd_num):
        """Formatea un comando para el tablero de puntuación.

        El transmisor lo convierte a un paquete RF de 3 bytes ['G', cmd, 0]
        enviado a la dirección "00001" del tablero.

        Args:
            cmd_num (int): Número de comando (1-5).
                1 = toggle pausa/resume del cronómetro
                2 = gol equipo 1 (rojo)
                3 = gol equipo 2 (azul)
                4 = reset goles
                5 = reset cronómetro

        Returns:
            str: Comando formateado "T{n}".

        Example:
            >>> RobotCommandProtocol.format_tablero_command(2)
            'T2'
        """
        return f"T{cmd_num}"
