"""
Sistema de Árboles de Comportamiento para Robot Soccer

Este módulo implementa un sistema de Árboles de Comportamiento (Behavior Trees)
para controlar los robots de forma flexible y modular
"""

from enum import Enum
import logging


# Estado de ejecución de un nodo en el árbol
class NodeStatus(Enum):
    RUNNING = 0  # Nodo en ejecución
    SUCCESS = 1  # Nodo ejecutado con éxito
    FAILURE = 2  # Nodo ejecutado sin éxito
    INVALID = 3  # Estado no válido


class BehaviorNode:
    """Clase base para todos los nodos del árbol de comportamiento."""

    def __init__(self, name="Node"):
        self.name = name
        self.status = NodeStatus.INVALID
        self.parent = None
        self.logger = logging.getLogger(f"ai.behavior_tree.{self.name}")

    def tick(self, blackboard):
        """
        Ejecuta una iteración del nodo.

        Args:
            blackboard: Objeto con datos compartidos del entorno

        Returns:
            NodeStatus: Estado resultante de la ejecución
        """
        self.logger.debug(f"Ticking node {self.name}")
        self.status = self._process(blackboard)
        self.logger.debug(f"Node {self.name} returned {self.status}")
        return self.status

    def _process(self, blackboard):
        """
        Método a implementar por las clases derivadas.
        Contiene la lógica específica del nodo.
        """
        raise NotImplementedError("_process debe ser implementado por la clase derivada")


# NODOS COMPUESTOS

class CompositeNode(BehaviorNode):
    """Nodo que puede tener hijos."""

    def __init__(self, name="Composite"):
        super().__init__(name)
        self.children = []

    def add_child(self, child):
        """Añade un nodo hijo."""
        child.parent = self
        self.children.append(child)
        return self  # Para permitir encadenamiento

    def add_children(self, *children):
        """Añade múltiples nodos hijos."""
        for child in children:
            self.add_child(child)
        return self  # Para permitir encadenamiento


class SequenceNode(CompositeNode):
    """
    Ejecuta sus hijos en secuencia hasta que uno falla.
    Solo tiene éxito si todos los hijos tienen éxito.
    """

    def __init__(self, name="Sequence"):
        super().__init__(name)
        self.current_child_idx = 0

    def _process(self, blackboard):
        # Reiniciar si estamos empezando de nuevo
        if self.status != NodeStatus.RUNNING:
            self.current_child_idx = 0

        # Ejecutar hijos en secuencia
        while self.current_child_idx < len(self.children):
            child = self.children[self.current_child_idx]
            status = child.tick(blackboard)

            if status == NodeStatus.RUNNING:
                return NodeStatus.RUNNING

            if status == NodeStatus.FAILURE:
                return NodeStatus.FAILURE

            # Si llegamos aquí, el hijo tuvo éxito, pasamos al siguiente
            self.current_child_idx += 1

        # Si llegamos al final, la secuencia tuvo éxito
        return NodeStatus.SUCCESS


class SelectorNode(CompositeNode):
    """
    Ejecuta sus hijos en secuencia hasta que uno tiene éxito.
    Solo falla si todos los hijos fallan.
    """

    def __init__(self, name="Selector"):
        super().__init__(name)
        self.current_child_idx = 0

    def _process(self, blackboard):
        # Reiniciar si estamos empezando de nuevo
        if self.status != NodeStatus.RUNNING:
            self.current_child_idx = 0

        # Ejecutar hijos en secuencia
        while self.current_child_idx < len(self.children):
            child = self.children[self.current_child_idx]
            status = child.tick(blackboard)

            if status == NodeStatus.RUNNING:
                return NodeStatus.RUNNING

            if status == NodeStatus.SUCCESS:
                return NodeStatus.SUCCESS

            # Si llegamos aquí, el hijo falló, pasamos al siguiente
            self.current_child_idx += 1

        # Si llegamos al final, todos fallaron
        return NodeStatus.FAILURE


class ParallelNode(CompositeNode):
    """
    Ejecuta todos sus hijos en paralelo.
    El éxito/fracaso depende del policy establecido.
    """

    def __init__(self, name="Parallel", success_threshold=None, failure_threshold=None):
        """
        Args:
            success_threshold: Número de hijos que deben tener éxito para que el nodo tenga éxito
                               (None = todos)
            failure_threshold: Número de hijos que deben fallar para que el nodo falle
                               (None = todos)
        """
        super().__init__(name)
        self.success_threshold = success_threshold
        self.failure_threshold = failure_threshold

    def _process(self, blackboard):
        # Contar resultados
        success_count = 0
        failure_count = 0
        running_count = 0

        for child in self.children:
            status = child.tick(blackboard)

            if status == NodeStatus.SUCCESS:
                success_count += 1
            elif status == NodeStatus.FAILURE:
                failure_count += 1
            elif status == NodeStatus.RUNNING:
                running_count += 1

        # Comprobar umbrales
        success_threshold = self.success_threshold if self.success_threshold is not None else len(self.children)
        failure_threshold = self.failure_threshold if self.failure_threshold is not None else len(self.children)

        if success_count >= success_threshold:
            return NodeStatus.SUCCESS

        if failure_count >= failure_threshold:
            return NodeStatus.FAILURE

        # Si no se ha alcanzado ningún umbral, sigue ejecutándose
        return NodeStatus.RUNNING


# NODOS DECORADORES

class DecoratorNode(BehaviorNode):
    """Nodo que modifica el comportamiento de un único hijo."""

    def __init__(self, child=None, name="Decorator"):
        super().__init__(name)
        self.child = None
        if child:
            self.set_child(child)

    def set_child(self, child):
        """Establece el nodo hijo."""
        self.child = child
        child.parent = self
        return self  # Para permitir encadenamiento


class InverterNode(DecoratorNode):
    """Invierte el resultado del hijo: éxito -> fracaso, fracaso -> éxito."""

    def __init__(self, child=None, name="Inverter"):
        super().__init__(child, name)

    def _process(self, blackboard):
        if not self.child:
            return NodeStatus.FAILURE

        status = self.child.tick(blackboard)

        if status == NodeStatus.SUCCESS:
            return NodeStatus.FAILURE
        elif status == NodeStatus.FAILURE:
            return NodeStatus.SUCCESS

        return status  # RUNNING o INVALID se mantienen igual


class RepeatNode(DecoratorNode):
    """Repite la ejecución del hijo un número específico de veces."""

    def __init__(self, child=None, num_repeats=None, name="Repeat"):
        """
        Args:
            num_repeats: Número de repeticiones (None = infinito)
        """
        super().__init__(child, name)
        self.num_repeats = num_repeats
        self.current_repeats = 0

    def _process(self, blackboard):
        if not self.child:
            return NodeStatus.FAILURE

        # Reiniciar contador si es necesario
        if self.status != NodeStatus.RUNNING:
            self.current_repeats = 0

        # Verificar si hemos alcanzado el límite
        if self.num_repeats is not None and self.current_repeats >= self.num_repeats:
            return NodeStatus.SUCCESS

        # Ejecutar hijo
        status = self.child.tick(blackboard)

        # Si el hijo está en ejecución, seguimos esperando
        if status == NodeStatus.RUNNING:
            return NodeStatus.RUNNING

        # Si el hijo falló, propagamos el fallo
        if status == NodeStatus.FAILURE:
            return NodeStatus.FAILURE

        # Si el hijo tuvo éxito, incrementamos el contador y seguimos
        self.current_repeats += 1

        # Verificar de nuevo si hemos alcanzado el límite
        if self.num_repeats is not None and self.current_repeats >= self.num_repeats:
            return NodeStatus.SUCCESS

        # Si no hemos terminado, seguimos ejecutando
        return NodeStatus.RUNNING


class ConditionNode(BehaviorNode):
    """
    Nodo que evalúa una condición sobre el blackboard.
    Tiene éxito si la condición es verdadera, falla en caso contrario.
    """

    def __init__(self, condition_func, name="Condition"):
        """
        Args:
            condition_func: Función que toma el blackboard y devuelve un booleano
        """
        super().__init__(name)
        self.condition_func = condition_func

    def _process(self, blackboard):
        if self.condition_func(blackboard):
            return NodeStatus.SUCCESS
        return NodeStatus.FAILURE


class ActionNode(BehaviorNode):
    """
    Nodo que ejecuta una acción sobre el mundo.
    Devuelve el estado proporcionado por la acción.
    """

    def __init__(self, action_func, name="Action"):
        """
        Args:
            action_func: Función que toma el blackboard y devuelve un NodeStatus
        """
        super().__init__(name)
        self.action_func = action_func

    def _process(self, blackboard):
        return self.action_func(blackboard)