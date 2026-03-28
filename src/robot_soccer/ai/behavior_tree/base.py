"""Sistema de Árboles de Comportamiento para Robot Soccer.

Este módulo implementa un sistema de Árboles de Comportamiento (Behavior Trees)
para controlar los robots de forma flexible y modular.
"""

import logging
from enum import Enum

log = logging.getLogger(__name__)


class NodeStatus(Enum):
    """Enumeración de posibles estados de ejecución del nodo."""

    RUNNING = 0  # Nodo en ejecución
    SUCCESS = 1  # Nodo ejecutado con éxito
    FAILURE = 2  # Nodo ejecutado sin éxito
    INVALID = 3  # Estado no válido


class BehaviorTreeTracer:
    """Rastreador para depurar la ejecución del árbol de comportamiento.

    Rastrea la ejecución de los nodos y proporciona información de depuración para el
    análisis del árbol de comportamiento.
    """

    def __init__(self):
        """Inicializa el tracer con estructuras vacías."""
        self.trace = []
        self.next_action = None
        self.conditions_met = []

    def clear(self):
        """Limpia todas las estructuras de traza."""
        self.trace = []
        self.next_action = None
        self.conditions_met = []

    def add_node_result(self, node_name, node_type, status):
        """Registra el resultado de un nodo.

        Args:
            node_name (str): Nombre del nodo ejecutado.
            node_type (str): Tipo del nodo.
            status (NodeStatus): Estado resultante de la ejecución.
        """
        self.trace.append({
            'name': node_name,
            'type': node_type,
            'status': status
        })

    def set_next_action(self, action_name, priority=None):
        """Establece la próxima acción a ejecutar.

        Args:
            action_name (str): Nombre de la acción.
            priority (int, optional): Prioridad de la acción.
        """
        self.next_action = {
            'name': action_name,
            'priority': priority
        }

    def add_condition_met(self, condition_name, result):
        """Registra una condición evaluada.

        Args:
            condition_name (str): Nombre de la condición.
            result (bool): Resultado de la evaluación.
        """
        self.conditions_met.append({
            'name': condition_name,
            'result': result
        })

global_tracer = BehaviorTreeTracer()


class BehaviorNode:
    """Clase base para todos los nodos del árbol de comportamiento."""

    def __init__(self, name="Node"):
        """Inicializa el nodo de comportamiento.

        Args:
            name (str): Nombre del nodo.
        """
        self.name = name
        self.status = NodeStatus.INVALID
        self.parent = None

    def tick(self, blackboard):
        """Ejecuta una iteración del nodo.

        Args:
            blackboard: Objeto con datos compartidos del entorno

        Returns:
            NodeStatus: Estado resultante de la ejecución
        """
        log.debug("Ticking node %s", self.name)
        self.status = self._process(blackboard)
        log.debug("Node %s returned %s", self.name, self.status)

        # Registrar en el tracer
        node_type = self.__class__.__name__
        global_tracer.add_node_result(self.name, node_type, self.status)

        return self.status

    def _process(self, blackboard):
        """Método a implementar por las clases derivadas.

        Contiene la lógica específica del nodo.

        Args:
            blackboard: Objeto con datos compartidos del entorno.

        Returns:
            NodeStatus: Estado resultante de la ejecución.

        Raises:
            NotImplementedError: Si no se implementa en la clase derivada.
        """
        raise NotImplementedError(
            "_process debe ser implementado por la clase derivada"
        )


# NODOS COMPUESTOS

class CompositeNode(BehaviorNode):
    """Nodo que puede tener hijos."""

    def __init__(self, name="Composite"):
        """Inicializa el nodo compuesto.

        Args:
            name (str): Nombre del nodo compuesto.
        """
        super().__init__(name)
        self.children = []

    def _process(self, blackboard):
        """Process the composite node logic."""
        raise NotImplementedError("Subclasses must implement _process")

    def add_child(self, child):
        """Añade un nodo hijo al nodo compuesto.

        Args:
            child (BehaviorNode): Nodo hijo a añadir.
        """
        child.parent = self
        self.children.append(child)
        return self  # Para permitir encadenamiento

    def add_children(self, *children):
        """Añade múltiples nodos hijos."""
        for child in children:
            self.add_child(child)
        return self  # Para permitir encadenamiento

class SequenceNode(CompositeNode):
    """Nodo secuencia que ejecuta hijos en orden hasta que uno falle.

    Ejecuta los nodos hijos en secuencia. Si todos tienen éxito, retorna SUCCESS.
    Si alguno falla, retorna FAILURE inmediatamente.
    """

    def __init__(self, name="Sequence"):
        """Inicializa el nodo secuencia.

        Args:
            name (str): Nombre del nodo secuencia.
        """
        super().__init__(name)
        self.current_child_idx = 0

    def _process(self, blackboard):
        """Procesa los nodos hijos secuencialmente.

        Args:
            blackboard: Objeto con datos compartidos del entorno.

        Returns:
            NodeStatus: SUCCESS si todos los hijos tienen éxito,
                       FAILURE si alguno falla, RUNNING si alguno está ejecutándose.
        """
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
    """Nodo selector que ejecuta hijos hasta que uno tenga éxito.

    Ejecuta los nodos hijos en orden hasta que uno tenga éxito.
    Si todos fallan, retorna FAILURE.
    """

    def __init__(self, name="Selector"):
        """Inicializa el nodo selector.

        Args:
            name (str): Nombre del nodo selector.
        """
        super().__init__(name)
        self.current_child_idx = 0

    def _process(self, blackboard):
        """Procesa los nodos hijos hasta encontrar uno exitoso.

        Args:
            blackboard: Objeto con datos compartidos del entorno.

        Returns:
            NodeStatus: SUCCESS si algún hijo tiene éxito,
                       FAILURE si todos fallan, RUNNING si alguno está ejecutándose.
        """
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
    """Nodo paralelo que ejecuta todos los hijos simultáneamente.

    Ejecuta todos los hijos en paralelo. La política de éxito/fallo
    se define por el número mínimo de éxitos requeridos.
    """

    def __init__(self, name="Parallel", success_threshold=None, failure_threshold=None):
        """Inicializa el nodo paralelo.

        Args:
            name (str): Nombre del nodo paralelo.
            success_threshold (int): Número mínimo de hijos que deben tener éxito.
            failure_threshold (int, optional): Número máximo de fallos permitidos.
        """
        super().__init__(name)
        self.success_threshold = success_threshold
        self.failure_threshold = failure_threshold

    def _process(self, blackboard):
        """Procesa todos los nodos hijos en paralelo.

        Args:
            blackboard: Objeto con datos compartidos del entorno.

        Returns:
            NodeStatus: Basado en los umbrales de éxito/fallo configurados.
        """
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
        success_threshold = self.success_threshold if self.success_threshold is not None \
                                                    else len(self.children)
        failure_threshold = self.failure_threshold if self.failure_threshold is not None \
                                                    else len(self.children)

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
        """Inicializa el nodo decorador.

        Args:
            child (Node): Nodo hijo.
            name (str): Nombre del nodo decorador.
        """
        super().__init__(name)
        self.child = None
        if child:
            self.set_child(child)

    def _process(self, blackboard):
        """Process the composite node logic."""
        raise NotImplementedError("Subclasses must implement _process")

    def set_child(self, child):
        """Establece el nodo hijo del decorador.

        Args:
            child (BehaviorNode): Nodo hijo a decorar.
        """
        self.child = child
        child.parent = self
        return self  # Para permitir encadenamiento


class InverterNode(DecoratorNode):
    """Invierte el resultado del hijo: éxito -> fracaso, fracaso -> éxito."""

    def __init__(self, child=None, name="Inverter"):
        """Inicializa el nodo inversor.

        Args:
            child (Node): Nodo hijo.
            name (str): Nombre del nodo inversor.
        """
        super().__init__(child, name)

    def _process(self, blackboard):
        """Invierte el resultado del nodo hijo.

        Args:
            blackboard: Objeto con datos compartidos del entorno.

        Returns:
            NodeStatus: Resultado invertido del hijo.
        """
        if not self.child:
            return NodeStatus.FAILURE

        status = self.child.tick(blackboard)

        if status == NodeStatus.SUCCESS:
            return NodeStatus.FAILURE
        if status == NodeStatus.FAILURE:
            return NodeStatus.SUCCESS

        return status  # RUNNING o INVALID se mantienen igual


class RepeatNode(DecoratorNode):
    """Decorador que repite la ejecución del hijo."""

    def __init__(self, child=None, num_repeats=None, name="Repeat"):
        """Inicializa el nodo repetidor.

        Args:
            child (Node): Nodo hijo.
            num_repeats (int): Número de repeticiones.
            name (str): Nombre del nodo repetidor.
        """
        super().__init__(child, name)
        self.num_repeats = num_repeats
        self.current_repeats = 0

    def _process(self, blackboard):
        """Repite la ejecución del nodo hijo.

        Args:
            blackboard: Objeto con datos compartidos del entorno.

        Returns:
            NodeStatus: RUNNING mientras no se alcance el límite de repeticiones.
        """
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
    """Nodo hoja que evalúa una condición.

    Representa una condición booleana que puede ser evaluada.
    """

    def __init__(self, condition_func, name="Condition"):
        """Inicializa el nodo condición.

        Args:
            name (str): Nombre del nodo condición.
            condition_func (callable): Función que evalúa la condición.
        """
        super().__init__(name)
        self.condition_func = condition_func

    def _process(self, blackboard):
        """Evalúa la condición.

        Args:
            blackboard: Objeto con datos compartidos del entorno.

        Returns:
            NodeStatus: SUCCESS si la condición es verdadera, FAILURE si es falsa.
        """
        # Evaluar la condición y registrarla
        result = self.condition_func(blackboard)
        global_tracer.add_condition_met(self.name, result)

        if self.condition_func(blackboard):
            return NodeStatus.SUCCESS
        return NodeStatus.FAILURE


class ActionNode(BehaviorNode):
    """Nodo hoja que ejecuta una acción.

    Representa una acción específica que puede ser ejecutada.
    """

    def __init__(self, action_func, name="Action"):
        """Inicializa el nodo acción.

        Args:
            name (str): Nombre del nodo acción.
            action_func (callable): Función que ejecuta la acción.
        """
        super().__init__(name)
        self.action_func = action_func

    def _process(self, blackboard):
        """Ejecuta la acción.

        Args:
            blackboard: Objeto con datos compartidos del entorno.

        Returns:
            NodeStatus: Resultado de la ejecución de la acción.
        """
        # Ejecutar la acción
        status = self.action_func(blackboard)

        # Si la acción está en ejecución o tuvo éxito, registrarla como próxima acción (NUEVO)
        if status != NodeStatus.FAILURE:
            global_tracer.set_next_action(self.name)

        return status


class StatefulActionNode(BehaviorNode):
    """Nodo acción con ciclo de vida stateful (patrón BehaviorTree.CPP).

    A diferencia de ActionNode (que llama action_func en cada tick),
    StatefulActionNode distingue entre dos fases:

      on_start_func(blackboard)   -> llamada UNA SOLA VEZ al activarse.
                                     Aquí se emite el comando RF/PID.
                                     Debe retornar NodeStatus.RUNNING normalmente.

      on_running_func(blackboard) -> llamada en cada tick subsecuente mientras
                                     el nodo esté RUNNING. Solo monitorea
                                     completación; NO reemite el comando.
                                     Retorna RUNNING, SUCCESS o FAILURE.

    Esto evita que el nodo reemita comandos en cada tick (cada 10-100 ms),
    dando tiempo al PID de completar rotaciones/movimientos entre decisiones
    del árbol de comportamiento.

    Referencia: BehaviorTree.CPP StatefulActionNode
    (https://www.behaviortree.dev/docs/tutorial-advanced/asynchronous_nodes/)
    """

    def __init__(self, on_start_func, on_running_func=None, name="StatefulAction"):
        """Inicializa el nodo acción stateful.

        Args:
            on_start_func (callable): Función llamada una vez al activarse.
                Signature: (blackboard) -> NodeStatus
            on_running_func (callable, optional): Función llamada cada tick
                mientras RUNNING. Si None, siempre retorna RUNNING.
                Signature: (blackboard) -> NodeStatus
            name (str): Nombre del nodo.
        """
        super().__init__(name)
        self._on_start_func = on_start_func
        self._on_running_func = on_running_func or (lambda bb: NodeStatus.RUNNING)
        self._started = False

    def tick(self, blackboard):
        """Ejecuta el nodo con ciclo de vida stateful.

        Llama on_start_func solo en la primera activación.
        Llama on_running_func en cada tick subsecuente mientras RUNNING.
        """
        if not self._started or self.status != NodeStatus.RUNNING:
            # Primera activación o reinicio después de completar
            log.info("[StatefulAction:%s] on_start → emitiendo comando (primera vez o reinicio)", self.name)
            self._started = True
            result = self._on_start_func(blackboard)
        else:
            # Ya estaba RUNNING: solo monitorear, no reemitir comando
            log.debug("[StatefulAction:%s] on_running → monitoreando completación", self.name)
            result = self._on_running_func(blackboard)

        if result != NodeStatus.RUNNING:
            # Completado (SUCCESS o FAILURE): resetear para próxima activación
            self._started = False

        self.status = result

        if result != NodeStatus.FAILURE:
            global_tracer.set_next_action(self.name)
        global_tracer.add_node_result(self.name, 'StatefulActionNode', result)

        return result

    def _process(self, blackboard):
        # No se usa — tick() se sobreescribe directamente
        return self._on_start_func(blackboard)


def get_global_tracer():
    """Obtiene la instancia global del tracer.

    Returns:
        BehaviorTreeTracer: Instancia global del tracer.
    """
    return global_tracer
