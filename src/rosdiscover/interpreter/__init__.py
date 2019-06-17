# -*- coding: utf-8 -*-
"""
This module is used to model the architectural consequences of particular
ROS commands (e.g., launching a given :code:`.launch` file via
:code:`roslaunch`).

The main class within this module is :class:`Interpreter`, which acts as a
model evaluator / virtual machine for a ROS architecture.
"""
from typing import (Dict, Iterator, Any, Optional, Tuple, Callable, Set,
                    FrozenSet)
import logging

import attr
import roswire

from .summary import NodeSummary
from .parameter import ParameterServer

logger = logging.getLogger(__name__)  # type: logging.Logger
logger.setLevel(logging.DEBUG)

FullName = str


class NodeContext(object):
    def __init__(self,
                 name: str,
                 namespace: str,
                 kind: str,
                 package: str,
                 remappings: Dict[str, str],
                 params: ParameterServer
                 ) -> None:
        self.__name = name
        self.__namespace = namespace
        self.__kind = kind
        self.__package = package
        self.__params = params
        self.__provides = set()  # type: Set[Tuple[str, str]]
        self.__subs = set()  # type: Set[Tuple[str, str]]
        self.__pubs = set()  # type: Set[Tuple[str, str]]

        self.__action_servers = set()  # type: Set[Tuple[str, str]]

        self.__reads = set()  # type: Set[str]
        self.__writes = set()  # type: Set[str]

        self.__remappings = {
            self.resolve(x): self.resolve(y)
            for (x, y) in remappings.items()
        }  # type: Dict[str, str]

    @property
    def fullname(self) -> str:
        ns = self.__namespace
        if ns[-1] != '/':
            ns += ' /'
        return '{}{}'.format(ns, self.__name)

    def _remap(self, name: str) -> str:
        if name in self.__remappings:
            name_new = self.__remappings[name]
            logger.info("applying remapping from [%s] to [%s]",
                        name, name_new)
            return name_new
        else:
            return name

    def summarize(self) -> NodeSummary:
        return NodeSummary(name=self.__name,
                           fullname=self.fullname,
                           namespace=self.__namespace,
                           kind=self.__kind,
                           package=self.__package,
                           reads=self.__reads,
                           writes=self.__writes,
                           pubs=self.__pubs,
                           subs=self.__subs,
                           provides=self.__provides,
                           action_servers=self.__action_servers,
                           action_clients=self.__action_clients)

    def resolve(self, name: str) -> str:
        """Resolves a given name within the context of this node.

        Returns:
            the fully qualified form of a given name.
        """
        if name[0] == '/':
            return name
        elif name[0] == '~':
            return '/{}/{}'.format(self.__name, name[1:])
        # FIXME
        else:
            return '/{}'.format(name)

    def provide(self, service: str, fmt: str) -> None:
        """Instructs the node to provide a service."""
        logger.debug("node [%s] provides service [%s] using format [%s]",
                     self.__name, service, fmt)

        service_name_full = self.resolve(service)
        service_name_full = self._remap(service_name_full)
        self.__provides.add((service_name_full, fmt))

    def sub(self, topic_name: str, fmt: str) -> None:
        """Subscribes the node to a given topic.

        Parameters:
            topic: the unqualified name of the topic.
            fmt: the message format used by the topic.
        """
        topic_name_full = self.resolve(topic_name)
        topic_name_full = self._remap(topic_name_full)
        logger.debug("node [%s] subscribes to topic [%s] with format [%s]",
                     self.__name, topic_name, fmt)
        self.__subs.add((topic_name_full, fmt))

    def pub(self, topic_name: str, fmt: str) -> None:
        """Instructs the node to publish to a given topic.

        Parameters:
            topic: the unqualified name of the topic.
            fmt: the message format used by the topic.
        """
        topic_name_full = self.resolve(topic_name)
        topic_name_full = self._remap(topic_name_full)
        logger.debug("node [%s] publishes to topic [%s] with format [%s]",
                     self.__name, topic_name, fmt)
        self.__pubs.add((topic_name_full, fmt))

    def read(self, param: str, default: Optional[Any] = None) -> None:
        """Obtains the value of a given parameter from the parameter server."""
        logger.debug("node [%s] reads parameter [%s]",
                     self.__name, param)
        param = self.resolve(param)
        self.__reads.add(param)
        return self.__params.get(param, default)

    def write(self, param: str, val: Any) -> None:
        logger.debug("node [%s] writes [%s] to parameter [%s]",
                     self.__name, val, param)
        param = self.resolve(param)
        self.__writes.add(param)

    def action_server(self, ns: str, fmt: str) -> None:
        # type: (str, str) -> None
        """Creates a new action server.

        Parameters:
            ns: the namespace of the action server.
            fmt: the name of the action format used by the server.
        """
        logger.debug("node [%s] provides action server [%s] with format [%s]",
                     self.__name, ns, fmt)

        ns = self.resolve(ns)
        self.__action_servers.add((ns, fmt))

        self.sub('{}/goal'.format(ns), '{}Goal'.format(fmt))
        self.sub('{}/cancel'.format(ns), 'actionlib_msgs/GoalID')
        self.pub('{}/status'.format(ns), 'actionlib_msgs/GoalStatusArray')
        self.pub('{}/feedback'.format(ns), '{}Feedback'.format(fmt))
        self.pub('{}/result'.format(ns), '{}Result'.format(fmt))

    def action_client(self, ns: str, fmt: str) -> None:
        """Creates a new action client.

        Parameters:
            ns: the namespace of the corresponding action server.
            fmt: the name of the action format used by the server.
        """
        logger.debug("node [%s] provides action client [%s] with format [%s]",
                     self.__name, ns, fmt)

        ns = self.resolve(ns)
        self.__action_clients.add((ns, fmt))

        self.pub('{}/goal'.format(ns), '{}Goal'.format(fmt))
        self.pub('{}/cancel'.format(ns), 'actionlib_msgs/GoalID')
        self.sub('{}/status'.format(ns), 'actionlib_msgs/GoalStatusArray')
        self.sub('{}/feedback'.format(ns), '{}Feedback'.format(fmt))
        self.sub('{}/result'.format(ns), '{}Result'.format(fmt))


class Model(object):
    """Models the architectural interactions of a node type."""
    _models: Dict[Tuple[str, str], Model] = {}

    @staticmethod
    def register(package: str,
                 name: str,
                 definition: Callable[[NodeContext], None]
                 ) -> None:
        key = (package, name)
        models = Model._models
        if key in models:
            m = "model [{}] already registered for package [{}]"
            m.format(name, package)
            raise Exception(m)
        models[key] = Model(package, name, definition)
        logger.debug("registered model [%s] for package [%s]",
                     name, package)

    @staticmethod
    def find(package: str, name: str) -> Model:
        return Model._models[(package, name)]

    def __init__(self,
                 package,       # type: str
                 name,          # type: str
                 definition     # type: Callable[[NodeContext], None]
                 ):             # type: (...) -> None
        self.__package = package
        self.__name = name
        self.__definition = definition

    def eval(self, context: NodeContext) -> None:
        return self.__definition(context)


def model(package: str, name: str) -> Any:
    def register(m: Callable[[NodeContext], None]) -> Any:
        Model.register(package, name, m)
        return m
    return register


class Interpreter(object):
    # FIXME use image
    def __init__(self, workspace) -> None:
        # type: (Workspace) -> None
        self.__workspace = workspace
        self.__params = ParameterServer()
        self.__nodes: Set[NodeSummary] = set()

    @property
    def parameters(self) -> ParameterServer:
        """The simulated parameter server for this interpreter."""
        return self.__params

    @property
    def nodes(self) -> Iterator[NodeSummary]:
        """Returns an iterator of summaries for each ROS node."""
        yield from self.__nodes

    def launch(self, fn: str) -> None:
        """Simulates the effects of `roslaunch` using a given launch file."""
        config = roslaunch.config.ROSLaunchConfig()
        loader = roslaunch.xmlloader.XmlLoader()
        loader.load(fn, config)

        for param in config.params.values():
            self.__params[param.key] = param.value

        for node in config.nodes:
            logger.debug("launching node: %s", node.name)
            try:
                remappings = {str(old): str(new)
                              for (old, new) in node.remap_args}
                self.load(pkg=node.package,
                          nodetype=node.type,
                          name=node.name,
                          namespace=node.namespace,  # FIXME
                          remappings=remappings,
                          args=node.args)
            except Exception:
                logger.exception("failed to launch node: %s", node.name)
                raise

    def create_nodelet_manager(self, name: str) -> None:
        """Creates a nodelet manager with a given name."""
        logger.info('launched nodelet manager: %s', name)

    def load_nodelet(self,
                     pkg: str,
                     nodetype: str,
                     name: str,
                     namespace: str,
                     remappings: Dict[str, str],
                     manager: str
                     ) -> None:
        """Loads a nodelet using the provided instructions.

        Parameters:
            pkg: the name of the package to which the nodelet belongs.
            nodetype: the name of the type of nodelet that should be loaded.
            name: the name that should be assigned to the nodelet.
            namespace: the namespace into which the nodelet should be loaded.
            remappings: a dictionary of name remappings that should be applied
                to this nodelet, where keys correspond to old names and values
                correspond to new names.
            manager: the name of the manager for this nodelet.

        Raises:
            Exception: if there is no model for the given nodelet type.
        """
        logger.info('launching nodelet [%s] inside manager [%s]',
                    name, manager)
        return self.load(pkg, nodetype, name, namespace, remappings, '')

    def load(self,
             pkg: str,
             nodetype: str,
             name: str,
             namespace: str,
             remappings: Dict[str, str],
             args: str
             ) -> None:
        """Loads a node using the provided instructions.

        Parameters:
            pkg: the name of the package to which the node belongs.
            nodetype: the name of the type of node that should be loaded.
            name: the name that should be assigned to the node.
            namespace: the namespace into which the node should be loaded.
            remappings: a dictionary of name remappings that should be applied
                to this node, where keys correspond to old names and values
                correspond to new names.
            args: a string containing command-line arguments to the node.

        Raises:
            Exception: if there is no model for the given node type.
        """
        # logger.info("loading node: %s (%s)", name, nodetype)
        if nodetype == 'nodelet':
            if args == 'manager':
                return self.create_nodelet_manager(name)
            else:
                load, pkg_and_nodetype, mgr = args.split(' ')
                pkg, _, nodetype = pkg_and_nodetype.partition('/')
                return self.load_nodelet(pkg, nodetype, name, namespace, remappings, mgr)

        if remappings:
            logger.info("using remappings: %s", remappings)

        try:
            model = Model.find(pkg, nodetype)
        except Exception:
            m = "failed to find model for node type [{}] in package [{}]"
            m = m.format(nodetype, pkg)
            raise Exception(m)

        ctx = NodeContext(name=name,
                          namespace=namespace,
                          kind=nodetype,
                          package=pkg,
                          remappings=remappings,
                          params=self.__params)
        model.eval(ctx)
        self.__nodes.add(ctx.summarize())
