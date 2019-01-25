"""
This module is used to model the architectural consequences of particular
ROS commands (e.g., launching a given :code:`.launch` file via
:code:`roslaunch`).

The main class within this module is :class:`Interpreter`, which acts as a
model evaluator / virtual machine for a ROS architecture.
"""
from typing import Dict, Iterator, Any, Optional, Tuple, Callable, Set, FrozenSet
import logging

import attr
import roslaunch  # FIXME try to lose this dependency!

from ..workspace import Workspace

logger = logging.getLogger(__name__)  # type: logging.Logger
logger.setLevel(logging.DEBUG)

FullName = str


class ParameterServer(object):
    def __init__(self):
        # type: () -> None
        self.__contents = {}  # type: Dict[str, Any]

    def __getitem__(self, key):
        # type: (str) -> Any
        return self.__contents[key]

    def __contains__(self, key):
        # type: (str) -> bool
        return key in self.__contents

    def __setitem__(self, key, val):
        # type: (str, Any) -> None
        self.__contents[key] = val

    def get(self, key, default):
        # type: (str, Any) -> Any
        return self.__contents.get(key, default)


@attr.s(frozen=True)
class NodeSummary(object):
    name = attr.ib(type=str)
    fullname = attr.ib(type=str)
    namespace = attr.ib(type=str)
    kind = attr.ib(type=str)
    package = attr.ib(type=str)
    pubs = attr.ib(type=FrozenSet[Tuple[FullName, str]],
                   converter=frozenset)
    subs = attr.ib(type=FrozenSet[Tuple[FullName, str]],
                   converter=frozenset)
    reads = attr.ib(type=FrozenSet[FullName],
                    converter=frozenset)
    writes = attr.ib(type=FrozenSet[FullName],
                    converter=frozenset)

    def to_dict(self):
        # type: () -> Dict[str, Any]
        pubs = [{'name': str(n), 'format': str(f)} for (n, f) in self.pubs]
        subs = [{'name': str(n), 'format': str(f)} for (n, f) in self.subs]
        return {'name': str(self.name),
                'fullname': str(self.fullname),
                'namespace': str(self.namespace),
                'kind': str(self.kind),
                'package': str(self.package),
                'reads': list(self.reads),
                'writes': list(self.writes),
                'pubs': pubs,
                'subs': subs}


class NodeContext(object):
    def __init__(self,
                 name,          # type: str
                 namespace,     # type: str
                 kind,          # type: str
                 package,       # type: str
                 remappings,    # type: Dict[str, str]
                 params         # type: ParameterServer
                 ):             # type: (...) -> None
        self.__name = name
        self.__namespace = namespace
        self.__kind = kind
        self.__package = package
        self.__params = params
        self.__subs = set()  # type: Set[Tuple[str, str]]
        self.__pubs = set()  # type: Set[Tuple[str, str]]
        self.__reads = set()  # type: Set[str]
        self.__writes = set()  # type: Set[str]

        self.__remappings = {
            self.resolve(x): self.resolve(y)
            for (x, y) in remappings.items()
        }  # type: Dict[str, str]

    @property
    def fullname(self):
        # type: () -> str
        ns = self.__namespace
        if ns[-1] != '/':
            ns += ' /'
        return '{}{}'.format(ns, self.__name)

    def _remap(self, name):
        # type: (str) -> str
        if name in self.__remappings:
            name_new = self.__remappings[name]
            logger.info("applying remapping from [%s] to [%s]",
                        name, name_new)
            return name_new
        else:
            return name

    def summarize(self):
        # type: (...) -> NodeSummary
        return NodeSummary(name=self.__name,
                           fullname=self.fullname,
                           namespace=self.__namespace,
                           kind=self.__kind,
                           package=self.__package,
                           reads=self.__reads,
                           writes=self.__writes,
                           pubs=self.__pubs,
                           subs=self.__subs)

    def resolve(self, name):
        # type: (str) -> FullName
        """
        Resolves a given name within the context of this node.

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

    def provide(self, service, fmt):
        # type: (str, str) -> None
        """
        Instructs the node to provide a service.
        """
        logger.debug("node [%s] provides service [%s] using format [%s]",
                     self.__name, service, fmt)

    def sub(self, topic_name, fmt):
        # type: (str, str) -> None
        """
        Subscribes the node to a given topic.

        Parameters:
            topic: the unqualified name of the topic.
            fmt: the message format used by the topic.
        """
        topic_name_full = self.resolve(topic_name)
        topic_name_full = self._remap(topic_name_full)
        logger.debug("node [%s] subscribes to topic [%s] with format [%s]",
                     self.__name, topic_name, fmt)
        self.__subs.add((topic_name_full, fmt))

    def pub(self, topic_name, fmt):
        # type: (str, str) -> None
        """
        Instructs the node to publish to a given topic.

        Parameters:
            topic: the unqualified name of the topic.
            fmt: the message format used by the topic.
        """
        topic_name_full = self.resolve(topic_name)
        topic_name_full = self._remap(topic_name_full)
        logger.debug("node [%s] publishes to topic [%s] with format [%s]",
                     self.__name, topic_name, fmt)
        self.__pubs.add((topic_name_full, fmt))

    def read(self,
             param,     # type: str
             default    # type: Optional[Any]
             ):         # type: (...) -> Any
        """
        Obtains the value of a given parameter from the parameter
        server.
        """
        logger.debug("node [%s] reads parameter [%s]",
                     self.__name, param)
        param = self.resolve(param)
        return self.__params.get(param, default)

    def write(self, param, val):
        # type: (str, Any) -> None
        logger.debug("node [%s] writes [%s] to parameter [%s]",
                     self.__name, val, param)


class Model(object):
    """
    Models the architectural interactions of a node type.
    """
    _models = {}  # type: Dict[Tuple[str, str], Model]

    @staticmethod
    def register(package,       # type: str
                 name,          # type: str
                 definition     # type: Callable[[NodeContext], None]
                 ):             # type: (...) -> None
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
    def find(package, name):
        # type: (str, str) -> Model
        return Model._models[(package, name)]

    def __init__(self,
                 package,       # type: str
                 name,          # type: str
                 definition     # type: Callable[[NodeContext], None]
                 ):             # type: (...) -> None
        self.__package = package
        self.__name = name
        self.__definition = definition

    def eval(self, context):
        # type: (NodeContext) -> None
        return self.__definition(context)


def model(package, name):
    # type: (str, str) -> Any
    def register(m):
        # type: (Callable[[NodeContext], None]) -> Any
        Model.register(package, name, m)
        return m
    return register


class Interpreter(object):
    def __init__(self, workspace):
        # type: (Workspace) -> None
        self.__workspace = workspace
        self.__params = ParameterServer()
        self.__nodes = set()  # type: Set[NodeSummary]

    @property
    def parameters(self):
        """
        The simulated parameter server for this interpreter.
        """
        # type: () -> ParameterServer
        return self.__params

    @property
    def nodes(self):
        """
        Returns an iterator over the summaries for each node on the ROS graph.
        """
        # type: () -> Iterator[NodeSummary]
        for n in self.__nodes:
            yield n

    def launch(self, fn):
        # type: (str) -> None
        """
        Simulates the effects of `roslaunch` using a given launch file.
        """
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

    def create_nodelet_manager(self, name):
        # type: (str) -> None
        """
        Creates a nodelet manager with a given name.
        """
        logger.info('launched nodelet manager: %s', name)

    def load_nodelet(self,
                     pkg,           # type: str
                     nodetype,      # type: str
                     name,          # type: str
                     namespace,     # type: str
                     remappings,    # type: Dict[str, str]
                     manager        # type: str
                     ):             # type: (...) -> None
        """
        Loads a nodelet using the provided instructions.

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
             pkg,           # type: str
             nodetype,      # type: str
             name,          # type: str
             namespace,     # type: str
             remappings,    # type: Dict[str, str]
             args           # type: str
             ):             # type: (...) -> None
        """
        Loads a node using the provided instructions.

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
