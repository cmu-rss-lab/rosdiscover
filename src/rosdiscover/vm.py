# for now, we need to include prebaked modules
# - a node may be composed of multiple components
from typing import Dict, Iterator, Any, Optional, Tuple, Callable, Set, FrozenSet
import logging

import attr
import roslaunch

from .workspace import Workspace

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


@attr.s(frozen=True)
class NodeSummary(object):
    name = attr.ib(type=str)
    kind = attr.ib(type=str)
    package = attr.ib(type=str)
    pubs = attr.ib(type=FrozenSet[Tuple[FullName, str]],
                   converter=frozenset)
    subs = attr.ib(type=FrozenSet[Tuple[FullName, str]],
                   converter=frozenset)

    def to_dict(self):
        # type: () -> Dict[str, Any]
        pubs = [{'name': str(n), 'format': str(f)} for (n, f) in self.pubs]
        subs = [{'name': str(n), 'format': str(f)} for (n, f) in self.subs]
        return {'name': str(self.name),
                'kind': str(self.kind),
                'package': str(self.package),
                'pubs': pubs,
                'subs': subs}


class NodeContext(object):
    def __init__(self,
                 name,      # type: str
                 kind,      # type: str
                 package,   # type: str
                 params     # type: ParameterServer
                 ):         # type: (...) -> None
        self.__name = name
        self.__kind = kind
        self.__package = package
        self.__params = params
        self.__subs = set()  # type: Set[Tuple[str, str]]
        self.__pubs = set()  # type: Set[Tuple[str, str]]

    def summarize(self):
        # type: (...) -> NodeSummary
        return NodeSummary(name=self.__name,
                           kind=self.__kind,
                           package=self.__package,
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
            return '/{}/{}'.format(self.__name, name)
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

        # FIXME
        return default

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


class VM(object):
    def __init__(self, workspace):
        # type: (Workspace) -> None
        self.__workspace = workspace
        self.__params = ParameterServer()
        self.__nodes = set()  # type: Set[NodeSummary]

    @property
    def parameters(self):
        # type: () -> ParameterServer
        return self.__params

    @property
    def nodes(self):
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

        for node in config.nodes:
            logger.debug("launching node: %s", node.name)
            try:
                self.load(pkg=node.package,
                          nodetype=node.type,
                          name=node.name,
                          namespace=node.namespace)  # FIXME
            except Exception:
                logger.exception("failed to launch node: %s", node.name)

    def load(self,
             pkg,       # type: str
             nodetype,  # type: str
             name,      # type: str
             namespace  # type: str
             ):         # type: (...) -> None
        if nodetype == 'nodelet':
            raise Exception('nodelets are not currently supported.')

        try:
            model = Model.find(pkg, nodetype)
        except Exception:
            m = "failed to find model for node type [{}] in package [{}]"
            m = m.format(nodetype, pkg)
            raise Exception(m)

        ctx = NodeContext(name=name,
                          kind=nodetype,
                          package=pkg,
                          params=self.__params)
        model.eval(ctx)
        self.__nodes.add(ctx.summarize())
