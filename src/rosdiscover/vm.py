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
    fullname = attr.ib(type=str)
    namespace = attr.ib(type=str)
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
                'fullname': str(self.fullname),
                'namespace': str(self.namespace),
                'kind': str(self.kind),
                'package': str(self.package),
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

    def create_nodelet_manager(self, name):
        # type: (str) -> None
        logger.info('launched nodelet manager: %s', name)

    def load_nodelet(self,
                     pkg,           # type: str
                     nodetype,      # type: str
                     name,          # type: str
                     namespace,     # type: str
                     remappings,    # type: Dict[str, str]
                     manager        # type: str
                     ):             # type: (...) -> None
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
