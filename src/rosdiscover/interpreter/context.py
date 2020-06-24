# -*- coding: utf-8 -*-
from typing import Any, List, Mapping, Optional, Set, Tuple

from loguru import logger
import attr
import dockerblade
import roswire.name as rosname
import typing

from .summary import NodeSummary
from .parameter import ParameterServer

if typing.TYPE_CHECKING:
    from .plugin import ModelPlugin


@attr.s(slots=True, auto_attribs=True)
class NodeContext:
    name: str
    namespace: str
    kind: str
    package: str
    args: str
    remappings: Mapping[str, str]
    launch_filename: str
    _params: ParameterServer = attr.ib(repr=False)
    _files: dockerblade.files.FileSystem = attr.ib(repr=False)
    _nodelet: bool = attr.ib(default=False, repr=False)
    _placeholder: bool = attr.ib(default=False, repr=False)
    _uses: Set[Tuple[str, str]] = attr.ib(factory=set, repr=False)
    _provides: Set[Tuple[str, str]] = attr.ib(factory=set, repr=False)
    _subs: Set[Tuple[str, str]] = attr.ib(factory=set, repr=False)
    _pubs: Set[Tuple[str, str]] = attr.ib(factory=set, repr=False)
    _action_servers: Set[Tuple[str, str]] = attr.ib(factory=set, repr=False)
    _action_clients: Set[Tuple[str, str]] = attr.ib(factory=set, repr=False)
    # The tuple is (name, dynamic) where name is the name of the parameter
    # and dynamic is whether the node reacts to updates to the parameter via reconfigure
    _reads: Set[Tuple[str, bool]] = attr.ib(factory=set, repr=False)
    _writes: Set[str] = attr.ib(factory=set, repr=False)
    _plugins: List['ModelPlugin'] = attr.ib(factory=list)

    def __attrs_post_init__(self) -> None:
        assert rosname.name_is_legal(self.namespace)
        self.namespace = rosname.global_name(self.namespace)
        self.remappings = {
            self._resolve_without_remapping(x):
            self._resolve_without_remapping(y)
            for (x, y) in self.remappings.items()
        }

    @property
    def fullname(self) -> str:
        ns = self.namespace
        if ns[-1] != '/':
            ns += ' /'
        return f'{ns}{self.name}'

    def _apply_remappings(self, name: str) -> str:
        """Applies any appropriate remappings to a fully qualified name."""
        for remap_from in sorted(self.remappings):
            remap_to = self.remappings[remap_from]
            if name.startswith(remap_from):
                name_new = name.replace(remap_from, remap_to, 1)
                logger.info(f"applying remapping from [{name}] to [{name_new}]")
                name = name_new
        return name

    def summarise(self) -> NodeSummary:
        return NodeSummary(name=self.name,
                           fullname=self.fullname,
                           filename=self.launch_filename,
                           namespace=self.namespace,
                           kind=self.kind,
                           package=self.package,
                           nodelet=self._nodelet,
                           placeholder=self._placeholder,
                           reads=self._reads,
                           writes=self._writes,
                           pubs=self._pubs,
                           subs=self._subs,
                           provides=self._provides,
                           uses=self._uses,
                           action_servers=self._action_servers,
                           action_clients=self._action_clients)

    def _resolve_without_remapping(self, name: str) -> str:
        """Resolves a given name to a global name, without applying
        any remappings, within the context of this node."""
        # global
        if name[0] == '/':
            return name
        # private
        elif name[0] == '~':
            return f'{self.fullname}/{name[1:]}'
        # relative and base names
        else:
            return rosname.namespace_join(self.namespace, name)

    def resolve(self, name: str) -> str:
        """Resolves a given name within the context of this node.

        Returns
        -------
        str
            The fully qualified form of a given name.

        References
        ----------
        * http://wiki.ros.org/Names
        """
        name = self._resolve_without_remapping(name)
        return self._apply_remappings(name)

    def provide(self, service: str, fmt: str) -> None:
        """Instructs the node to provide a service."""
        logger.debug(f"node [{self.name}] provides service [{service}] "
                     f"using format [{fmt}]")
        service_name_full = self.resolve(service)
        self._provides.add((service_name_full, fmt))

    def use(self, service: str, fmt: str) -> None:
        """Instructs the node to use a given service."""
        logger.debug(f"node [{self.name}] uses a service [{service}] "
                     f"with format [{fmt}]")
        service_name_full = self.resolve(service)
        self._uses.add((service_name_full, fmt))

    def sub(self, topic_name: str, fmt: str) -> None:
        """Subscribes the node to a given topic.

        Parameters
        ----------
        topic_name: str
            The unqualified name of the topic.
        fmt: str
            The name of message format used by the topic.
        """
        topic_name_full = self.resolve(topic_name)
        logger.debug(f"node [{self.name}] subscribes to topic "
                     f"[{topic_name}] with format [{fmt}]")
        self._subs.add((topic_name_full, fmt))

    def pub(self, topic_name: str, fmt: str) -> None:
        """Instructs the node to publish to a given topic.

        Parameters
        ----------
        topic_name: str
            the unqualified name of the topic.
        fmt: str
            the message format used by the topic.
        """
        topic_name_full = self.resolve(topic_name)
        logger.debug(f"node [{self.name}] publishes to topic "
                     f"[{topic_name}] with format [{fmt}]")
        self._pubs.add((topic_name_full, fmt))

    def read(self,
             param: str,
             default: Optional[Any] = None,
             dynamic: bool = False
             ) -> Any:
        """Obtains the value of a given parameter from the parameter server."""
        logger.debug(f"node [{self.name}] reads parameter [{param}]")
        param = self.resolve(param)
        self._reads.add((param, dynamic))
        return self._params.get(param, default)

    def write(self, param: str, val: Any) -> None:
        logger.debug(f"node [{self.name}] writes [{val}] to "
                     f"parameter [{param}]")
        param = self.resolve(param)
        self._writes.add(param)

    def read_file(self, fn: str) -> str:
        """Reads the contents of a text file."""
        return self._files.read(fn)

    def action_server(self, ns: str, fmt: str) -> None:
        """Creates a new action server.

        Parameters
        ----------
        ns: str
            the namespace of the action server.
        fmt: str
            the name of the action format used by the server.
        """
        logger.debug(f"node [{self.name}] provides action server "
                     f"[{ns}] with format [{fmt}]")
        ns = self.resolve(ns)
        self._action_servers.add((ns, fmt))

        self.sub(f'{ns}/goal', f'{fmt}Goal')
        self.sub(f'{ns}/cancel', 'actionlib_msgs/GoalID')
        self.pub(f'{ns}/status', 'actionlib_msgs/GoalStatusArray')
        self.pub(f'{ns}/feedback', f'{fmt}Feedback')
        self.pub(f'{ns}/result', f'{fmt}Result')

    def action_client(self, ns: str, fmt: str) -> None:
        """Creates a new action client.

        Parameters
        ----------
        ns: str
            the namespace of the corresponding action server.
        fmt: str
            the name of the action format used by the server.
        """
        logger.debug(f"node [{self.name}] provides action client "
                     f"[{ns}] with format [{fmt}]")
        ns = self.resolve(ns)
        self._action_clients.add((ns, fmt))

        self.pub(f'{ns}/goal', f'{fmt}Goal')
        self.pub(f'{ns}/cancel', 'actionlib_msgs/GoalID')
        self.sub(f'{ns}/status', 'actionlib_msgs/GoalStatusArray')
        self.sub(f'{ns}/feedback', f'{fmt}Feedback')
        self.sub(f'{ns}/result', f'{fmt}Result')

    def load_plugin(self, plugin: 'ModelPlugin') -> None:
        """Loads a given dynamic plugin."""
        logger.debug(f'loading plugin in node [{self.name}]: {plugin}')
        self._plugins.append(plugin)

    def mark_nodelet(self) -> None:
        self._nodelet = True

    def mark_placeholder(self) -> None:
        self._placeholder = True
