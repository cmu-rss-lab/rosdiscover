# -*- coding: utf-8 -*-
import re
import typing
from typing import Any, List, Mapping, Optional, Set, Tuple, Union

import attr
import dockerblade
import roswire.name as rosname
from loguru import logger
from roswire import AppInstance, ROSDistribution, ROSVersion

from .parameter import ParameterServer
from .provenance import Provenance
from .summary import NodeSummary
from ..core import Action, Service, Topic

if typing.TYPE_CHECKING:
    from .plugin import ModelPlugin
    from ..recover.symbolic import SymbolicUnknown

UNKNOWN_NAME = "\\unknown"


@attr.s(slots=True, auto_attribs=True)
class NodeContext:
    name: str
    namespace: str
    kind: str
    package: str
    args: str
    remappings: Mapping[str, str]
    launch_filename: str
    app: "AppInstance" = attr.ib(repr=False)
    _params: ParameterServer = attr.ib(repr=False)
    _files: dockerblade.files.FileSystem = attr.ib(repr=False)
    _nodelet: bool = attr.ib(default=False, repr=False)
    _provenance: "Provenance" = attr.ib(default=Provenance.UNKNOWN, repr=False)
    _uses: Set[Service] = attr.ib(factory=set, repr=False)
    _provides: Set[Service] = attr.ib(factory=set, repr=False)
    _subs: Set[Topic] = attr.ib(factory=set, repr=False)
    _pubs: Set[Topic] = attr.ib(factory=set, repr=False)
    _action_servers: Set[Action] = attr.ib(factory=set, repr=False)
    _action_clients: Set[Action] = attr.ib(factory=set, repr=False)
    # The tuple is (name, dynamic) where name is the name of the parameter
    # and dynamic is whether the node reacts to updates to the parameter via reconfigure
    _reads: Set[Tuple[str, bool]] = attr.ib(factory=set, repr=False)
    _writes: Set[str] = attr.ib(factory=set, repr=False)
    _plugins: List['ModelPlugin'] = attr.ib(factory=list)

    def merge(self, context: 'NodeContext') -> None:
        self._params.update(context._params)
        self._uses.update(context._uses)
        self._provides.update(context._provides)
        self._subs.update(context._subs)
        self._pubs.update(context._pubs)
        self._action_servers.update(context._action_servers)
        self._action_clients.update(context._action_clients)
        self._reads.update(context._reads)
        self._writes.update(context._writes)

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

    @property
    def ros_distro(self) -> ROSDistribution:
        return self.app.description.distribution

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
                           provenance=self._provenance,
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
        # replace multiple /'s with a single /
        name = re.sub(r'/+', '/', name)
        # global
        if name[0] == '/':
            return name
        # private
        elif name[0] == '~':
            return f'{self.fullname}/{name[1:]}'
        # relative and base names
        else:
            return rosname.namespace_join(self.namespace, name)

    def resolve(self, name: Union[str, 'SymbolicUnknown']) -> str:
        """Resolves a given name within the context of this node.

        Returns
        -------
        str
            The fully qualified form of a given name.

        References
        ----------
        * http://wiki.ros.org/Names
        """
        if isinstance(name, str):
            name = self._resolve_without_remapping(name)
            return self._apply_remappings(name)
        else:
            logger.warning(f"Unable to resolve unknown name in NodeContext [{self.name}]")
            return UNKNOWN_NAME

    def _name_str(self, name: Union[str, 'SymbolicUnknown']) -> str:
        if isinstance(name, str):
            return name
        return "Unknown Symbol"

    def provide(self, service: Union[str, 'SymbolicUnknown'], fmt: str) -> None:
        """Instructs the node to provide a service."""
        logger.debug(f"node [{self.name}] provides service [{self._name_str(service)}] "
                     f"using format [{fmt}]")
        service_name_full = self.resolve(service)
        self._provides.add(Service(name=service_name_full, format=fmt))

    def use(self, service: Union[str, 'SymbolicUnknown'], fmt: str) -> None:
        """Instructs the node to use a given service."""
        logger.debug(f"node [{self.name}] uses a service [{self._name_str(service)}] "
                     f"with format [{fmt}]")
        service_name_full = self.resolve(service)
        self._uses.add(Service(name=service_name_full, format=fmt))

    def sub(self, topic_name: Union[str, 'SymbolicUnknown'], fmt: str, implicit: bool = False) -> None:
        """Subscribes the node to a given topic.

        Parameters
        ----------
        topic_name: str
            The unqualified name of the topic.
        fmt: str
            The name of message format used by the topic.
        implicit: bool
            the topic is implicit (e.g., generated by action server or client)
            and probably shouldn't be subscribed to directly
        """
        topic_name_full = self.resolve(topic_name)
        logger.debug(f"node [{self.name}] subscribes to topic "
                     f"[{self._name_str(topic_name)}] with format [{fmt}]")
        self._subs.add(Topic(name=topic_name_full, format=fmt, implicit=implicit))

    def pub(self, topic_name: Union[str, 'SymbolicUnknown'], fmt: str, implicit: bool = False) -> None:
        """Instructs the node to publish to a given topic.

        Parameters
        ----------
        topic_name: str
            the unqualified name of the topic.
        fmt: str
            the message format used by the topic.
        implicit: bool
            the topic is implicit (e.g., generated by action server or client)
            and probably shouldn't be subscribed to directly
        """
        topic_name_full = self.resolve(topic_name)
        logger.debug(f"node [{self.name}] publishes to topic "
                     f"[{self._name_str(topic_name)}] with format [{fmt}]")
        self._pubs.add(Topic(name=topic_name_full, format=fmt, implicit=implicit))

    def read(self,
             param: Union[str, 'SymbolicUnknown'],
             default: Optional[Any] = None,
             dynamic: bool = False
             ) -> Any:
        """Obtains the value of a given parameter from the parameter server."""
        logger.debug(f"node [{self.name}] reads parameter [{self._name_str(param)}]")
        param = self.resolve(param)
        self._reads.add((param, dynamic))
        return self._params.get(param, default)

    def write(self, param: Union[str, 'SymbolicUnknown'], val: Any) -> None:
        logger.debug(f"node [{self.name}] writes [{val}] to "
                     f"parameter [{self._name_str(param)}]")
        param = self.resolve(param)
        self._writes.add(param)
        self._params[param] = val

    # FIXME we _may_ want to record this interaction in our summary
    def has_param(self, param: Union[str, 'SymbolicUnknown']) -> bool:
        """Determines whether a given parameter has been defined."""
        logger.debug(f"node [{self.name}] checks for existence of parameter [{param}]")
        param = self.resolve(param)
        return param in self._params

    def delete_param(self, param: Union[str, 'SymbolicUnknown']) -> None:
        raise NotImplementedError("parameter deletion is not implemented")

    def read_file(self, fn: Union[str, 'SymbolicUnknown']) -> str:
        """Reads the contents of a text file."""
        if isinstance(fn, str):
            if not self._files.exists(fn):
                message = f"'{fn}' does not exist"
                logger.error(message)
                raise FileNotFoundError(message)
            return self._files.read(fn)
        logger.warning(f"Unable to resolve unknown parameter filename in NodeContext [{self.name}]")
        return UNKNOWN_NAME

    def parameter_keys(self, prefix: str) -> typing.Iterable[str]:
        prefix = self.resolve(prefix)
        return (key for key in self._params.keys() if key.startswith(prefix))

    def action_server(self, ns: Union[str, 'SymbolicUnknown'], fmt: str) -> None:
        """Creates a new action server.

        Parameters
        ----------
        ns: str
            the namespace of the action server.
        fmt: str
            the name of the action format used by the server.
        """
        logger.debug(f"node [{self.name}] provides action server "
                     f"[{self._name_str(ns)}] with format [{fmt}]")
        ns = self.resolve(ns)
        self._action_servers.add(Action(name=ns, format=fmt))
        if self.actions_have_topics():
            # Topics are implicit because they are created by the action server
            # and are only really intended for interaction between the
            # action client and action server.
            self.sub(f'{ns}/goal', f'{fmt}Goal', implicit=True)
            self.sub(f'{ns}/cancel', 'actionlib_msgs/GoalID', implicit=True)
            self.pub(f'{ns}/status', 'actionlib_msgs/GoalStatusArray', implicit=True)
            self.pub(f'{ns}/feedback', f'{fmt}Feedback', implicit=True)
            self.pub(f'{ns}/result', f'{fmt}Result', implicit=True)

    def action_client(self, ns: Union[str, 'SymbolicUnknown'], fmt: str) -> None:
        """Creates a new action client.

        Parameters
        ----------
        ns: str
            the namespace of the corresponding action server.
        fmt: str
            the name of the action format used by the server.
        """
        logger.debug(f"node [{self.name}] provides action client "
                     f"[{self._name_str(ns)}] with format [{fmt}]")
        ns = self.resolve(ns)
        self._action_clients.add(Action(name=ns, format=fmt))

        if self.actions_have_topics():
            # Topics are implicit because they are created by the action client
            # and are only really intended for interaction between the
            # action client and action server.
            self.pub(f'{ns}/goal', f'{fmt}Goal', implicit=True)
            self.pub(f'{ns}/cancel', 'actionlib_msgs/GoalID', implicit=True)
            self.sub(f'{ns}/status', 'actionlib_msgs/GoalStatusArray', implicit=True)
            self.sub(f'{ns}/feedback', f'{fmt}Feedback', implicit=True)
            self.sub(f'{ns}/result', f'{fmt}Result', implicit=True)

    def actions_have_topics(self):
        distribution = self.app.description.distribution
        if distribution.ros == ROSVersion.ROS1:
            return True
        else:
            return distribution < ROSDistribution.FOXY

    def load_nodelet(self, nodelet_context: 'NodeContext'):
        self.merge(nodelet_context)
        # In the recovered architecture, the nodelets themselves don't
        # report what they publish etc.
        # TODO: Fix this when we have NodeletManagerContexts and NodeletContexts
        nodelet_context._params.clear()
        nodelet_context._uses.clear()
        nodelet_context._provides.clear()
        nodelet_context._subs.clear()
        nodelet_context._pubs.clear()
        nodelet_context._action_servers.clear()
        nodelet_context._action_clients.clear()
        nodelet_context._reads.clear()
        nodelet_context._writes.clear()

    def load_plugin(self, plugin: 'ModelPlugin') -> None:
        """Loads a given dynamic plugin."""
        logger.debug(f'loading plugin in node [{self.name}]: {plugin}')
        self._plugins.append(plugin)

    def mark_nodelet(self) -> None:
        self._nodelet = True

    def mark_placeholder(self) -> None:
        self._provenance = Provenance.PLACEHOLDER

    def mark_handwritten(self) -> None:
        self._provenance = Provenance.HANDWRITTEN

    def mark_recovered(self) -> None:
        self._provenance = Provenance.RECOVERED
