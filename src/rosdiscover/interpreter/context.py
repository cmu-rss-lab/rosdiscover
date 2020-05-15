# -*- coding: utf-8 -*-
from typing import Dict, Any, Optional, Tuple, Set

from loguru import logger
import dockerblade
import roswire.name as rosname

from .summary import NodeSummary
from .parameter import ParameterServer


class NodeContext:
    def __init__(self,
                 name: str,
                 namespace: str,
                 kind: str,
                 package: str,
                 args: str,
                 remappings: Dict[str, str],
                 params: ParameterServer,
                 files: dockerblade.files.FileSystem,
                 ) -> None:
        assert rosname.name_is_legal(namespace)
        namespace = rosname.global_name(namespace)

        self.__name = name
        self.__namespace = namespace
        self.__kind = kind
        self.__package = package
        self.__params = params
        self.__files = files
        self.__args = args
        self.__nodelet: bool = False
        self.__uses: Set[Tuple[str, str]] = set()
        self.__provides: Set[Tuple[str, str]] = set()
        self.__subs: Set[Tuple[str, str]] = set()
        self.__pubs: Set[Tuple[str, str]] = set()

        self.__action_servers: Set[Tuple[str, str]] = set()
        self.__action_clients: Set[Tuple[str, str]] = set()

        # The tuple is (name, dynamic) where name is the name of the parameter
        # and dynamic is whether the node reacts to updates to the parameter via reconfigure
        self.__reads: Set[Tuple[str, bool]] = set()
        self.__writes: Set[str] = set()
        self.__placeholder : bool = False

        self.__remappings: Dict[str, str] = {
            self.resolve(x): self.resolve(y)
            for (x, y) in remappings.items()
        }

    @property
    def args(self) -> str:
        return self.__args

    @property
    def fullname(self) -> str:
        ns = self.__namespace
        if ns[-1] != '/':
            ns += ' /'
        return f'{ns}{self.__name}'

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
                           nodelet=self.__nodelet,
                           reads=self.__reads,
                           writes=self.__writes,
                           pubs=self.__pubs,
                           subs=self.__subs,
                           provides=self.__provides,
                           uses=self.__uses,
                           action_servers=self.__action_servers,
                           action_clients=self.__action_clients)

    def resolve(self, name: str) -> str:
        """Resolves a given name within the context of this node.

        Returns
        -------
        str
            the fully qualified form of a given name.

        References
        ----------
        * http://wiki.ros.org/Names
        """
        # global
        if name[0] == '/':
            return name
        # private
        elif name[0] == '~':
            return f'{self.fullname}/{name[1:]}'
        # relative and base names
        else:
            return rosname.namespace_join(self.__namespace, name)

    def provide(self, service: str, fmt: str) -> None:
        """Instructs the node to provide a service."""
        logger.debug(f"node [{self.__name}] provides service [{service}] "
                     f"using format [{fmt}]")
        service_name_full = self.resolve(service)
        service_name_full = self._remap(service_name_full)
        self.__provides.add((service_name_full, fmt))

    def use(self, service: str, fmt: str) -> None:
        """Instructs the node to use a given service."""
        logger.debug(f"node [{self.__name}] uses a service [{service}] "
                     f"with format [{fmt}]")
        service_name_full = self.resolve(service)
        service_name_full = self._remap(service_name_full)
        self.__uses.add((service_name_full, fmt))

    def sub(self, topic_name: str, fmt: str) -> None:
        """Subscribes the node to a given topic.

        Parameters:
            topic: the unqualified name of the topic.
            fmt: the message format used by the topic.
        """
        topic_name_full = self.resolve(topic_name)
        topic_name_full = self._remap(topic_name_full)
        logger.debug(f"node [{self.__name}] subscribes to topic "
                     f"[{topic_name}] with format [{fmt}]")
        self.__subs.add((topic_name_full, fmt))

    def pub(self, topic_name: str, fmt: str) -> None:
        """Instructs the node to publish to a given topic.

        Parameters:
            topic: the unqualified name of the topic.
            fmt: the message format used by the topic.
        """
        topic_name_full = self.resolve(topic_name)
        topic_name_full = self._remap(topic_name_full)
        logger.debug(f"node [{self.__name}] publishes to topic "
                     f"[{topic_name}] with format [{fmt}]")
        self.__pubs.add((topic_name_full, fmt))

    def read(self, param: str, default: Optional[Any] = None, dynamic: Optional[bool] = False) -> None:
        """Obtains the value of a given parameter from the parameter server."""
        logger.debug(f"node [{self.__name}] reads parameter [{param}]")
        param = self.resolve(param)
        self.__reads.add((param, dynamic))
        return self.__params.get(param, default)

    def write(self, param: str, val: Any) -> None:
        logger.debug(f"node [{self.__name}] writes [{val}] to "
                     f"parameter [{param}]")
        param = self.resolve(param)
        self.__writes.add(param)

    def read_file(self, fn: str) -> str:
        """Reads the contents of a text file."""
        return self.__files.read(fn)

    def action_server(self, ns: str, fmt: str) -> None:
        """Creates a new action server.

        Parameters:
            ns: the namespace of the action server.
            fmt: the name of the action format used by the server.
        """
        logger.debug(f"node [{self.__name}] provides action server "
                     f"[{ns}] with format [{fmt}]")
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
        logger.debug(f"node [{self.__name}] provides action client "
                     f"[{ns}] with format [{fmt}]")
        ns = self.resolve(ns)
        self.__action_clients.add((ns, fmt))

        self.pub('{}/goal'.format(ns), '{}Goal'.format(fmt))
        self.pub('{}/cancel'.format(ns), 'actionlib_msgs/GoalID')
        self.sub('{}/status'.format(ns), 'actionlib_msgs/GoalStatusArray')
        self.sub('{}/feedback'.format(ns), '{}Feedback'.format(fmt))
        self.sub('{}/result'.format(ns), '{}Result'.format(fmt))

    def mark_nodelet(self):
        self.__nodelet = True

    def mark_placeholder(self):
        self.__placeholder = True
