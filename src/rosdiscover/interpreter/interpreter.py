# -*- coding: utf-8 -*-
from typing import Dict, Iterator, Optional
import contextlib
import types
import typing as t

from loguru import logger
import roswire
from roswire import AppInstance, ROSVersion
from roswire.common.launch.config import NodeConfig
from roswire.ros1.launch.reader import ROS1LaunchFileReader
from roswire.ros2.launch.reader import ROS2LaunchFileReader

from .context import NodeContext
from .summary import SystemSummary
from .parameter import ParameterServer
from ..config import Config
from ..launch import Launch
from ..project import ProjectModels


class Interpreter:
    """
    Attributes
    ----------
    params: ParameterServer
        The simulated parameter server for this interpreter.
    """
    @classmethod
    @contextlib.contextmanager
    def for_config(cls,
                   config: Config
                   ) -> Iterator['Interpreter']:
        """Constructs an interpreter for a given configuration"""
        rsw = roswire.ROSWire()  # TODO don't maintain multiple instances
        with rsw.launch(config.image, config.sources, environment=config.environment) as app:
            with Interpreter(config, app) as interpreter:
                yield interpreter

    def __init__(
        self,
        config: Config,
        app: roswire.System,
    ) -> None:
        self._app = app
        self.params = ParameterServer()
        self.nodes: Dict[str, NodeContext] = {}
        self.models = ProjectModels(config, allow_recovery=True)
        self._waiting_nodelets: Dict[str, t.List[t.Tuple[str, t.Callable[[], None]]]] = {}

    def open(self) -> None:
        self.models.open()

    def close(self) -> None:
        self.models.close()

    def __enter__(self) -> "Interpreter":
        self.open()
        return self

    def __exit__(
        self,
        ex_type: t.Optional[t.Type[BaseException]],
        ex_val: t.Optional[BaseException],
        ex_tb: t.Optional[types.TracebackType],
    ) -> None:
        self.close()

    @property
    def app(self) -> AppInstance:
        return self._app

    def summarise(self) -> SystemSummary:
        """Produces an immutable description of the system architecture."""
        node_summaries = [node.summarise() for node in self.nodes.values()]
        node_to_summary = {s.fullname: s for s in node_summaries}
        return SystemSummary(node_to_summary)

    def launch(self, launch_description: Launch) -> None:
        """Simulates the effects of `roslaunch` using a given launch file."""
        # NOTE this method also supports command-line arguments
        if self._app.description.distribution.ros == ROSVersion.ROS1:
            reader = ROS1LaunchFileReader.for_app_instance(self._app)
        else:
            reader = ROS2LaunchFileReader.for_app_instance(self._app)
        logger.debug(f"get_argv: {launch_description.get_argv()}")
        config = reader.read(launch_description.filename, launch_description.get_argv())

        for param in config.params.values():
            self.params[param.name] = param.value

        def key(x: NodeConfig) -> str:
            return "z" if x.typ == "nodelet" and x.args and 'manager' in x.args else "a"
        # Sort nodes so that nodelets occur after node managers
        sorted_nodes = sorted(config.nodes, key=key)
        assert sorted_nodes[-1].typ == "nodelet" and 'manager' not in sorted_nodes[-1].args

        for node in sorted_nodes:
            if not node.filename:
                m = ("unable to determine associated launch file for "
                     f"node: {node}")
                raise Exception(m)
            logger.debug(f"launching node: {node.name} from {node.filename}")
            try:
                args = node.args or ''
                remappings = {old: new for (old, new) in node.remappings}
                self._load(pkg=node.package,
                           nodetype=node.typ,
                           name=node.name,
                           namespace=node.namespace,  # FIXME
                           launch_filename=node.filename,
                           remappings=remappings,
                           args=args
                           )
            # FIXME this is waaay too permissive
            except Exception:
                logger.exception(f"failed to launch node: {node.name}")
                raise

        # now that all nodes have been initialised, load all plugins
        for node_context in self.nodes.values():
            for plugin in node_context._plugins:
                plugin.load(self)

        # Now that everything has been loaded, check that all nodelets have started
        for manager, nodelets in self._waiting_nodelets.items():
            if nodelets:
                for nodelet_info in nodelets:
                    logger.error(f"Nodelet {nodelet_info[0]} is stil waiting for manager {manager}")

    def _create_nodelet_manager(self,
                                name: str,
                                namespace: str,
                                launch_filename: str,
                                remappings: t.Mapping[str, str]) -> None:
        """Creates a nodelet manager with a given name."""
        logger.info(f'launched nodelet manager: {name}')
        ctx = NodeContext(name=name,
                          namespace=namespace,
                          kind="nodelet",
                          package="nodelet",
                          launch_filename=launch_filename,
                          remappings=remappings,
                          files=self._app.files,
                          params=self.params,
                          app=self._app,
                          args='')
        self.nodes[ctx.fullname] = ctx
        if ctx.fullname in self._waiting_nodelets:
            for nodelet in self._waiting_nodelets[ctx.fullname]:
                nodelet[1]()
                del nodelet

    def _load_nodelet(self,
                      pkg: str,
                      nodetype: str,
                      name: str,
                      namespace: str,
                      launch_filename: str,
                      remappings: Dict[str, str],
                      manager: Optional[str] = None
                      ) -> None:
        """Loads a nodelet using the provided instructions.

        Parameters
        ----------
        pkg: str
            the name of the package to which the nodelet belongs.
        nodetype: str
            the name of the type of nodelet that should be loaded.
        name: str
            the name that should be assigned to the nodelet.
        namespace: str
            the namespace into which the nodelet should be loaded.
        launch_filename: str
            the absolute path to the XML launch file where this node
            was declared.
        remappings: Dict[str, str]
            a dictionary of name remappings that should be applied
            to this nodelet, where keys correspond to old names and values
            correspond to new names.
        manager: Optional[str]
            the name of the manager, if any, for this nodelet. If
            this nodelet is standalone, :code:`manager` should be set to
            :code:`None`.

        Raises
        ------
        Exception
            if there is no model for the given nodelet type.
        """
        if manager:
            logger.info(f'launching nodelet [{name}] '
                        f'inside manager [{manager}]')

            def load_wrapper():
                return self._load(pkg=pkg,
                                  nodetype=nodetype,
                                  name=manager,
                                  namespace=namespace,
                                  launch_filename=launch_filename,
                                  remappings=remappings,
                                  args='manager'
                                  )
            if manager in self.nodes.keys():
                load_wrapper()
            else:
                if manager in self._waiting_nodelets:
                    self._waiting_nodelets[manager].append((name, load_wrapper))
                else:
                    self._waiting_nodelets[manager] = [(name, load_wrapper)]
        else:
            logger.info(f'launching standalone nodelet [{name}]')
            return self._load(pkg=pkg,
                              nodetype=nodetype,
                              name=name,
                              namespace=namespace,
                              launch_filename=launch_filename,
                              remappings=remappings,
                              args=''
                              )

    def _load(self,
              pkg: str,
              nodetype: str,
              name: str,
              namespace: str,
              launch_filename: str,
              remappings: Dict[str, str],
              args: str,
              ) -> None:
        """Loads a node using the provided instructions.

        Parameters
        ----------
        pkg: str
            the name of the package to which the node belongs.
        nodetype: str
            the name of the type of node that should be loaded.
        name: str
            the name that should be assigned to the node.
        namespace: str
            the namespace into which the node should be loaded.
        launch_filename: str
            the absolute path to the XML launch file where this node
            was declared.
        remappings: Dict[str, str]
            a dictionary of name remappings that should be applied
            to this node, where keys correspond to old names and values
            correspond to new names.
        args: str
            a string containing command-line arguments to the node.

        Raises
        ------
        Exception
            if there is no model for the given node type.
        """
        args = args.strip()
        if nodetype == 'nodelet':
            if args == 'manager':
                return self._create_nodelet_manager(name, namespace, launch_filename, remappings)
            elif args.startswith('standalone '):
                pkg_and_nodetype = args.partition(' ')[2]
                pkg, _, nodetype = pkg_and_nodetype.partition('/')
                return self._load_nodelet(pkg=pkg,
                                          nodetype=nodetype,
                                          name=name,
                                          namespace=namespace,
                                          launch_filename=launch_filename,
                                          remappings=remappings
                                          )
            else:
                load, pkg_and_nodetype, mgr = args.split(' ')
                pkg, _, nodetype = pkg_and_nodetype.partition('/')
                return self._load_nodelet(pkg=pkg,
                                          nodetype=nodetype,
                                          name=name,
                                          namespace=namespace,
                                          launch_filename=launch_filename,
                                          remappings=remappings,
                                          manager=mgr
                                          )

        if remappings:
            logger.info(f"using remappings: {remappings}")

        try:
            model = self.models.fetch(pkg, nodetype)
        except Exception:
            m = (f"failed to find model for node type [{nodetype}] "
                 f"in package [{pkg}]")
            logger.warning(m)
            raise Exception(m)
        ctx: t.Optional[NodeContext] = None
        if args == 'manager':
            # This is being loaded into an existing manager, so find that as the context
            if name in self.nodes:
                ctx = self.nodes[name]
            elif f"/{name}" in self.nodes:
                ctx = self.nodes[f"/{name}"]
            else:
                raise ValueError(f"The nodelet manager {name} has not been launched")
        if not ctx:
            ctx = NodeContext(name=name,
                              namespace=namespace,
                              kind=nodetype,
                              package=pkg,
                              args=args,
                              launch_filename=launch_filename,
                              remappings=remappings,
                              files=self._app.files,
                              params=self.params,
                              app=self._app)
            self.nodes[ctx.fullname] = ctx
        model.eval(ctx)
