# -*- coding: utf-8 -*-
import types
import typing as t
import contextlib

from loguru import logger
import roswire
from roswire import AppInstance, ROSVersion
from roswire.ros1.launch.reader import ROS1LaunchFileReader
from roswire.ros2.launch.reader import ROS2LaunchFileReader

from .context import NodeContext
from .model import HandwrittenModel
from .summary import SystemSummary
from .parameter import ParameterServer
from ..config import Config, NodeSourceInfo
from ..launch import Launch
from ..recover import NodeRecoveryTool


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
                   ) -> t.Iterator['Interpreter']:
        """Constructs an interpreter for a given configuration"""
        rsw = roswire.ROSWire()  # TODO don't maintain multiple instances
        with rsw.launch(config.image, config.sources, environment=config.environment) as app:
            with Interpreter(app, config.node_sources) as interpreter:
                yield interpreter

    def __init__(self, app: roswire.System, node_sources: t.Sequence[NodeSourceInfo]) -> None:
        self._app = app
        self.params = ParameterServer()
        self.nodes: t.Dict[str, NodeContext] = {}
        self._node_sources = {(source.package_name, source.node_name): source for source in node_sources}
        self._recovery_tool: t.Optional[NodeRecoveryTool] = None

    def __enter__(self) -> "Interpreter":
        return self

    def __exit__(
        self,
        ex_type: t.Optional[t.Type[BaseException]],
        ex_val: t.Optional[BaseException],
        ex_tb: t.Optional[types.TracebackType],
    ) -> None:
        if self._recovery_tool:
            self._recovery_tool.close()

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

        for node in config.nodes:
            if not node.filename:
                m = ("unable to determine associated launch file for "
                     f"node: {node}")
                raise Exception(m)
            logger.debug(f"launching node: {node.name}")
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

    def _create_nodelet_manager(self, name: str) -> None:
        """Creates a nodelet manager with a given name."""
        logger.info('launched nodelet manager: %s', name)

    def _load_nodelet(self,
                      pkg: str,
                      nodetype: str,
                      name: str,
                      namespace: str,
                      launch_filename: str,
                      remappings: t.Dict[str, str],
                      manager: t.Optional[str] = None
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
              remappings: t.Dict[str, str],
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
                return self._create_nodelet_manager(name)
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
        if (pkg, nodetype) in self._node_sources:
            sources = self._node_sources[(pkg, nodetype)]
            logger.info(f"Found {nodetype} in {pkg}: Need to examine {str(sources.sources)}")
            if not self._recovery_tool:
                self._recovery_tool = NodeRecoveryTool(app=self.app.app)
                self._recovery_tool.open()
            model_str = self._recovery_tool.recover(pkg, nodetype, sources.sources)
            logger.info(f"Found model for {nodetype} in {pkg}")
            logger.info(model_str)

            logger.error("The substance is not implemented yet.")
            return
        else:
            try:
                model = HandwrittenModel.find(pkg, nodetype)
            except Exception:
                m = (f"failed to find model for node type [{nodetype}] "
                     f"in package [{pkg}]")
                logger.warning(m)
                raise Exception(m)

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
