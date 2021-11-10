# -*- coding: utf-8 -*-
__all__ = ("ProjectModels",)

import os
import types
import typing as t

import attr
from loguru import logger

from ..config import Config
from ..interpreter.model import NodeModel, HandwrittenModel, PlaceholderModel
from ..recover import NodeRecoveryTool, RecoveredNodeModelDatabase


@attr.s(slots=True, auto_attribs=True)
class ProjectModels:
    """Provides an interface for accessing and recovering the models for a given project.

    Attributes
    ----------
    config: Config
        The configuration for the project
    allow_recovery: bool
        Allows models to be recovered from source if :code:`True`.
    allow_placeholders: bool
        If :code:`True`, uses a placeholder model if there is no handwritten
        model available and recovery is disabled or not possible.
    """
    config: Config
    allow_recovery: bool = attr.ib(default=True)
    allow_placeholders: bool = attr.ib(default=True)
    _recovered_models: RecoveredNodeModelDatabase = attr.ib(init=False)
    _recovery_tool: NodeRecoveryTool = attr.ib(init=False)
    # TODO add: use_model_cache
    # TODO add: prefer_recovered

    def __attrs_post_init__(self) -> None:
        # TODO allow model database path to be specified
        recovered_model_database_path = os.path.expanduser("~/.rosdiscover/recovered-models")
        self._recovered_models = RecoveredNodeModelDatabase(recovered_model_database_path)
        self._recovery_tool = NodeRecoveryTool(app=self.config.app)

    def __enter__(self) -> "ProjectModels":
        self.open()
        return self

    def __exit__(
        self,
        ex_type: t.Optional[t.Type[BaseException]],
        ex_val: t.Optional[BaseException],
        ex_tb: t.Optional[types.TracebackType],
    ) -> None:
        self.close()

    def open(self) -> None:
        self._recovery_tool.open()

    def close(self) -> None:
        self._recovery_tool.close()

    def _recover(self, package: str, node: str) -> t.Optional[NodeModel]:
        # have we already recovered this model?
        logger.debug(f"Recovering {package}/{node}")
        if self._recovered_models.contains(self.config, package, node):
            return self._recovered_models.fetch(self.config, package, node)

        # is this node model irrecoverable?
        if (package, node) not in self.config.node_sources:
            try:
                logger.info(f"Attempting to recover {package}/{node} from CMakeLists.txt")
                model = self._recovery_tool.recover_using_cmakelists(package, node)
            except ValueError:
                logger.exception(f"Error recovering {package}/{node}")
                return None
            except RuntimeError:
                logger.exception(f"Static recovery failed for {package}/{node}")
                return None
        else:
            logger.info(f"Attempting to use passed in node_sources for {package}/{node}")
            node_info = self.config.node_sources[(package, node)]
            cmake_filename_and_line = node_info.origin.split(':') if node_info.origin else ["<unknown>", "-1"]

            # use the recovery tool to recover the model before saving it to the database
            model = self._recovery_tool.recover(
                package,
                node,
                node_info.entrypoint,
                node_info.sources,
                node_info.restrict_to_paths,
                cmake_filename_and_line[0],
                int(cmake_filename_and_line[1])
            )
        self._recovered_models.store(model)
        return model

    def _fetch_handwritten(self, package: str, node: str) -> t.Optional[NodeModel]:
        logger.debug(f"Handwritten {package}/{node}")
        if HandwrittenModel.exists(package, node):
            return HandwrittenModel.fetch(package, node)
        # '*' is a placeholder for a package name. It's admittedly a hack, but it
        # saves looking for packages that aren't on the system. It is used when
        # we use dummy nodes for misconfiguration detection
        # FIXME: Do something more principled.
        if HandwrittenModel.exists("*", node):
            return HandwrittenModel.fetch("*", node)
        return None

    def _fetch_placeholder(self, package: str, node: str) -> NodeModel:
        return PlaceholderModel(package, node)

    def fetch(self, package: str, node: str) -> NodeModel:
        """Retrieves the model for a given node.
        By default, the handwritten model for this node will be returned, if
        available, since this is likely to be the most accurate. If there is no
        handwritten model for the given node, then the model will be statically
        recovered from source if model recovery is allowed. Finally, if there
        is no handwritten model and model recovery is disabled or not possible,
        a placeholder model will be returned unless placeholders are disabled.

        Parameters
        ----------
        package: str
            The name of the package to which the node belongs.
        node: str
            The name of the type of node.

        Returns
        -------
        NodeModel
            When searching for a given model, highest preference will be given to
            handwritten models. If there is no recovered model, then this method will
            attempt to return a handwritten model. If there is also no handwritten model,
            then a placeholder model will be returned instead.

        Raises
        ------
        ValueError
            If there is no model could be fetched for the given node and placeholders
            have been disabled.
        """
        # TODO this exists to make it easier to customize preferences later on
        # e.g., to prefer recovered models over handwritten ones
        model_sources: t.List[t.Callable[[str, str], t.Optional[NodeModel]]] = [
            self._fetch_handwritten,
        ]
        if self.allow_recovery:
            model_sources.append(self._recover)

        fetched_model: t.Optional[NodeModel] = None
        for model_source in model_sources:
            fetched_model = model_source(package, node)
            if fetched_model:
                return fetched_model

        if self.allow_placeholders:
            return self._fetch_placeholder(package, node)
        else:
            raise ValueError(f"failed to fetch model for node [{node}] in package [{package}]")
