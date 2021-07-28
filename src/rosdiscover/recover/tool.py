# -*- coding: utf-8 -*-
__all__ = ('NodeRecoveryTool',)

import contextlib
import enum
import shlex
import os
import types
import typing as t

from loguru import logger
import attr
import roswire

from ..config import Config


class RosBuildTool(enum.Enum):
    CATKIN = "catkin"
    CATKIN_MAKE = "catkin_make"
    CATKIN_MAKE_ISOLATED = "catkin_make_isolated"

    @classmethod
    def from_string(cls, name: str) -> "RosBuildTool":
        """Finds the build tool with the given name.

        Raises
        ------
        ValueError
            if the name of the build tool is unrecognized
        """
        try:
            return RosBuildTool[CATKIN]
        except KeyError as err:
            raise ValueError(f"unrecognized ROS build tool: {name}") from err


@attr.s(auto_attribs=True)
class NodeRecoveryTool:
    _app: roswire.app.App
    _app_instance: t.Optional[roswire.app.AppInstance] = attr.ib(default=None, repr=False)

    @classmethod
    @contextlib.contextmanager
    def for_config(cls, config: Config) -> t.Iterator["NodeRecoveryTool"]:
        with NodeRecoveryTool(app=config.app) as tool:
            yield tool

    def __enter__(self) -> "NodeRecoveryTool":
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
        if self._app_instance:
            raise ValueError("tool has already been started")

        logger.debug("launching container for static recovery")
        volumes = {
            "rosdiscover-cxx-extract-opt": {
                "mode": "ro",
                "bind": "/opt/rosdiscover",
            },
            "rosdiscover-cxx-extract-llvm": {
                "mode": "ro",
                "bind": "/opt/llvm11",
            },
        }
        self._app_instance = self._app.launch(
            volumes=volumes,
        )
        logger.debug("launched static recovery container")

    def close(self) -> None:
        if not self._app_instance:
            raise ValueError("tool has not been started")

        self._app_instance.close()
        self._app_instance = None

    # TODO we probably need to run this on all C/C++ source files within the workspace and
    # not just the translation unit source files (e.g., header files)
    def _prepare_source_file(self, abs_path: str) -> None:
        """Prepares a source file for static recovery."""
        assert self._app_instance
        shell = self._app_instance.shell
        escaped_abs_path = shlex.quote(abs_path)
        shell.run(f'sed -i "s#std::isnan#__STDISNAN__#g" {escaped_abs_path}')
        shell.run(f'sed -i "s#isnan#__STDISNAN__#g" {escaped_abs_path}')
        shell.run(f'sed -i "s#__STDISNAN__#std::isnan#g" {escaped_abs_path}')

    def _find_package_workspace(self, package: roswire.common.Package) -> str:
        """Determines the absolute path of the workspace to which a given package belongs.

        Raises
        ------
        ValueError
            if the workspace for the given package could not be determined
        """
        files = self._app_instance.files
        workspace_path = os.path.dirname(package.path)
        while workspace_path != "/":
            catkin_marker_path = os.path.join(workspace_path, ".catkin_workspace")
            if files.exists(catkin_marker_path):
                return workspace_path
            workspace_path = os.path.dirname(workspace_path)

        raise ValueError(f"unable to determine workspace for package: {package_name}")

    def _find_build_directory(self, workspace: str) -> str:
        """Determines the absolute path to the build directory within a given workspace.

        Raises
        ------
        ValueError
            if the build directory could not be found
        """
        files = self._app_instance.files

        build_dir = os.path.join(workspace, "build")
        if files.isdir(build_dir):
            return build_dir

        build_dir = os.path.join(workspace, "build_isolated")
        if files.isdir(build_dir):
            return build_dir

        raise ValueError(f"unable to find build directory in workspace: {workspace}")

    def _detect_build_tool(self, workspace: str) -> RosBuildTool:
        """Detects the build tool that was used to construct a given workspace.

        Parameters
        ----------
        workspace: str
            The absolute path to the workspace.

        Raises
        ------
        ValueError
            if the build directory could not be found inside the workspace
        ValueError
            if the build tool used to construct the workspace was unrecognized
        """
        files = self._app_instance.files
        build_directory = self._find_build_directory(workspace)

        # TODO raise exception if .built_by doesn't exist
        # TODO look at .built_by file

        raise NotImplementedError

    def nice_recover(
        self,
        package_name: str,
        node_name: str,
        sources: t.Collection[str],
    ) -> None:
        try:
            package = self._app.description.packages[package_name]
        except KeyError as err:
            raise ValueError(f"no package found with given name: {package_name}") from err

        workspace = self._find_package_workspace(package)
        build_tool = self._detect_build_tool(workspace)

        # TODO find the build directory that holds compile_commands.json
        # catkin tools: build/{PACKAGE_NAME}/compile_commands.json
        # catkin_make: WS/build/compile_commands.json
        # catkin_make_isolated: ?

        # TODO find the compile_commands.json file; raise an exception if it doesn't exist

    def recover(
        self,
        workspace_abs_path: str,
        source_file_abs_paths: t.Collection[str],
    ) -> None:
        """Statically recovers the dynamic architecture of a given node.

        Parameters
        ----------
        workspace_abs_path: str
            The absolute path to the Catkin workspace (within the container)
            where the source code is located
        source_file_abs_paths: str
            A list of the C++ translation unit source files (i.e., .cpp files)
            for the given node, provided as absolute paths within the container
        """
        if not self._app_instance:
            raise ValueError("tool has not been started")

        logger.debug("beginning static recovery process")
        shell = self._app_instance.shell
        files = self._app_instance.files

        if not source_file_abs_paths:
            raise ValueError("expected at least one source file")

        if not os.path.isabs(workspace_abs_path):
            raise ValueError(f"expected absolute workspace path: {workspace_abs_path}")

        if not files.isdir(workspace_abs_path):
            raise ValueError(f"no directory found at given workspace path: {workspace_abs_path}")

        for source_file in source_file_abs_paths:
            if not os.path.isabs(source_file):
                raise ValueError(f"expected absolute source file path: {source_file}")
            if not files.exists(source_file):
                raise ValueError(f"source file was not found: {source_file}")

        for source_file in source_file_abs_paths:
            self._prepare_source_file(source_file)

        env = {
            "PATH": "/opt/rosdiscover/bin:/opt/llvm11/bin:${PATH:-}",
            "LIBRARY_PATH": "/opt/rosdiscover/lib:/opt/llvm11/lib:${LIBRARY_PATH:-}",
            "LD_LIBRARY_PATH": "/opt/rosdiscover/lib:/opt/llvm11/lib:${LD_LIBRARY_PATH:-}",
        }
        env_args = [f"{var}={val}" for (var, val) in env.items()]
        args = env_args + [
            "rosdiscover",
            "-p",
            shlex.quote(workspace_abs_path),
            ' '.join(shlex.quote(p) for p in source_file_abs_paths),
        ]
        args_s = ' '.join(args)
        logger.debug(f"running static recovery command: {args_s}")
        outcome = shell.run(args_s, text=True, stderr=True)
        assert isinstance(outcome.output, str)
        logger.debug(f"static recovery output: {outcome.output}")
        logger.debug("finished static recovery process")
