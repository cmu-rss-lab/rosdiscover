# -*- coding: utf-8 -*-
__all__ = ('NodeRecoveryTool',)

import contextlib
import enum
import json
import os
import shlex
import types
import typing as t

import attr
import roswire
from loguru import logger
from roswire import CMakeBinaryTarget, CMakeTarget, ROSVersion, SourceLanguage

from .loader import SymbolicProgramLoader
from .model import CMakeListsInfo, RecoveredNodeModel
from .symbolic import SymbolicProgram
from ..config import Config


@attr.s(frozen=True, auto_exc=True, auto_attribs=True, str=False)
class CompileCommandsNotFound(ValueError):
    path: str

    def __str__(self) -> str:
        return f"Could not find compile_commands.json at expected location: {self.path}"


class RosBuildTool(enum.Enum):
    CATKIN_TOOLS = "catkin"
    CATKIN_MAKE = "catkin_make"
    CATKIN_MAKE_ISOLATED = "catkin_make_isolated"

    @classmethod
    def from_built_by(cls, contents: str) -> "RosBuildTool":
        """Finds the build tool based on the contents of a .built_by file.

        Raises
        ------
        ValueError
            if the name of the build tool is unrecognized
        """
        contents_to_tool = {
            "catkin build": RosBuildTool.CATKIN_TOOLS,
            "catkin_make": RosBuildTool.CATKIN_MAKE,
            "catkin_make_isolated": RosBuildTool.CATKIN_MAKE_ISOLATED,
        }
        if contents not in contents_to_tool:
            raise ValueError(f"unrecognized ROS build tool: {contents}")
        return contents_to_tool[contents]


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
        assert self._app_instance
        files = self._app_instance.files
        workspace_path = os.path.dirname(package.path)
        while workspace_path != "/":

            catkin_marker_path = os.path.join(workspace_path, ".catkin_workspace")
            logger.debug(f"looking for workspace marker: {catkin_marker_path}")
            if files.exists(catkin_marker_path):
                return workspace_path

            catkin_tools_dir = os.path.join(workspace_path, ".catkin_tools")
            logger.debug(f"looking for workspace marker: {catkin_tools_dir}")
            if files.exists(catkin_tools_dir):
                return workspace_path

            workspace_path = os.path.dirname(workspace_path)

        raise ValueError(f"unable to determine workspace for package: {package}")

    def _find_build_directory(self, workspace: str) -> str:
        """Determines the absolute path to the build directory within a given workspace.

        Raises
        ------
        ValueError
            if the build directory could not be found
        """
        assert self._app_instance
        files = self._app_instance.files

        build_dir = os.path.join(workspace, "build")
        if files.isdir(build_dir):
            return build_dir

        build_dir = os.path.join(workspace, "build_isolated")
        if files.isdir(build_dir):
            return build_dir

        raise ValueError(f"unable to find build directory in workspace: {workspace}")

    def _detect_build_tool(
        self,
        workspace: str,
        build_directory: t.Optional[str] = None,
    ) -> RosBuildTool:
        """Detects the build tool that was used to construct a given workspace.

        Parameters
        ----------
        workspace: str
            The absolute path to the workspace.
        build_directory: str, optional
            The absolute path to the build directory within the given workspace.

        Raises
        ------
        ValueError
            if the build directory could not be found inside the workspace
        ValueError
            if the build tool used to construct the workspace was unrecognized
        ValueError
            if the workspace does not provide a .built_by file inside its build directory
        """
        assert self._app_instance
        files = self._app_instance.files

        if not build_directory:
            build_directory = self._find_build_directory(workspace)

        built_by_path = os.path.join(build_directory, ".built_by")

        try:
            build_tool_name = files.read(built_by_path, binary=False)
        except FileNotFoundError:
            raise ValueError(f"unable to find expected .built_by file: {built_by_path}")

        return RosBuildTool.from_built_by(build_tool_name)

    def _find_compile_commands_file(self, package: roswire.common.Package) -> str:
        """Locates the compile_commands.json for a given package.

        Raises
        ------
        CompileCommandsNotFound
            if no compile_commands.json was found for the given package
        ValueError
            if the build directory could not be found inside the workspace
        ValueError
            if the build tool used to construct the workspace was unrecognized
        ValueError
            if the workspace does not provide a .built_by file inside its build directory

        Returns
        -------
        str
            the absolute path of the compile_commands.json file
        """
        assert self._app_instance
        files = self._app_instance.files
        workspace = self._find_package_workspace(package)
        build_directory = self._find_build_directory(workspace)
        build_tool = self._detect_build_tool(workspace, build_directory)

        compile_commands_directory: str
        if build_tool == RosBuildTool.CATKIN_MAKE:
            compile_commands_directory = build_directory
        elif build_tool in (RosBuildTool.CATKIN_TOOLS, RosBuildTool.CATKIN_MAKE_ISOLATED):
            compile_commands_directory = os.path.join(build_directory, package.name)
        else:
            raise AssertionError(
                f"attempted to find compile_commands.json for unsupported build tool: {build_tool.value}"
            )

        compile_commands_path = os.path.join(compile_commands_directory, "compile_commands.json")
        if not files.exists(compile_commands_path):
            raise CompileCommandsNotFound(compile_commands_path)
        return compile_commands_path

    def _info_via_cmake(
        self,
        package: roswire.common.Package,
        node_name: str,
    ) -> CMakeTarget:
        assert self._app_instance
        cmake_info: t.Mapping[str, CMakeTarget]
        if self._app.description.distribution.ros == ROSVersion.ROS1:
            ros1 = self._app_instance.ros1()
            cmake_info = ros1.package_node_sources(package)
        else:
            ros2 = self._app_instance.ros2
            cmake_info = ros2.package_node_sources(package)
        # ROSWire puts nodelets into CMakeInfo by their name, which happens to
        # be a combination of package/node_name passed in via nodelet loading
        # So, lookup via that
        # TODO: Improve this API
        nodelet_name_reference = f"{package}/{node_name}"
        if node_name not in cmake_info and nodelet_name_reference not in cmake_info:
            logger.info(f"CMakeLists.txt contains: {str(cmake_info.keys())}")
            raise ValueError(f"{node_name} is not in the CMakeLists.txt of package '{package.name}")
        if node_name in cmake_info:
            node_source_info = cmake_info[node_name]
        else:
            node_source_info = cmake_info[nodelet_name_reference]
        if node_source_info.language != SourceLanguage.CXX:
            raise NotImplementedError("Can only recover node information for C++ nodes")
        logger.info(f"Recovered sources for {node_name} as {str(node_source_info.sources)}")
        return node_source_info

    def recover_using_cmakelists(self, package_name: str, node_name: str) -> RecoveredNodeModel:
        try:
            package = self._app.description.packages[package_name]
        except KeyError as err:
            raise ValueError(f"no package found with given name: {package_name}") from err

        source_info = self._info_via_cmake(package, node_name)
        entrypoint = "main"
        if isinstance(source_info, CMakeBinaryTarget):
            assert source_info.entrypoint is not None
            entrypoint = source_info.entrypoint
        return self.recover(package_name, node_name, entrypoint, source_info.sources,
                            source_info.restrict_to_paths, source_info.cmakelists_file,
                            source_info.cmakelists_line)

    def recover(
        self,
        package_name: str,
        node_name: str,
        entrypoint: str,
        sources: t.Collection[str],
        path_restrictions: t.Collection[str],
        filename: str = "<unknown>",
        lineno: int = -1,
    ) -> RecoveredNodeModel:
        """Statically recovers the dynamic architecture of a given node.

        Parameters
        ----------
        package_name: str
            The name of the package to which the node belongs
        node_name: str
            The name of the node
        entrypoint: str
            The function that represents the entrypoint for the node (e.g., main)
        sources: str
            A list of the translation unit source files for node, provided as paths
            relative to the root of the package directory
        filename: str
            The name of the cmake file that this is dervied from ("<unknown>" by default, indicates no information)
        lineno: int
            The line number in the cmake file that generated this (-1 by default, indicates no information)

        Raises
        ------
        ValueError
            if no package is found with the given name
        CompileCommandsNotFound
            if no compile_commands.json was found for the given package
        ValueError
            if the build directory could not be found inside the workspace
        ValueError
            if the build tool used to construct the workspace was unrecognized
        ValueError
            if the workspace does not provide a .built_by file inside its build directory
        RuntimeError
            if the static recovery process fails
        """
        try:
            package = self._app.description.packages[package_name]
        except KeyError as err:
            raise ValueError(f"no package found with given name: {package_name}") from err

        # ensure that no absolute paths are given
        if any(os.path.isabs(path) for path in sources):
            raise ValueError("expected source paths to be relative to package directory")

        # compute the absolute paths of each source file
        sources = [os.path.join(package.path, path) for path in sources]

        compile_commands_path = self._find_compile_commands_file(package)

        # recover a symbolic description of the node executable
        program = self._recover(compile_commands_path, entrypoint, sources, path_restrictions)

        package_abs_path = self._app.description.packages[package_name].path
        cmakeinfo = None
        if filename != "<unknown>" and lineno != -1:
            cmakeinfo = CMakeListsInfo(filename=filename, lineno=lineno)
        return RecoveredNodeModel(
            image_sha256=self._app.sha256,
            package_name=package_name,
            package_abs_path=package_abs_path,
            source_paths=tuple(sources),
            node_name=node_name,
            program=program,
            cmakelist_info=cmakeinfo
        )

    def _recover(
        self,
        compile_commands_path: str,
        entrypoint: str,
        source_file_abs_paths: t.Collection[str],
        restrict_to_paths: t.Collection[str]
    ) -> SymbolicProgram:
        """Invokes the C++ recovery binary to recover the dynamic architecture of a given node.

        Parameters
        ----------
        compile_commands_path: str
            The absolute path to the compile_commands.json associated with the given node.
        entrypoint: str
            The name of the function that is the entry point for the symbolic program.
        source_file_abs_paths: str
            A list of the C++ translation unit source files (i.e., .cpp files)
            for the given node, provided as absolute paths within the container

        Returns
        -------
        SymbolicProgram
            A description of the recovered program.
        """
        if not self._app_instance:
            raise ValueError("tool has not been started")

        logger.debug("beginning static recovery process")
        shell = self._app_instance.shell
        files = self._app_instance.files

        if not source_file_abs_paths:
            raise ValueError("expected at least one source file")

        if not os.path.isabs(compile_commands_path):
            raise ValueError(f"expected absolute compile commands path: {compile_commands_path}")

        if not files.exists(compile_commands_path):
            raise CompileCommandsNotFound(compile_commands_path)

        for source_file in source_file_abs_paths:
            if not os.path.isabs(source_file):
                raise ValueError(f"expected absolute source file path: {source_file}")
            if not files.exists(source_file):
                raise ValueError(f"source file was not found: {source_file}")

        for source_file in source_file_abs_paths:
            self._prepare_source_file(source_file)

        # create a temporary file inside the container to hold the JSON-based node summary
        json_model_filename = files.mktemp('.json')

        env = {
            "PATH": "/opt/rosdiscover/bin:${PATH:-}",
            "LIBRARY_PATH": "/opt/rosdiscover/lib:${LIBRARY_PATH:-}",
            "LD_LIBRARY_PATH": "/opt/rosdiscover/lib:${LD_LIBRARY_PATH:-}",
        }
        env_args = [f"{var}={val}" for (var, val) in env.items()]
        args = env_args + [
            "rosdiscover-cxx-extract",
            "-p",
            shlex.quote(os.path.dirname(compile_commands_path)),
            "-output-filename",
            json_model_filename,
        ]
        for path in restrict_to_paths:
            args += ['-restrict-to', path]

        args += [' '.join(shlex.quote(p) for p in source_file_abs_paths)]
        args_s = ' '.join(args)
        logger.debug(f"running static recovery command: {args_s}")
        outcome = shell.run(args_s, text=True, stderr=True)
        assert isinstance(outcome.output, str)

        if outcome.returncode != 0:
            logger.error(f"static recovery failed [returncode: {outcome.returncode}]: {outcome.output}")
            raise RuntimeError("static recovery process failed")

        logger.debug(f"static recovery output: {outcome.output}")
        logger.debug("finished static recovery process")

        # load the symbolic summary from the temporary file
        model_loader = SymbolicProgramLoader(self._app)
        json_model_file_contents = files.read(json_model_filename, binary=False)
        json_model = json.loads(json_model_file_contents)

        # inject the name of the entrypoint into the JSON description
        json_model["program"]["entrypoint"] = entrypoint

        summary = model_loader.load(json_model)
        logger.debug(f"recovered node summary: {summary}")
        return summary
