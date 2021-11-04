# -*- coding: utf-8 -*-
from __future__ import annotations

__all__ = ('Config',)

from types import MappingProxyType
import os
import enum
import typing as t

from loguru import logger
import attr
import roswire
import roswire.common
import yaml

from .launch import Launch


class ROSNodeKind(enum.Enum):
    NODE = "node"
    NODELET = "nodelet"

    def __str__(self) -> str:
        return self.value

    @classmethod
    def value_of(cls, value: str) -> "ROSNodeKind":
        if 'node' == value:
            return ROSNodeKind.NODE
        if 'nodelet' == value:
            return ROSNodeKind.NODELET
        raise NotImplementedError


@attr.s(frozen=True, slots=True, auto_attribs=True)
class NodeSourceInfo:
    """
    This is a class to represent nodes and their sources, as a bootstrap for recovering node information
    from static analysis.

    Note: This is an interim class, and will be replaced by CMakelists.txt analysis in the fullness of time.

    Attributes
    ----------
    package_name: str
        The name of the package where the node is from
    node_name: str
        The name of the node provided by the package
    node_kind: ROSNodeKind
        The kind of node represented
    restrict_to_paths: Collection[str]
        The paths to restrict source analysis to
    entrypoint: str
        The entry point for the main program in the source. The format for this is
        a fully qualified classname, followed by the name of the function, like:
        qualified.class.name::main
    sources: Sequence[str]
        The list of sources for building the node
    origin: Optional[str]
    """
    package_name: str
    node_name: str
    node_kind: ROSNodeKind
    restrict_to_paths: t.Collection[str]
    entrypoint: str
    sources: t.Sequence[str]
    origin: t.Optional[str]

    @classmethod
    def from_dict(cls, dict_: t.Mapping[str, t.Any]) -> 'NodeSourceInfo':
        """
        Raises
        ------
        ValueError
            If 'package' is undefined.
        ValueError
            If 'node' is undefined.
        ValueError
            If 'sources' is undefined.
        """
        if 'package' not in dict_:
            raise ValueError("'package' is undefined for the node source.")
        if 'node' not in dict_:
            raise ValueError("'node' is undefined for the node source.")
        if 'sources' not in dict_:
            raise ValueError("'sources' is undefined for the node source.")

        kind = ROSNodeKind.NODE
        entrypoint = "main"

        if 'kind' in dict_:
            if not isinstance(dict_['kind'], str):
                raise ValueError("expected 'kind' to be a string")
            kind = ROSNodeKind.value_of(dict_['kind'])

        if kind == ROSNodeKind.NODELET and 'entrypoint' not in dict_:
            raise ValueError("'entrypoint' is undefined for the nodelet source.")

        if not isinstance(dict_['package'], str):
            raise ValueError("expected 'package' to be a string")
        if not isinstance(dict_['node'], str):
            raise ValueError("expected 'node' to be a string")
        if not isinstance(dict_['sources'], list):
            raise ValueError("expected 'sources' to be a list")
        if 'entrypoint' in dict_:
            if not isinstance(dict_['entrypoint'], str):
                raise ValueError("expected 'entrypoint' to be a string")
            entrypoint = dict_['entrypoint']

        if 'restrict-analysis-to-paths' in dict_:
            if not isinstance(dict_['restrict-analysis-to-paths'], list):
                raise ValueError("expected 'restrict-analysis-to-paths' to be a list")

        restricted_paths = dict_.get('restrict-analysis-to-paths', [])

        for path in restricted_paths:
            if not os.path.isabs(path):
                raise ValueError(f"Restricted path [{path}] must be absolute")

        if 'origin' in dict_:
            if not isinstance(dict_['origin'], str):
                raise ValueError("expected 'origin' to be a string")
        origin = dict_.get('origin', None)

        return NodeSourceInfo(
            package_name=dict_['package'],
            node_name=dict_['node'],
            node_kind=kind,
            entrypoint=entrypoint,
            sources=list(dict_['sources']),
            restrict_to_paths=restricted_paths,
            origin=origin,
        )

    def to_dict(self) -> t.Dict[str, t.Any]:
        dict_ = {
            "package": self.package_name,
            "node": self.node_name,
            "kind": str(self.node_kind),
            "entrypoint": self.entrypoint,
            "restrict-analysis-to-paths": list(self.restrict_to_paths),
            "sources": list(self.sources),
        }
        if self.origin:
            dict_['origin'] = self.origin
        return dict_


@attr.s(frozen=True, slots=True, auto_attribs=True)
class Config:
    """
    Attributes
    ----------
    image: str
        The name of the Docker image for the application.
    sources: Sequence[str]
        The sequence of source files that should be sourced to obtain the
        catkin workspace for the application.
    launches: Sequence[str]
        The sequence of launch files that should be used to launch the
        application.
    environment: Mapping[str, str]
        A set of environment variables that should be used by the application.
    app: roswire.app.App
        The ROSWire application for this configuration.
    node_sources: List[NodeSourceInfo]
        Provides instructions on how to statically recover the architecture code for individual nodes
    """
    image: str
    sources: t.Sequence[str]
    launches: t.Sequence[Launch]
    environment: t.Mapping[str, str] = attr.ib(factory=dict)
    node_sources: t.Mapping[t.Tuple[str, str], NodeSourceInfo] = attr.ib(factory=dict)
    app: roswire.app.App = attr.ib(init=False)

    @classmethod
    def from_dict(cls, dict_: t.Mapping[str, t.Any]) -> 'Config':
        """
        Raises
        ------
        ValueError
            If 'image' is undefined in configuration.
        ValueError
            If 'launches' is undefined in configuration.
        """
        if 'image' not in dict_:
            raise ValueError("'image' is undefined in configuration")
        if 'launches' not in dict_:
            raise ValueError("'launches' is undefined in configuration")
        if 'sources' not in dict_:
            raise ValueError("'sources' is undefined in configuration")
        if not isinstance(dict_['image'], str):
            raise ValueError("expected 'image' to be a string")
        if not isinstance(dict_['sources'], list):
            raise ValueError("expected 'sources' to be a list")
        if not isinstance(dict_['launches'], list):
            raise ValueError("expected 'launches' to be a list")
        if all(type(d) == dict for d in dict_["launches"]):
            launch_args_provided = True
        elif all(type(d) == str for d in dict_["launches"]):
            launch_args_provided = False
        else:
            raise ValueError("expected 'launches' to be a list or dict")

        has_environment = 'environment' in dict_
        if has_environment and not isinstance(dict_['environment'], dict):
            raise ValueError("expected 'environment' to be a mapping")

        has_node_sources = 'node_sources' in dict_
        if has_node_sources and not isinstance(dict_['node_sources'], list):
            raise ValueError("expected 'node_sources' to be a list")

        image: str = dict_['image']
        sources: t.Sequence[str] = dict_['sources']
        environment: t.Mapping[str, str] = dict(dict_.get('environment', {}))
        node_sources_list: t.Sequence[t.Dict[str, t.Any]] = list(dict_.get('node_sources', []))

        node_sources = {(nsi.package_name, nsi.node_name): nsi
                        for nsi in (NodeSourceInfo.from_dict(d)
                                    for d in node_sources_list)}
        launches_inputs: t.Sequence[t.Any] = dict_['launches']
        if launch_args_provided:
            launches = list(map(lambda d: Launch.from_dict(d), launches_inputs))
        else:
            launches = list(map(lambda s: Launch(filename=s, arguments=dict()), launches_inputs))

        return Config(
            image=image,
            sources=sources,
            launches=launches,
            environment=environment,
            node_sources=node_sources,
        )

    def to_dict(self) -> t.Dict[str, t.Any]:
        return {
            "image": self.image,
            "sources": list(self.sources),
            "launches": [launch.to_dict() for launch in self.launches],
            "environment": dict(self.environment),
            "node_sources": [s.to_dict() for s in self.node_sources.values()],
        }

    def with_recovered_node_sources(self) -> "Config":
        recovered_node_sources = self.find_node_sources()
        return Config(
            image=self.image,
            sources=list(self.sources),
            launches=list(self.launches),
            environment=dict(self.environment),
            node_sources=recovered_node_sources,
        )

    def find_node_sources(self) -> t.Mapping[t.Tuple[str, str], NodeSourceInfo]:
        """Determines the sources for each node and nodelet in the associated image.

        Returns
        -------
        t.Mapping[t.Tuple[str, str], NodeSourceInfo]
            a mapping from (package, node) names to their corresponding sources
        """
        app_description = self.app.describe()

        if app_description.distribution.ros != roswire.ROSVersion.ROS1:
            raise NotImplementedError("find_node_sources is only implemented for ROS1")

        package_node_to_sources: t.Dict[t.Tuple[str, str], NodeSourceInfo] = {}

        with self.app.launch() as app_instance:
            ros = app_instance.ros1()
            for package_cmake_targets in ros.cmake_targets_for_all_packages():
                package = package_cmake_targets.package
                targets = package_cmake_targets.targets
                for target in targets:
                    node_sources = self.__cmake_target_to_node_sources(package, target)
                    if not node_sources:
                        continue

                    key = (node_sources.package_name, node_sources.node_name)
                    package_node_to_sources[key] = node_sources

        return package_node_to_sources

    def __cmake_target_to_node_sources(
        self,
        package: roswire.common.Package,
        target: roswire.common.CMakeTarget,
    ) -> t.Optional[NodeSourceInfo]:
        if isinstance(target, roswire.common.source.CMakeLibraryTarget):
            return self.__cmake_library_to_node_sources(package, target)
        elif isinstance(target, roswire.common.source.CMakeBinaryTarget):
            return self.__cmake_binary_to_node_sources(package, target)
        elif isinstance(target, roswire.common.source.CMakeTarget):
            logger.warning(f"Bad target: {target.name} on line {target.cmakelists_line} in {target.cmakelists_file} "
                           f"is in {target.language}")
            return None

    def __cmake_library_to_node_sources(
        self,
        package: roswire.common.Package,
        target: roswire.common.source.CMakeLibraryTarget,
    ) -> t.Optional[NodeSourceInfo]:
        assert target.name is not None

        # not all libraries are nodelets
        if not target.entrypoint:
            return None

        return NodeSourceInfo(
            package_name=package.name,
            node_name=target.name,
            node_kind=ROSNodeKind.NODELET,
            restrict_to_paths=target.restrict_to_paths,
            entrypoint=target.entrypoint,
            sources=list(target.sources),
            origin=f"{target.cmakelists_file}:{target.cmakelists_line}",
        )

    def __cmake_binary_to_node_sources(
        self,
        package: roswire.common.Package,
        target: roswire.common.source.CMakeBinaryTarget,
    ) -> t.Optional[NodeSourceInfo]:
        assert target.name is not None
        return NodeSourceInfo(
            package_name=package.name,
            node_name=target.name,
            node_kind=ROSNodeKind.NODE,
            restrict_to_paths=target.restrict_to_paths,
            entrypoint="main",
            sources=list(target.sources),
            origin=f"{target.cmakelists_file}:{target.cmakelists_line}",
        )

    @classmethod
    def from_yaml_string(cls, yml: str) -> 'Config':
        logger.debug(f'loading config from YAML string:\n{yml}')
        dict_ = yaml.load(yml, Loader=yaml.SafeLoader)
        config = cls.from_dict(dict_)
        logger.debug(f'loaded config from YAML string: {config}')
        return config

    @classmethod
    def load(cls, filename: str) -> 'Config':
        logger.debug(f'loading config from file: {filename}')
        with open(filename, 'r') as f:
            contents = f.read()
        config = cls.from_yaml_string(contents)
        logger.debug(f'loaded config from file [{filename}]: {config}')
        return config

    @property
    def image_sha256(self) -> str:
        """Returns the SHA-256 of the Docker image for this application, represented
        as a hexadecimal string."""
        return self.app.sha256

    def __attrs_post_init__(self) -> None:
        object.__setattr__(self, 'sources', tuple(self.sources))
        object.__setattr__(self, 'launches', tuple(self.launches))
        environment = MappingProxyType(dict(self.environment))
        object.__setattr__(self, 'environment', environment)

        # TODO allow this context to be passed in
        rsw = roswire.ROSWire()
        app = roswire.App(roswire=rsw,
                          image=self.image,
                          sources=self.sources)
        object.__setattr__(self, 'app', app)
