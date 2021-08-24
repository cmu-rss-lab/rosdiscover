# -*- coding: utf-8 -*-
__all__ = ('Config',)

import enum
from pathlib import Path
from types import MappingProxyType
import typing as t

from loguru import logger
import attr
import roswire
import yaml

from .launch import Launch


class ROSNodeKind(enum.Enum):
    NODE = "node"
    NODELET = "nodelet"

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
    """
    package_name: str
    node_name: str
    node_kind: ROSNodeKind
    restrict_to_paths: t.Collection[str]
    entrypoint: str
    sources: t.Sequence[str]

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
                raise ValueError("expected 'restrict-analysis-to-paths' to bae a list")
        restricted_paths = dict_.get('restrict-analysis-to-paths', [])

        for path in restricted_paths:
            if not Path(path).is_absolute():
                raise ValueError(f"Restricuted path '{path}' should be absolute")

        return NodeSourceInfo(
            package_name=dict_['package'],
            node_name=dict_['node'],
            node_kind=kind,
            entrypoint=entrypoint,
            sources=list(dict_['sources']),
            restrict_to_paths=restricted_paths
        )


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

        return Config(image=image,
                      sources=sources,
                      launches=launches,
                      environment=environment,
                      node_sources=node_sources)

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
