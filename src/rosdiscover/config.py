# -*- coding: utf-8 -*-
__all__ = ('Config',)

from types import MappingProxyType
import typing as t

from loguru import logger
import attr
import roswire
import yaml


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
    sources: Sequence[str]
        The list of sources for building the node
    """
    package_name: str
    node_name: str
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

        if not isinstance(dict_['package'], str):
            raise ValueError("expected 'package' to be a string")
        if not isinstance(dict_['node'], str):
            raise ValueError("expected 'node' to be a string")
        if not isinstance(dict_['sources'], list):
            raise ValueError("expected 'sources' to be a string")

        return NodeSourceInfo(
            package_name=dict_['package'],
            node_name=dict_['node'],
            sources=list(dict_['sources']),
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
        The list of information about the sources for nodes
    """
    image: str
    sources: t.Sequence[str]
    launches: t.Sequence[str]
    environment: t.Mapping[str, str] = attr.ib(factory=dict)
    node_sources: t.Mapping[t.Tuple[str, str], NodeSourceInfo] = attr.ib(factory=list)
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

        has_environment = 'environment' in dict_
        if has_environment and not isinstance(dict_['environment'], dict):
            raise ValueError("expected 'environment' to be a mapping")

        has_node_sources = 'node_sources' in dict_
        if has_node_sources and not isinstance(dict_['node_sources'], list):
            raise ValueError("expected 'node_sources' to be a list")

        image: str = dict_['image']
        sources: t.Sequence[str] = dict_['sources']
        launches: t.Sequence[str] = dict_['launches']
        environment: t.Mapping[str, str] = dict(dict_.get('environment', {}))
        node_sources: t.Sequence[t.Dict[str, t.Any]] = list(dict_.get('node_sources', []))

        return Config(image=image,
                      sources=sources,
                      launches=launches,
                      environment=environment,
                      node_sources={(nsi.package_name, nsi.node_name): nsi
                                    for nsi in (NodeSourceInfo.from_dict(d) for d in node_sources)
                                    })

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
