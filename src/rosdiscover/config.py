# -*- coding: utf-8 -*-
__all__ = ('Config',)

from types import MappingProxyType
from typing import Any, Mapping, Sequence

from loguru import logger
import attr
import roswire
import yaml

from .launch import Launch


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
    launches: Sequence[Launch]
        The sequence of Launch objects that should be used to launch the
        application.
    environment: Mapping[str, str]
        A set of environment variables that should be used by the application.
    app: roswire.app.App
        The ROSWire application for this configuration.
    """
    image: str
    sources: Sequence[str]
    launches: Sequence[Launch]
    environment: Mapping[str, str] = attr.ib(factory=dict)
    app: roswire.app.App = attr.ib(init=False)

    @classmethod
    def from_dict(cls, dict_: Mapping[str, Any]) -> 'Config':
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
            raise ValueError("expected elements of 'launches' to be str or dict")
            
        has_environment = 'environment' in dict_
        if has_environment and not isinstance(dict_['environment'], dict):
            raise ValueError("expected 'environment' to be a mapping")

        image: str = dict_['image']
        sources: Sequence[str] = dict_['sources']
        launches_inputs: Sequence[Any] = dict_['launches']
        if launch_args_provided: 
            launches = list(map(lambda d: Launch.from_dict(d), launches_inputs))
        else: 
            launches = list(map(lambda s: Launch(filename = s, arguments = dict()), launches_inputs))
        logger.debug(f'getArgV: {launches[0].get_argv()}')
        environment: Mapping[str, str] = dict(dict_.get('environment', {}))
        return Config(image=image,
                      sources=sources,
                      launches=launches,
                      environment=environment)

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
