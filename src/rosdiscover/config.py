# -*- coding: utf-8 -*-
__all__ = ('Config',)

from types import MappingProxyType
from typing import Any, Mapping, Sequence

import attr
import yaml


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
    workspaces: Sequence[str]
        The sequence of catkin workspaces that should be used to provide
        the application.
    environment: Mapping[str, str]
        A set of environment variables that should be used by the application.
    """
    image: str
    sources: Sequence[str]
    launches: Sequence[str]
    workspaces: Sequence[str]
    environment: Mapping[str, str] = attr.ib(factory=dict)

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
        if 'workspaces' not in dict_:
            raise ValueError("'workspaces' is undefined in configuration")

        if not isinstance(dict_['image'], str):
            raise ValueError("expected 'image' to be a string")
        if not isinstance(dict_['sources'], list):
            raise ValueError("expected 'sources' to be a list")
        if not isinstance(dict_['launches'], list):
            raise ValueError("expected 'launches' to be a list")
        if not isinstance(dict_['workspaces'], list):
            raise ValueError("expected 'workspaces' to be a list")

        has_environment = 'environment' in dict_
        if has_environment and not isinstance(dict_['environment'], dict):
            raise ValueError("expected 'environment' to be a mapping")

        image: str = dict_['image'] 
        sources: Sequence[str] = dict_['sources']
        launches: Sequence[str] = dict_['launches']
        workspaces: Sequence[str] = dict_['workspaces']
        environment: Mapping[str, str] = dict_.get('environment', {})
        return Config(image=image,
                      sources=sources,
                      workspaces=workspaces,
                      launches=launches)

    @classmethod
    def from_yaml_file(cls, filename: str) -> 'Config':
        dict_ = yaml.load(filename, Loader=yaml.SafeLoader)
        return cls.from_dict(dict_)

    def __attrs_post_init__(self) -> None:
        object.__setattr__(self, 'sources', tuple(self.sources))
        object.__setattr__(self, 'launches', tuple(self.launches))
        object.__setattr__(self, 'workspaces', tuple(self.workspaces))
        environment = MappingProxyType(self.environment.copy())
        object.__setattr__(self, 'environment', environment)
