# -*- coding: utf-8 -*-
__all__ = ('Config',)

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
    """
    image: str
    sources: Sequence[str]
    launches: Sequence[str]

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

        image: str = dict_['image'] 
        sources: Sequence[str] = dict_['sources']
        launches: Sequence[str] = dict_['launches']
        return Config(image=image,
                      sources=sources,
                      launches=launches)

    @classmethod
    def from_yaml_file(cls, filename: str) -> 'Config':
        dict_ = yaml.load(args.config, Loader=yaml.SafeLoader)
        return cls.from_dict(dict_)

    def __attrs_post_init__(self) -> None:
        object.__setattr__(self, 'sources', tuple(self.sources))
        object.__setattr__(self, 'launches', tuple(self.launches))
