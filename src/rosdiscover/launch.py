# -*- coding: utf-8 -*-
__all__ = ('Launch',)

from typing import Any, Mapping

import attr


# Data class storing the file name of a launch file and its arguments
@attr.s(frozen=True, slots=True, auto_attribs=True)
class Launch:
    """
    Attributes
    ----------
    filename: str
        The file name of the launch files that should be launched
    arguments: Mapping[str, str]
        A set of launch arguments that should be passed to roslaunch
    """
    filename: str
    arguments: Mapping[str, str]

    def get_argv(self) -> List[str]:
        return [f'{argk}:={self.arguments.get(argk)}' for argk in self.arguments.keys()]

    @classmethod
    def from_dict(cls, dict_: Mapping[str, Any]) -> 'Launch':
        """
        Raises
        ------
        ValueError
            If 'filename' is undefined in configuration.
        """
        if 'filename' not in dict_:
            raise ValueError("'filename' is undefined in configuration")

        if not isinstance(dict_['filename'], str):
            raise ValueError("expected 'filename' to be a string")

        has_arguments = 'arguments' in dict_
        if has_arguments and not isinstance(dict_['arguments'], dict):
            raise ValueError("expected 'arguments' to be a mapping")

        filename: Optional[Any] = dict_.get('filename')
        arguments: Mapping[str, str] = dict(dict_.get('arguments', {}))
        return Launch(filename=filename,
                      arguments=arguments)
