__all__ = ['ParameterServer']

from typing import MutableMapping, Any


class ParameterServer(MutableMapping[str, Any]):
    """
    Models the state of the parameter server.
    """
    def __init__(self):
        # type: () -> None
        self.__contents = {}  # type: Dict[str, Any]

    def __getitem__(self, key):
        # type: (str) -> Any
        return self.__contents[key]

    def __contains__(self, key):
        # type: (str) -> bool
        return key in self.__contents

    def __setitem__(self, key, val):
        # type: (str, Any) -> None
        self.__contents[key] = val

    def get(self, key, default):
        # type: (str, Any) -> Any
        return self.__contents.get(key, default)
