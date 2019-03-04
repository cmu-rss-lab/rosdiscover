__all__ = ['ParameterServer']

from typing import MutableMapping, Iterator, Any


class ParameterServer(MutableMapping[str, Any]):
    """
    Models the state of the parameter server.
    """
    def __init__(self):
        # type: () -> None
        self.__contents = {}  # type: Dict[str, Any]

    def __len__(self):
        # type: () -> int
        return len(self.__contents)

    def __delitem__(self, key):
        # type: (str) -> None
        del self.__contents[key]

    def __iter__(self):
        # type: () -> Iterator[str]
        return self.__contents.__iter__()

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
