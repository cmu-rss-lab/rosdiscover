# -*- coding: utf-8 -*-
__all__ = ('ParameterServer',)

from typing import Any, Dict, Iterator, MutableMapping, Optional


class ParameterServer(MutableMapping[str, Any]):
    """Models the state of the parameter server."""
    def __init__(self) -> None:
        self.__contents: Dict[str, Any] = {}

    def __len__(self) -> int:
        return len(self.__contents)

    def __delitem__(self, key: str) -> None:
        del self.__contents[key]

    def __iter__(self) -> Iterator[str]:
        return self.__contents.__iter__()

    def __getitem__(self, key: str) -> Any:
        return self.__contents[key]

    def __contains__(self, key: Any) -> bool:
        return isinstance(key, str) and key in self.__contents

    def __setitem__(self, key: str, val: Any) -> None:
        self.__contents[key] = val

    def get(self, key: Any, default: Optional[Any] = None) -> Optional[Any]:
        return self.__contents.get(key, default)
