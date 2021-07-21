# -*- coding: utf-8 -*-
__all__ = ('Model', 'model')

from typing import Dict, Any, Tuple, Callable

from loguru import logger

from .context import NodeContext


class Model:
    """Models the architectural interactions of a node type."""
    _models: Dict[Tuple[str, str], 'Model'] = {}
    __placeholder: bool = False

    @staticmethod
    def register(package: str,
                 name: str,
                 definition: Callable[[NodeContext], None]
                 ) -> None:
        key = (package, name)
        models = Model._models
        if key in models:
            m = f"model [{name}] already registered for package [{package}]"
            raise Exception(m)
        models[key] = Model(package, name, definition)
        logger.debug(f"registered model [{name}] for package [{package}]")

    @staticmethod
    def find(package: str, name: str) -> 'Model':
        if (package, name) in Model._models:
            return Model._models[(package, name)]
        except Exception:
            m = (f"failed to find model for node type [{name}] "
                 f"in package [{package}]")
            logger.warning(m)
            ph = Model._models[('PLACEHOLDER', 'PLACEHOLDER')]
            ph.__package = package
            ph.__name = name
            ph.__placeholder = True
            return ph

    def __init__(self,
                 package,       # type: str
                 name,          # type: str
                 definition     # type: Callable[[NodeContext], None]
                 ):             # type: (...) -> None
        self.__package = package
        self.__name = name
        self.__definition = definition
        self.__placeholder = False

    def eval(self, context: NodeContext) -> None:
        return self.__definition(context)

    def mark_placeholder(self):
        self.__placeholder = True


def model(package: str, name: str) -> Any:
    def register(m: Callable[[NodeContext], None]) -> Any:
        Model.register(package, name, m)
        return m
    return register
