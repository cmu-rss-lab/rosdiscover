# for now, we need to include prebaked modules
# - a node may be composed of multiple components
from typing import Dict, Iterator, Any, Optional, Tuple

from .workspace import Workspace


class NodeContext(object):
    def __init__(self,
                 name: str
                 ) -> None:
        self.__name = name

    def read(self,
             param: str,
             default: Optional[Any]
             ) -> Any:
        """
        Obtains the value of a given parameter from the parameter
        server.
        """
        raise NotImplementedError


class Model(object):
    """
    Models the architectural interactions of a node type.
    """
    _models = {}  # type: Dict[Tuple[str, str], Model]

    @staticmethod
    def register(package: str,
                 name: str,
                 definition: Callable[[NodeContext], None]
                 ) -> None:
        key = (package, name)
        models = Model._models
        if key in models:
            m = "model [{}] already registered for package [{}]"
            m.format(name, package)
            raise Exception(m)
        models[key] = Model(package, name, definition)

    @staticmethod
    def find(package: str, name: str) -> 'Model':
        return Model._models[(package, name)]

    def __init__(self,
                 package: str,
                 name: str,
                 definition: Callable[[NodeContext], None]
                 ) -> None:
        self.__package = package
        self.__name = name
        self.__definition = definition

    def eval(context: NodeContext) -> None:
        return self.__definition(context)


class VM(object):
    def __init__(self,
                 workspace: Workspace
                 ) -> None:
        pass

    def topics(self) -> Iterator[Topic]:
        return

    def nodes(self) -> Iterator[Node]:
        return

    def load(self,
             pkg: str,
             nodetype: str,
             name: str
             ) -> None:
        if nodetype == 'nodelet':
            raise Exception('nodelets are not currently supported.')

        try:
            model = Model.find(pkg, nodetype)
        except Exception:
            m = "failed to find model for node type [{}] in package [{}]"
            m = m.format(nodetype, pkg)
            raise Exception(m)

        ctx = NodeContext(name)
        model.eval(ctx)
