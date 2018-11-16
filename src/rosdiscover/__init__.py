import logging
logger = logging.getLogger(__name__)  # type: logging.Logger
logger.setLevel(logging.DEBUG)
logging.basicConfig()

from .version import __version__
from .workspace import Workspace
from .interpreter import Model, Interpreter
from . import models
