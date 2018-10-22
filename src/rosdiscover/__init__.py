import logging

from .version import __version__
from .workspace import Workspace
from .vm import Model, VM
from . import models

logger = logging.getLogger(__name__)  # type: logging.Logger
logger.setLevel(logging.DEBUG)
