# -*- coding: utf-8 -*-
import logging
from .version import __version__
from .interpreter import Model, Interpreter
from . import models

logging.getLogger(__name__).setLevel(logging.DEBUG)
