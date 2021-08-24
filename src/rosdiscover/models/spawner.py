# -*- coding: utf-8 -*-
from loguru import logger

from ..interpreter import model


@model('controller_manager', 'spawner')
def spawner(c):
    logger.warning("Missing handwritten model for controller_manager/spawner")
