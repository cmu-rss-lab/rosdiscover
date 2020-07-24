# -*- coding: utf-8 -*-
import pytest

import os

from loguru import logger
import rosdiscover

DIR_HERE = os.path.dirname(__file__)

logger.enable('rosdiscover')


@pytest.fixture
def config(request) -> rosdiscover.Config:
    name = request.param
    filepath = os.path.join(DIR_HERE, 'configs', f'{name}.yml')
    return rosdiscover.Config.load(filepath)
