# -*- coding: utf-8 -*-
from typing import Sequence
import logging
import time

import roswire

logger = logging.getLogger(__name__)  # type: logging.Logger
logger.setLevel(logging.DEBUG)


def launch(image: str, launch_filenames: Sequence[str]) -> None:
    rsw = roswire.ROSWire()
    with rsw.launch(image) as system:
        with system.roscore() as ros:
            for fn_launch in launch_filenames:
                logger.debug("launching: %s", fn_launch)
                ros.launch(fn_launch)
                time.sleep(5)

            for node in ros.nodes:
                print(node)
