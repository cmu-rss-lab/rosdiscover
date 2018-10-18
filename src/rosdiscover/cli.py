import logging

import cement

from .workspace import Workspace

logger = logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)


class BaseController(cement.Controller):
    class Meta:
        label = 'base'
        description = 'discovery of ROS architectures'
        arguments = []

    def default(self) -> None:
        self.app.args.print_help()

    @cement.ex(
        help='simulates the architectural effects of a ROS launch',
        arguments=[
            (['filename'],
             {'help': 'a ROS launch file'})
        ]
    )
    def launch(self) -> None:
        pass


class CLI(cement.App):
    class Meta:
        label = 'rosdiscover'
        handlers = [BaseController]


def main():
    log_to_stdout = logging.StreamHandler()
    log_to_stdout.setLevel(logging.DEBUG)
    logging.getLogger('rosdiscover').addHandler(log_to_stdout)

    with CLI() as app:
        app.run()
