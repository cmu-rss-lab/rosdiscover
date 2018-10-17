import logging

from rosdiscover.workspace import Workspace
from rosdiscover.vm import Model, NodeContext, VM


def test_joint_state_publisher():
    import rosdiscover.models.joint_state_publisher
    m = Model.find('joint_state_publisher', 'joint_state_publisher')
    ctx = NodeContext('joint_state_publisher')
    m.eval(ctx)


def test_amcl():
    import rosdiscover.models.amcl
    m = Model.find('amcl', 'amcl')
    ctx = NodeContext('amcl')
    m.eval(ctx)


def test_launch():
    workspace = Workspace('/home/chris/brass/tbot')
    fn_launch = "/home/chris/brass/tbot/turtlebot_simulator/launch/turtlebot_in_stage.launch"
    vm = VM(workspace)
    vm.launch(fn_launch)


def test_fake_launch():
    import rosdiscover.models.amcl
    import rosdiscover.models.joint_state_publisher
    import rosdiscover.models.move_base
    import rosdiscover.models.stage_ros

    workspace = Workspace('/home/chris/brass/tbot')
    vm = VM(workspace)
    vm.load('stage_ros', 'stage_ros', 'stage_ros')
    vm.load('joint_state_publisher', 'joint_state_publisher', 'joint_state_publisher')
    vm.load('amcl', 'amcl', 'amcl')
    vm.load('move_base', 'move_base', 'move_base')

    # FIXME nodelets
    vm.load('yocs_velocity_smoother',
            'velocity_smoother',
            'navigation_velocity_smoother')
    vm.load('kobuki_safety_controller',
            'kobuki_safety_controller',
            'safety_controller')

    print(list(vm.nodes))
    # print(list(vm.topics))


if __name__ == '__main__':
    log_to_stdout = logging.StreamHandler()
    logging.getLogger('rosdiscover').addHandler(log_to_stdout)

    # test_joint_state_publisher()
    # test_amcl()

    test_fake_launch()
