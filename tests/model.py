import logging

from rosdiscover.vm import Model, NodeContext


def test_joint_state_publisher():
    import rosdiscover.models.joint_state_publisher
    m = Model.find('joint_state_publisher', 'joint_state_publisher')
    ctx = NodeContext('joint_state_publisher')
    m.eval(ctx)


if __name__ == '__main__':
    log_to_stdout = logging.StreamHandler()
    logging.getLogger('rosdiscover').addHandler(log_to_stdout)

    test_joint_state_publisher()
