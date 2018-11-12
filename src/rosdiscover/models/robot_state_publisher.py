from ..interpreter import model


@model('robot_state_publisher', 'robot_state_publisher')
def robot_state_publisher(c):
    # FIXME required!
    # c.read('robot_description')

    c.pub('/tf', 'tf2_msgs/TFMessage')
    c.pub('/tf_static', 'tf2_msgs/TFMessage')

    c.read('~publish_frequency', 50.0)
    c.read('~use_tf_static', True)
    c.read('~ignore_timestamp', False)

    # FIXME implement searchParam
    # http://wiki.ros.org/roscpp/Overview/Parameter%20Server
    # tf_prefix_key = c.search('~tf_prefix')
    # c.read(tf_prefix_key, '')

    c.sub('joint_states', 'sensor_msgs/JointState')
