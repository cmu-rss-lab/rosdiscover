from roswire import ROSVersion

from ..interpreter import model, NodeContext


@model('robot_state_publisher', 'robot_state_publisher')
def robot_state_publisher(c: NodeContext):
    if c.app.app.description.distribution.ros == ROSVersion.ROS1:
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

    elif c.app.app.description().distribution.ros == ROSVersion.ROS2:
        c.sub("/clock", "rosgraph_msgs/msg/Clock")
        c.sub('/joint_states', 'sensor_msgs/msg/JointState')

        c.pub('/robot_description', 'std_msgs/msg/String')
        c.pub('/tf', 'tf2_msgs/msg/TFMessage')
        c.pub('/tf_static', 'tf2_msgs/msgs/TFMessage')

        c.read('~ignore_timestamp', False)
        c.read('~publish_frequency', 50.0)
        c.read('~robot_description', '')
        c.read('~use_sim_time', False)
        c.read('~use_tf_static', True)
