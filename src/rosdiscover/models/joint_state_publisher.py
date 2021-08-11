from roswire import ROSDistribution

from ..interpreter import model, NodeContext


@model('joint_state_publisher', 'joint_state_publisher')
def joint_state_publisher(c: NodeContext):
    # https://github.com/ros/joint_state_publisher/blob/kinetic-devel/joint_state_publisher/joint_state_publisher/joint_state_publisher

    # joint_state_publisher#L33
    def get_param(name, value=None):
        private = "~{}".format(name)
        return c.read(private, c.read(name, value))

    get_param("robot_description")
    get_param("dependent_joints", {})
    get_param("use_mimic_tags", True)
    get_param("use_smallest_joint_limits", True)
    get_param("zeros")

    get_param("publish_default_positions", True)
    get_param("publish_default_velocities", False)
    get_param("publish_default_efforts", False)

    if c.ros_distro < ROSDistribution.NOETIC:
        get_param("use_gui", False)

    for source in get_param("source_list", []):
        c.sub(source, 'sensor_msgs/JointState')

    c.pub('joint_states', 'sensor_msgs/JointState')
