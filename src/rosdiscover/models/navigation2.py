from ..interpreter import model, NodeContext

from .plugins.navigation2 import Nav2Plugin


@model('nav2_controller', 'controller_server')
def controller_server(c: NodeContext):
    c.read('controller_frequency', 20.0)
    progress_checker_plugin = c.read('progress_checker_plugin', 'progress_checker')
    goal_checker_plugin = c.read('goal_checker_plugin', 'goal_checker')
    controller_plugins = c.read('controller_plugins', ['FollowPath'])
    c.read('min_x_velocity_threshold', 0.0001)
    c.read('min_y_velocity_threshold', 0.0001)
    c.read('min_theta_velocity_threshold', 0.0001)

    c.sub('/odom', 'nav2_msgs/msg/Odometry')
    c.pub('/cmd_vel', 'geometry_msgs/msgs/Twist')

    c.action_server('/follow_path', )

    # Process the plugins
    progress_checker = c.read(progress_checker_plugin)
    assert isinstance(progress_checker, dict)
    pp = Nav2Plugin.from_dict(progress_checker, c.name)
    c.load_plugin(pp, c)

    goal_checker = c.read(goal_checker_plugin)
    assert isinstance(goal_checker, dict)
    gc = Nav2Plugin.from_xml(goal_checker, c.name)
    c.load_plugin(gc, c)

    if isinstance(controller_plugins, list):
        for plugin in controller_plugins:
            controller_plugin = c.read(plugin)
            assert isinstance(controller_plugin, dict)
            cp = Nav2Plugin.from_xml(controller_plugin, c.name)
            c.load_plugin(cp, c)



