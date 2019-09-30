from ..interpreter import model


@model('pdw_turtlebot3_teleop', 'turtlebot3_teleop')
def pdw_turtlebot3_teleop(c):
    c.pub('cmd_vel', 'geometry_msgs/Twist')
    model = c.read('model', 'burger')
