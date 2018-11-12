from ..interpreter import model


@model('kobuki_safety_controller', 'SafetyControllerNodelet')
def safety_controller(c):
    c.read("~time_to_extend_bump_cliff_events", 0.0)
    c.sub("~enable", "std_msgs/Empty")
    c.sub("~disable", "std_msgs/Empty")
    c.sub("~events/bumper", 'kobuki_msgs/BumperEvent')
    c.sub("~events/cliff", 'kobuki_msgs/CliffEvent')
    c.sub("~events/wheel_drop", 'kobuki_msgs/WheelDropEvent')
    c.sub("~reset", "std_msgs/Empty")
    c.pub("~cmd_vel", "geometry_msgs/Twist")
