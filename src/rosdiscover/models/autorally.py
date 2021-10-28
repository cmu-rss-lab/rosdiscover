# -*- coding: utf-8 -*-
from ..interpreter import model, NodeContext

# Python models for autorally_gazebo
# Derived from: https://github.com/AutoRally/autorally/tree/melodic-devel/autorally_gazebo/nodes


@model('autorally_gazebo', 'autorally_controller.py')
def autorally_controller(c: NodeContext) -> None:

    c.read("~left_front_wheel/steering_link_name", "left_steering_link")
    c.read("~right_front_wheel/steering_link_name", "right_steering_link")
    left_steering_controller_name = \
        c.read("~left_front_wheel/steering_controller_name", "left_steering_controller")
    assert isinstance(left_steering_controller_name, str)
    right_steering_controller_name = \
        c.read("~right_front_wheel/steering_controller_name", "right_steering_controller")
    assert isinstance(right_steering_controller_name, str)
    c.read("~left_rear_wheel/link_name", "left_wheel")
    c.read("~right_rear_wheel/link_name", "right_wheel")

    left_front_axle_controller_name = \
        c.read("~left_front_wheel/axle_controller_name")
    assert isinstance(left_front_axle_controller_name, str)

    right_front_axle_controller_name = \
        c.read("~right_front_wheel/axle_controller_name")
    assert isinstance(right_front_axle_controller_name, str)

    left_rear_axle_controller_name = \
        c.read("~left_rear_wheel/axle_controller_name")
    assert isinstance(left_rear_axle_controller_name, str)

    right_rear_axle_controller_name = \
        c.read("~right_rear_wheel/axle_controller_name")
    assert isinstance(right_rear_axle_controller_name, str)

    c.read("~left_front_wheel/diameter", 1.0)
    c.read("~right_front_wheel/diameter")
    c.read("~left_rear_wheel/diameter")
    c.read("~right_rear_wheel/diameter")

    # https://github.com/AutoRally/autorally/blob/c2692f2970da6874ad9ddfeea3908adaf05b4b09/autorally_gazebo/nodes/autorally_controller.py#L258
    chassis_command_priorities = \
        c.parameter_keys("~chassisCommandProirities")  # Note, misspelling is deliberate

    shock_absorbers = c.read("~shock_absorbers", [])

    c.read("~cmd_timeout", 0.5)
    c.read("~publishing_frequency", 30.0)

    c.sub("runstop", "autorally_msgs/runstop")

    c.pub(f"{left_steering_controller_name}/command", "std_msgs/Float64")
    c.pub(f"{right_steering_controller_name}/command", "std_msgs/Float64")
    c.pub(f"{left_front_axle_controller_name}/command", "std_msgs/Float64")
    c.pub(f"{right_front_axle_controller_name}/command", "std_msgs/Float64")
    c.pub(f"{left_rear_axle_controller_name}/command", "std_msgs/Float64")
    c.pub(f"{right_rear_axle_controller_name}/command", "std_msgs/Float64")

    for shocker in shock_absorbers:
        assert isinstance(shocker, dict)
        assert 'controller_name' in shocker
        c.pub(f"{shocker['controller_name']}/command", "std_msgs/Float64")  # latched = True

    c.pub("wheelSpeeds", "autorally_msgs/wheelSpeeds")
    c.pub("chassisState", "autorally_msgs/chassisState")

    for cmd in chassis_command_priorities:
        cmd = cmd.split("/")[-1]
        c.sub(f"{cmd}/chassisCommand", "autorally_msgs/chassisCommand")

    c.sub('joint_states', "sensor_msgs/JointState")

    c.provide('~/list_controllers', "controller_manager_msgs/ListControllers")


@model('autorally_gazebo', 'ground_truth_republisher.py')
def ground_truth_republisher(c: NodeContext) -> None:
    c.pub('/ground_truth/state', 'nav_msgs/Odometry')
    c.sub('/ground_truth/state_raw', 'nav_msgs/Odometry')
