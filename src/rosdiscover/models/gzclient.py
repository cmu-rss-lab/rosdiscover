from ..interpreter import model


@model('gazebo_ros', 'gzclient')
def gzclient(c):
    # gazebo_ros/src/gazebo_ros/gazebo_interface.py
    c.use(f'{ns}/spawn_sdf_model', 'SpawnModel')
    c.use(f'{ns}/spawn_urdf_model', 'SpawnModel')
    c.use(f'{ns}/set_model_configuration', 'SetModelConfiguration')
