from ..interpreter import model


@model('gazebo_ros', 'spawn_model')
def spawn_model(c):
    ns_gz = '/gazebo'
    c.sub(f'{ns_gz}/model_states', 'gazebo_msgs/ModelStates')
    c.use(f'{ns_gz}/unpause_physics', 'std_srvs/Empty')
    c.use(f'{ns_gz}/delete_model', 'gazebo_msgs/DeleteModel')
    c.use(f'{ns_gz}/spawn_urdf_model_client', 'gazebo_msgs/SpawnModel')
    c.use(f'{ns_gz}/spawn_sdf_model_client', 'gazebo_msgs/SpawnModel')
    c.use(f'{ns_gz}/spawn_sdf_model_client', 'gazebo_msgs/SpawnModel')
    c.use(f'{ns_gz}/set_model_configuration', 'gazebo_msgs/SetModelConfiguration')
