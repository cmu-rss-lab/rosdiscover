import yaml

from ..interpreter import model


@model('yocs_cmd_vel_mux', 'CmdVelMuxNodelet')
def cmd_vel_mux(c):
    # FIXME handle IO
    fn = c.read("~yaml_cfg_file", None)

    # TODO ensure that file exists
    with open(fn, 'r') as f:
        yml = yaml.load(f)

    c.pub(yml.get('publisher', 'output'), 'geometry_msgs/Twist')
    for sub_desc in yml['subscribers']:
        c.sub(sub_desc['topic'], 'geometry_msgs/Twist')

    c.pub("active", "std_msgs/String")

    # dynamic reconfigure
    c.pub('~parameter_descriptions', 'dynamic_reconfigure/ConfigDescription')
    c.pub('~parameter_updates', 'dynamic_reconfigure/Config')
