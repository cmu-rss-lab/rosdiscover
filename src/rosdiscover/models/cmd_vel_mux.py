import yaml
from loguru import logger

from ..interpreter import model, NodeContext


@model('yocs_cmd_vel_mux', 'CmdVelMuxNodelet')
def cmd_vel_mux(c: NodeContext):
    # FIXME handle IO
    fn = c.read("~yaml_cfg_file", None)
    if not fn:
        logger.error("Couldn't read ~yaml_cfg_file")
        return
    logger.debug(f"Reading parameters from '{fn}'")
    yml = yaml.load(c.read_file(fn))
    c.pub(f"~{yml.get('publisher', 'output')}", 'geometry_msgs/Twist')
    for sub_desc in yml['subscribers']:
        c.sub(f"~{sub_desc['topic']}", 'geometry_msgs/Twist')

    c.pub("~active", "std_msgs/String")

    # dynamic reconfigure
    c.pub('~parameter_descriptions', 'dynamic_reconfigure/ConfigDescription')
    c.pub('~parameter_updates', 'dynamic_reconfigure/Config')
