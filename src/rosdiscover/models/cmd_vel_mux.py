from ..vm import model


@model('yocs_cmd_vel_mux', 'CmdVelMuxNodelet')
def cmd_vel_mux(c):
    # FIXME handle IO
    c.read("~yaml_cfg_file")

    # TODO FILE MUST EXIST

    # FIXME based on contents of a file
    name_output = "output_{}"

    # publishers and subscribers
    """
    for (unsigned int i = 0; i < cmd_vel_sub.size(); i++)
    {
      cmd_vel_sub[i].subs =
          nh_priv.subscribe<geometry_msgs::Twist>(cmd_vel_sub[i].topic, 10, CmdVelFunctor(i, this));

      // Create (stopped by now) a one-shot timer for every subscriber
      cmd_vel_sub[i].timer =
          nh_priv.createTimer(ros::Duration(cmd_vel_sub[i].timeout), TimerFunctor(i, this), true, false);

      NODELET_DEBUG("CmdVelMux : subscribed to '%s' on topic '%s'. pr: %d, to: %.2f",
                cmd_vel_sub[i].name.c_str(), cmd_vel_sub[i].topic.c_str(),
                cmd_vel_sub[i].priority, cmd_vel_sub[i].timeout);
    }
    """

    c.pub("active", "std_msgs/String")
