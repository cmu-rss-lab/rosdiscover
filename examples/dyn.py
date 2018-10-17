# actionlib/src/actionlib/action_client.py
def actionlib_client():
    reads += ('actionlib_client_pub_queue_size', 10)
    # TODO: rospy.remap_name(ns)
    pubs += ('___/goal', ActionGoal)  # TODO
    pubs += ('___/cancel', GoalID)
    subs += ('___/status', GoalStatusArray)
    subs += ('___/result', ActionResult)  # TODO
    subs += ('___/feedback', ActionFeedback)


# actionlib/src/actionlib/action_server.py
def actionlib_server():
    reads += ('actionlib_server_pub_queue_size', 50)
    pubs += ('___/status', GoalStatusArray)
    pubs += ('___/result', ActionResult)
    pubs += ('___/feedback', ActionFeedback)
    subs += ('___/goal', ActionGoal)
    subs += ('___/cancel', GoalID)

    # THIS IS A WEIRD ONE
    reads += ('___/status_frequency', 5.0)  # deprecated
    reads += ('actionlib_status_frequency', 5.0)

    reads += ('___/status_list_timeout', 5.0)
