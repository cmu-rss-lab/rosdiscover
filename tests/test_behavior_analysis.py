import unittest
import subprocess

from rosdiscover.acme import AcmeGenerator
from rosdiscover.config import Config
from rosdiscover.interpreter import Interpreter, SystemSummary
from rosdiscover.observer import Observer
from rosdiscover.recover import NodeRecoveryTool
from rosdiscover.recover.call import RateSleep
from rosdiscover.recover.model import CMakeListsInfo, RecoveredNodeModel

class TestStringMethods(unittest.TestCase):

    autoware_file = "/home/tdurschm/rosdiscover-evaluation/experiments/recovery/subjects/autoware/experiment.yml"
    turtlebot_file = "/home/tdurschm/rosdiscover-evaluation/experiments/recovery/subjects/turtlebot/experiment.yml"

    def get_model(self, config_path: str, package:str, node:str) -> RecoveredNodeModel:
        config = Config.load(config_path)
        with NodeRecoveryTool.for_config(config) as tool:
            print(f"spun up container: {tool}")
            return tool.recover_using_cmakelists(package, node)

    def assert_publish_calls(self, model, publishers):
        publish_calls = set()
        for p in model.program.publish_calls:
            publish_calls.add(p.publisher)
        
        self.assertSetEqual(publish_calls, publishers)

    def assert_publish_calls_in_sub_callback(self, model, publishers):
        publish_calls_in_sub_callback = set()
        for p in model.program.publish_calls_in_sub_callback:
            publish_calls_in_sub_callback.add(p.publisher)
        
        self.assertSetEqual(publish_calls_in_sub_callback, publishers)

    def assert_sub_callbacks(self, model, callbacks):
        sub_callback = set()
        for c in model.program.subscriber_callbacks:
            sub_callback.add(c.name)
        
        self.assertSetEqual(sub_callback, callbacks)

    def assert_rate_sleeps(self, model, sleeps):
        rate_sleeps = set()
        for r in model.program.rate_sleeps:
            rate_sleeps.add(r.rate.value)
        
        self.assertSetEqual(rate_sleeps, sleeps)        

    def test_velocity_set(self):
        model = self.get_model(self.autoware_file, "astar_planner", "velocity_set")

        self.assert_publish_calls_in_sub_callback(model,set())

        self.assert_rate_sleeps(model, {10.0})

        self.assert_sub_callbacks(model, 
            {
                "VelocitySetPath::waypointsCallback", 
                "VelocitySetPath::currentVelocityCallback",
                "VelocitySetInfo::configCallback",
                "VelocitySetInfo::pointsCallback",
                "VelocitySetInfo::localizerPoseCallback",
                "VelocitySetInfo::controlPoseCallback",
                "VelocitySetInfo::obstacleSimCallback",
                "VelocitySetInfo::detectionCallback",
                "CrossWalk::crossWalkCallback",
                "CrossWalk::areaCallback",
                "CrossWalk::lineCallback",
                "CrossWalk::pointCallback",
            }
        )

    def test_obj_reproj(self):
        model = self.get_model(self.autoware_file, "obj_reproj", "obj_reproj")

        self.assert_publish_calls_in_sub_callback(
            model,
            {'pub', 'marker_pub', 'jsk_bounding_box_pub'}
        )

        self.assert_rate_sleeps(model,set())

        self.assert_sub_callbacks(model, 
            {
                "obj_pos_xyzCallback", 
                "projection_callback",
                "camera_info_callback"
            }
        )

    def test_wf_simulator(self):
        model = self.get_model(self.autoware_file, "waypoint_follower", "wf_simulator")

        self.assert_publish_calls(
            model,
            {'odometry_publisher_', 'velocity_publisher_'}
        )

        self.assert_rate_sleeps(model, {50.0})

        self.assert_sub_callbacks(model, 
            {
                "(anonymous namespace)::CmdCallBack", 
                "(anonymous namespace)::controlCmdCallBack",
                "(anonymous namespace)::waypointCallback",
                "(anonymous namespace)::callbackFromClosestWaypoint",
                "(anonymous namespace)::initialposeCallback",
                "(anonymous namespace)::callbackFromPoseStamped",
                "(anonymous namespace)::callbackFromPoseStamped"
            }
        )

    def test_turtlebot_move_action_server(self):
        model = self.get_model(self.turtlebot_file, "turtlebot_actions", "turtlebot_move_action_server")

        self.assert_publish_calls_in_sub_callback(
            model,
            set()
        )

        self.assert_rate_sleeps(model, {25.0})

        self.assert_sub_callbacks(model, 
            set()
        )    
if __name__ == '__main__':
    unittest.main()
