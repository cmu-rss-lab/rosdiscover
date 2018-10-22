from typing import Set, Dict, List, Iterable, Iterator
import os
import logging

logger = logging.getLogger(__name__)  # type: logging.Logger
logger.setLevel(logging.DEBUG)


class Workspace(object):
    @staticmethod
    def find_files_with_suffix(dirname, suffix):
        # type: (str, str) -> Set[str]
        result = set()  # type: Set[str]
        for root, _, filenames in os.walk(dirname):
            for fn in filenames:
                if fn.endswith(suffix):
                    fn = os.path.join(root, fn)
                    result.add(fn)
        return result

    def __init__(self, dir_root):
        # type: (str) -> None

        def read_file(fn):
            # type: (str) -> str
            logger.debug("reading file: %s", fn)
            with open(fn, 'r') as f:
                return f.read()

        self.__dir_root = dir_root

        cpp_files = Workspace.find_files_with_suffix(dir_root, '.cpp')
        logger.debug("found C++ files:\n%s",
                     '\n'.join(['* {}'.format(fn) for fn in cpp_files]))

        py_files = Workspace.find_files_with_suffix(dir_root, '.py')
        logger.debug("found Python files:\n%s",
                     '\n'.join(['* {}'.format(fn) for fn in py_files]))

        # FIXME bad files
        cpp_files -= {
            '/home/chris/brass/examples/yujin_ocs/yocs_cmd_vel_mux/src/cmd_vel_subscribers.cpp',
            '/home/chris/brass/examples/rospack/src/rospack.cpp',
            '/home/chris/brass/examples/ros_comm/xmlrpcpp/test/Validator.cpp',
            '/home/chris/brass/examples/ecl_core/ecl_command_line/src/test/command_line.cpp',
            '/home/chris/brass/examples/orocos_kinematics_dynamics/orocos_kdl/tests/serialchaintest.cpp',
            '/home/chris/brass/examples/ros_comm/xmlrpcpp/test/TestXml.cpp',
            '/home/chris/brass/examples/ros_comm/xmlrpcpp/src/XmlRpcUtil.cpp',
            '/home/chris/brass/examples/geometry/tf/test/tf_unittest.cpp',

            '/home/chris/brass/examples/orocos_kinematics_dynamics/orocos_kdl/src/jntarray.cpp',
            '/home/chris/brass/examples/ros_comm/roscpp/src/libros/subscriber.cpp',
            '/home/chris/brass/examples/orocos_kinematics_dynamics/orocos_kdl/models/kukaLWRtestHCG.cpp',
        }
        py_files -= {
            '/home/chris/brass/examples/rocon_tools/rocon_python_wifi/src/rocon_python_wifi/flags.py',
            '/home/chris/brass/examples/rocon_multimaster/rocon_hub/src/rocon_hub/watcher.py',
            '/home/chris/brass/examples/rocon_multimaster/rocon_gateway/src/rocon_gateway/graph.py',
            '/home/chris/brass/examples/ros_comm/rosgraph/test/test_rosgraph_masterapi_offline.py',
            '/home/chris/brass/examples/ros_comm/rosbag/scripts/fix_md5sums.py',
            '/home/chris/brass/examples/rocon_multimaster/rocon_gateway/src/rocon_gateway/public_interface.py',
            '/home/chris/brass/examples/rocon_tools/rocon_ebnf/examples/sqs7.py',

            '/home/chris/brass/tbot/rocon_tools/rocon_python_wifi/src/rocon_python_wifi/flags.py'
        }

        filenames = cpp_files | py_files
        self.__files = {fn: read_file(fn) for fn in filenames}

    def __getitem__(self, fn):
        # type: (str) -> str
        return self.__files[fn]

    def __iter__(self):
        # type: () -> Iterator[str]:
        for fn in self.__files.keys():
            yield fn

    def filenames(self):
        # type: () -> Set[str]
        return set(self)

    def package_for_file(fn):
        # type: (str) -> str
        d = os.path.dirname(fn)
        if os.path.exists(os.path.join(d, 'package.xml')):
            return os.path.basename(d)
        else:
            return package_for_file(d)
