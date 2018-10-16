from typing import Set, Dict, List, Iterable
import os
import logging

logger = logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)


def package_for_file(fn: str) -> str:
    d = os.path.dirname(fn)
    if os.path.exists(os.path.join(d, 'package.xml')):
        return os.path.basename(d)
    else:
        return package_for_file(d)


def find_cpp_files(dirname: str) -> Set[str]:
    result = set()  # type: Set[str]
    for root, _, filenames in os.walk(dirname):
        for fn in filenames:
            if fn.endswith('.cpp'):
                fn = os.path.join(root, fn)
                result.add(fn)
    return result


def find_py_files(dirname: str) -> Set[str]:
    result = set()  # type: Set[str]
    for root, _, filenames in os.walk(dirname):
        for fn in filenames:
            if fn.endswith('.py'):
                fn = os.path.join(root, fn)
                result.add(fn)
    return result


def read_files(filenames: Iterable[str]) -> Dict[str, str]:
    contents = {}  # type: Dict[str, str]
    for fn in filenames:
        with open(fn, 'r') as f:
            logger.debug("reading file: %s", fn)
            contents[fn] = f.read()
    return contents


def obtain_sources(dirname: str) -> Dict[str, str]:
    cpp_files = find_cpp_files(dirname)
    py_files = find_py_files(dirname)

    logger.debug("found C++ files: %s",
                 '\n'.join(['* {}'.format(fn) for fn in cpp_files]))
    logger.debug("found Python files: %s",
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
        '/home/chris/brass/examples/rocon_tools/rocon_ebnf/examples/sqs7.py'
    }
    return read_files(py_files)
    # return read_files(cpp_files | py_files)
