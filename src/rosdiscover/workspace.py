from typing import Set, Dict, List, Iterable
import os


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


def read_files(filenames: Iterable[str]) -> Dict[str, str]:
    contents = {}  # type: Dict[str, str]
    for fn in filenames:
        with open(fn, 'r') as f:
            contents[fn] = f.read()
    return contents


def obtain_sources(dirname: str) -> Dict[str, str]:
    cpp_files = find_cpp_files(dirname)

    # FIXME bad files
    cpp_files -= {
        '/home/chris/brass/examples/yujin_ocs/yocs_cmd_vel_mux/src/cmd_vel_subscribers.cpp',
        '/home/chris/brass/examples/rospack/src/rospack.cpp',
        '/home/chris/brass/examples/ros_comm/xmlrpcpp/test/Validator.cpp',
        '/home/chris/brass/examples/ecl_core/ecl_command_line/src/test/command_line.cpp',
        '/home/chris/brass/examples/orocos_kinematics_dynamics/orocos_kdl/tests/serialchaintest.cpp',
        '/home/chris/brass/examples/ros_comm/xmlrpcpp/test/TestXml.cpp',
        '/home/chris/brass/examples/ros_comm/xmlrpcpp/src/XmlRpcUtil.cpp',
        '/home/chris/brass/examples/geometry/tf/test/tf_unittest.cpp'
    }
    return read_files(cpp_files)
