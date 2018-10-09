from typing import Set, Dict, List, Iterable
import re
import os
import logging

import attr
import rooibos

MATCH_INIT = 'ros::init(:[argc], :[argv], :[name]);'
# TODO is public or private?
MATCH_PARAM = ':[nh].getParam(:[name], :[var]);'
# FIXME nh.advertise<:[format]>
MATCH_PUBLISHER = ':[fcall](:[name], :[size]);'
R_PUB_FUNCTION_CALL = r'^\w+\.advertise<.+>$'
MATCH_NODE_HANDLE = 'ros::NodeHandle nh(:[name]);'

logger = logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)


@attr.s(frozen=True)
class Parameter(object):
    name = attr.ib(type=str)


def unwrap(s: str) -> str:
    q = ['"', "'"]
    if s[0] in q and s[-1] in q:
        return s[1:-1]


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
    # cpp_files = [
    #     '/home/chris/brass/examples/yujin_ocs/yocs_keyop/src/keyop.cpp',
    #     '/home/chris/brass/examples/yujin_ocs/yocs_keyop/src/main.cpp'
    # ]

    # FIXME bad files
    cpp_files -= {'/home/chris/brass/examples/yujin_ocs/yocs_cmd_vel_mux/src/cmd_vel_subscribers.cpp'}
    return read_files(cpp_files)


def find_parameters(rbs: rooibos.Client,
                    sources: Dict[str, str]
                    ) -> Set[str]:
    params = set()
    for filename, source in sources.items():
        logger.debug("finding parameters in file: %s", filename)
        for match in rbs.matches(source, MATCH_PARAM):
            name = match['name'].fragment
            logger.debug("found parameter: %s", name)
            params.add(name)
    return params


def find_node_handles(rbs: rooibos.Client,
                      sources: Dict[str, str]
                      ) -> Set[str]:
    handles = set()
    for filename, source in sources.items():
        logger.debug("finding node handles in file: %s", filename)
        for match in rbs.matches(source, MATCH_NODE_HANDLE):
            name = match['name'].fragment
            logger.debug("found node handle: %s", name)
            handles.add(name)
    return handles


def find_subs(rbs: rooibos.Client,
              sources: Dict[str, str]
              ) -> Set[str]:
    MATCH_SUBSCRIBER = ':[fcall](:[name], :[size], :[callback], :[obj]);'
    R_FUNCTION_CALL = r'^\w+\.subscribe(<.+>)?$'

    subs = set()
    for filename, source in sources.items():
        logger.debug("finding subs in file: %s", filename)
        for match in rbs.matches(source, MATCH_SUBSCRIBER):
            if 'fcall' not in match.environment:
                continue
            fcall = re.match(R_FUNCTION_CALL, match['fcall'].fragment)
            if fcall is None:
                continue

            # fmt = fcall.group('fmt')
            fmt = "UNKNOWN"
            name = match['name'].fragment
            # frmt = match['format'].fragment
            logger.debug("found subscriber: %s [%s]", name, fmt)
            subs.add(name)
    return subs


def find_pubs(rbs: rooibos.Client,
              sources: Dict[str, str]
              ) -> Set[str]:
    pubs = set()
    for filename, source in sources.items():
        logger.debug("finding pubs in file: %s", filename)
        for match in rbs.matches(source, MATCH_PUBLISHER):
            if 'fcall' not in match.environment:
                continue
            fcall = re.match(R_PUB_FUNCTION_CALL, match['fcall'].fragment)
            if fcall is None:
                continue

            # fmt = fcall.group('fmt')
            fmt = "UNKNOWN"
            name = match['name'].fragment
            # frmt = match['format'].fragment
            logger.debug("found publisher: %s [%s]", name, fmt)
            pubs.add(name)
    return pubs

def find_nodes(rbs: rooibos.Client,
               sources: Dict[str, str]
               ) -> Set[str]:
    nodes = set()
    for filename, source in sources.items():
        logger.debug("finding nodes in file: %s", filename)
        for match in rbs.matches(source, MATCH_INIT):
            name = match['name'].fragment
            # frmt = match['format'].fragment
            logger.debug("found node: %s", name)
            nodes.add(name)
    return nodes


def main():
    # enable logging
    log_to_stdout = logging.StreamHandler()
    log_to_stdout.setLevel(logging.DEBUG)
    logger.addHandler(log_to_stdout)

    # get the contents of all of the files
    sources = obtain_sources('/home/chris/brass/examples')
    with rooibos.ephemeral_server(verbose=False) as rbs:
        subs = find_subs(rbs, sources)
        params = find_parameters(rbs, sources)
        pubs = find_pubs(rbs, sources)
        handles = find_node_handles(rbs, sources)
        nodes = find_nodes(rbs, sources)

    logger.info("Found subscribers: %s",
                ', '.join(sorted(s for s in subs)))
    logger.info("Found publishers: %s",
                ', '.join(sorted(p for p in pubs)))
    logger.info("Found nodes: %s",
                ', '.join(sorted(n for n in nodes)))
    logger.info("Found parameters: %s",
                ', '.join(sorted(p for p in params)))


if __name__ == '__main__':
    main()
