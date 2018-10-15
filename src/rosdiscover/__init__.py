from typing import Set, Dict, List, Iterable, Any, Callable
import re
import os
import logging
from concurrent.futures import ThreadPoolExecutor

import attr
import rooibos

from .version import __version__
from .workspace import obtain_sources, package_for_file
from .decls import NodeInit, ParamRead
from .extract import Extractor


# this has no effect?!
NUM_THREADS = 8

# TODO is public or private?
MATCH_PUBLISHER = ':[fcall](:[name], :[size]);'
R_PUB_FUNCTION_CALL = r'^\w+\.advertise<.+>$'
MATCH_NODE_HANDLE = 'ros::NodeHandle nh(:[name]);'

logger = logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)


def find_param_reads(rbs: rooibos.Client,
                     sources: Dict[str, str]
                     ) -> Set[ParamRead]:
    # TODO find nearest node handle
    template = ':[nh].getParam(:[name], :[var]);'
    def extractor(fn: str, m: rooibos.Match) -> ParamRead:
        param = ParamRead(name=m['name'].fragment,
                          package=package_for_file(fn),
                          defined_in_file=fn)
        logger.debug("found parameter: %s", param)
        params.add(param)
    return extract(rbs, sources, template, extractor)


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


def extract(rbs: rooibos.Client,
            sources: Dict[str, str],
            tpl: str,
            extractor: Callable[[str, rooibos.Match, Set[Any]], None]
            ) -> Set[Any]:
    def process(fn: str) -> Set[Any]:
        return set(extractor(fn, m) for m in rbs.matches(sources[fn], tpl))

    with ThreadPoolExecutor(max_workers=NUM_THREADS) as executor:
        matches = executor.map(process, sources.keys())

    s = set()
    for m in matches:
        s = s | m
    return s


def main():
    # enable logging
    log_to_stdout = logging.StreamHandler()
    log_to_stdout.setLevel(logging.DEBUG)
    logging.getLogger('rosdiscover').addHandler(log_to_stdout)

    # get the contents of all of the files
    sources = obtain_sources('/home/chris/brass/examples')

    extractor = Extractor(sources, threads=8)
    extractor.extract()

if __name__ == '__main__':
    main()
