from typing import List, Tuple, Callable, Dict
from concurrent.futures import ThreadPoolExecutor
import concurrent.futures
import logging

import rooibos
from rooibos import Match

from .decls import NodeInit
from .workspace import package_for_file

logger = logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)


class Extractor(object):
    def __init__(self,
                 workspace: Dict[str, str],
                 *,
                 threads: int = 8
                 ) -> None:
        self.__workspace = workspace
        self.param_reads = set()
        self.param_writes = set()
        self.publishers = set()
        self.subscribers = set()
        self.node_inits = set()
        self.__workers = threads

    def extract(self) -> None:
        extractors = []  # type: List[Tuple[str, Callable[[str, Match], None]]]

        t = 'ros::init(:[argc], :[argv], :[name]);'
        def x(fn, m):
            node = NodeInit(name=m['name'].fragment,
                            defined_in_file=fn)
            logger.debug('found node: %s', node)
            self.node_inits.add(node)
        extractors += [(t, x)]

        with rooibos.ephemeral_server(verbose=False) as rbs:
            def scan(fn: str,
                     tpl: str,
                     callback: Callable[[str, Match], None]
                     ) -> None:
                logger.debug("finding [%s] in file [%s]", tpl, fn)
                for m in rbs.matches(self.__workspace[fn], tpl):
                    logger.debug("found match")
                    callback(fn, m)

            futures = set()
            with ThreadPoolExecutor(max_workers=self.__workers) as executor:
                for fn in self.__workspace:
                    for tpl, callback in extractors:
                        fut = executor.submit(scan, fn, tpl, callback)
                        futures.add(fut)
                concurrent.futures.wait(futures)
