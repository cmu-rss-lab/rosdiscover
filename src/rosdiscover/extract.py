from typing import List, Tuple, Callable, Dict
from concurrent.futures import ThreadPoolExecutor
import concurrent.futures
import re
import logging

import rooibos
from rooibos import Match

from .decls import FileDeclarations, NodeInit, ParamRead
from .workspace import Workspace

logger = logging.getLogger(__name__)  # type: logging.Logger
logger.setLevel(logging.DEBUG)


class Extractor(object):
    def __init__(self,
                 workspace: Workspace,
                 *,
                 threads: int = 8
                 ) -> None:
        self.__workspace = workspace
        self.__workers = threads

    def extract_from_file(self,
                          rbs: rooibos.Client,
                          filename: str
                          ) -> FileDeclarations:
        """
        Finds all declarations in a given file.
        """
        logger.debug("extracting declarations from file [%s]", filename)
        try:
            if filename.endswith('.py'):
                return self.extract_from_python_file(rbs, filename)
            return self.extract_from_cxx_file(rbs, filename)
        except Exception:
            logger.error("failed to extract declarations from file [%s]",
                         filename)
            raise

    def extract_from_cxx_file(self,
                              rbs: rooibos.Client,
                              filename: str
                              ) -> FileDeclarations:
        node_inits = set()
        param_reads = set()

        tpl = 'ros::init(:[argc], :[argv], :[name]);'
        for m in rbs.matches(self.__workspace[filename], tpl):
            node = NodeInit(name=m['name'].fragment,
                            defined_in_file=filename)
            logger.debug('found node: %s', node)
            node_inits.add(node)

        # TODO find nearest node handle
        # FIXME SLOWWW. needs narrowest match
        #tpl = ':[nh].getParam(:[name], :[var]);'
        #for m in rbs.matches(self.__workspace[filename], tpl):
        #    read = ParamRead(name=m['name'].fragment,
        #                     defined_in_file=filename)
        #    logger.debug("found parameter: %s", param)
        #    param_reads.add(read)

        return FileDeclarations(filename=filename,
                                node_inits=node_inits,
                                param_reads=param_reads)

    def extract_from_python_file(self,
                                 rbs: rooibos.Client,
                                 filename: str
                                 ) -> FileDeclarations:
        source = self.__workspace[filename]
        node_inits = set()
        param_reads = set()

        tpl = r'rospy.init_node\(([^\)]+)\)'
        for m in re.finditer(tpl, source):
            name = m.group(1)
            node = NodeInit(name=name,
                            defined_in_file=filename)
            logger.debug('found node: %s', node)
            node_inits.add(node)

        return FileDeclarations(filename=filename,
                                node_inits=node_inits,
                                param_reads=param_reads)

    def extract(self) -> None:
        with rooibos.ephemeral_server(verbose=False) as rbs:
            with ThreadPoolExecutor(max_workers=self.__workers) as executor:
                decls = executor.map(lambda fn: self.extract_from_file(rbs, fn),
                                     self.__workspace)
                file_to_decls = {d.filename: d for d in decls}
            logger.info("%s", file_to_decls)
