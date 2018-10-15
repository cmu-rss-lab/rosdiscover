from typing import List, Tuple, Callable, Dict
from concurrent.futures import ThreadPoolExecutor
import concurrent.futures
import logging

import rooibos
from rooibos import Match

from .decls import FileDeclarations, NodeInit
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
        self.__workers = threads

    def extract_from_file(self,
                          rbs: rooibos.Client,
                          filename: str
                          ) -> FileDeclarations:
        """
        Finds all declarations in a given file.
        """
        if filename.endswith('.py'):
            return self.extract_from_python_file(rbs, filename)
        return self.extract_from_cxx_file(rbs, filename)

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

        return FileDeclarations(filename=filename,
                                node_inits=node_inits,
                                param_reads=param_reads)

    def extract_from_python_file(self,
                                 filename: str
                                 ) -> FileDeclarations:
        raise NotImplementedError

    def extract(self) -> None:
        with rooibos.ephemeral_server(verbose=False) as rbs:
            with ThreadPoolExecutor(max_workers=self.__workers) as executor:
                filenames = self.__workspace.keys()
                decls = executor.map(lambda fn: self.extract_from_file(rbs, fn),
                                     filenames)
                file_to_decls = {d.filename: d for d in decls}
            logger.info("%s", file_to_decls)
