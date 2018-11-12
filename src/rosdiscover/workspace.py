from typing import Set, Dict, List, Iterable, Iterator
import os
import logging

logger = logging.getLogger(__name__)  # type: logging.Logger
logger.setLevel(logging.DEBUG)


class Workspace(object):
    """
    Provides access to the Catkin workspace located at a given directory.
    """
    @staticmethod
    def find_files_with_suffix(dirname, suffix):
        # type: (str, str) -> Set[str]
        """
        Finds all files in a given directory that end in a provided suffix.
        """
        result = set()  # type: Set[str]
        for root, _, filenames in os.walk(dirname):
            for fn in filenames:
                if fn.endswith(suffix):
                    fn = os.path.join(root, fn)
                    result.add(fn)
        return result

    def __init__(self, dir_root):
        # type: (str) -> None
        """
        Builds a Workspace for a given directory.
        """
        def read_file(fn):
            # type: (str) -> str
            logger.debug("reading file: %s", fn)
            with open(fn, 'r') as f:
                return f.read()

        self.__dir_root = dir_root

        # find C++ files
        cpp_files = Workspace.find_files_with_suffix(dir_root, '.cpp')
        logger.debug("found C++ files:\n%s",
                     '\n'.join(['* {}'.format(fn) for fn in cpp_files]))

        # find Python files
        py_files = Workspace.find_files_with_suffix(dir_root, '.py')
        logger.debug("found Python files:\n%s",
                     '\n'.join(['* {}'.format(fn) for fn in py_files]))

        # set of all source files
        filenames = cpp_files | py_files
        self.__files = {fn: read_file(fn) for fn in filenames}

    def __getitem__(self, fn):
        # type: (str) -> str
        """
        Returns the contents of a given file.

        Raises:
            KeyError: if no such file exists within this workspace.
        """
        return self.__files[fn]

    def __iter__(self):
        # type: () -> Iterator[str]:
        """
        Returns an iterator over the names of the files contained in this
        workspace.
        """
        for fn in self.__files.keys():
            yield fn

    def filenames(self):
        # type: () -> Set[str]
        """
        Returns a set of the names of the files contained within this
        workspace.
        """
        return set(self)

    def package_for_file(fn):
        # type: (str) -> str
        """
        Determines the name of the package to which a given file belongs.
        """
        d = os.path.dirname(fn)
        if os.path.exists(os.path.join(d, 'package.xml')):
            return os.path.basename(d)
        else:
            return package_for_file(d)
