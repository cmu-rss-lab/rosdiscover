# -*- coding: utf-8 -*-
from typing import Set
import re
import logging

import attr
import roswire

logger: logging.Logger = logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)

RE_PY_INIT = re.compile(r'rospy.init_node\(([^\)]+)\)')
RE_PY_PUB = re.compile(r'rospy.Publisher\(([^\)]+)\)')
RE_PY_SUB = re.compile(r'rospy.Subscriber\(([^\)]+)\)')


@attr.s(frozen=True)
class Expr:
    text: str = attr.ib()

    def eval_to_string(self) -> str:
        if self.text[0] == '"' and self.text[-1] == '"':
            return self.text[1:-1]
        if self.text[0] == "'" and self.text[-1] == "'":
            return self.text[1:-1]
        raise ValueError(f'unable to evaluate expression: {self.text}')


@attr.s(frozen=True)
class NodeInit:
    name: str = attr.ib()
    filename: str = attr.ib()


@attr.s(frozen=True)
class Extractor:
    system: roswire.System = attr.ib()

    def extract_from_python_file(self, filename: str) -> None:
        logger.debug("extracting declarations from Python file: %s", filename)
        node_inits: Set[NodeInit] = set()

        text = self.system.files.read(filename)
        logger.debug("file contents: %s", text)

        # find nodes
        for m in RE_PY_INIT.finditer(text):
            m_text = m.group(0)
            logger.debug("found node init call: %s", m_text)
            args_text = m.group(1)
            logger.debug("found call args: %s", args_text)

            expr_name = Expr(args_text.partition(',')[0])
            name = expr_name.eval_to_string()
            node_init = NodeInit(name, filename)
            logger.debug("extracted node init: %s", node_init)
            node_inits.add(node_init)

        # find publishers
        for m in RE_PY_PUB.finditer(text):
            m_text = m.group(0)
            logger.debug("found publisher call: %s", m_text)
            args_text = m.group(1)
            logger.debug("found call args: %s", args_text)

        # find subscribers
        for m in RE_PY_SUB.finditer(text):
            m_text = m.group(0)
            logger.debug("found subscriber call: %s", m_text)
            args_text = m.group(1)
            logger.debug("found call args: %s", args_text)
