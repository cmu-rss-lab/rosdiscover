# -*- coding: utf-8 -*-
import typing as t
import xml.dom.minidom as dom

import attr
from loguru import logger


@attr.s(frozen=True, auto_attribs=True)
class NodeletLibrary:
    path: str = attr.ib()
    class_name: str = attr.ib()
    class_type: str = attr.ib()
    base_class: str = attr.ib()
    description: t.Optional[str] = attr.ib()


@attr.s(frozen=True, auto_attribs=True)
class NodeletInfo:
    libraries: t.Collection['NodeletLibrary']

    @classmethod
    def from_nodelet_xml(cls, contents: str) -> 'NodeletInfo':
        libraries: t.Set['NodeletLibrary'] = set()
        contents = "<root>\n" + contents + "\n</root>"
        logger.debug(f"nodelet_plugin_xml = {contents=}")
        tree = dom.parseString(contents)
        root = NodeletInfo.get_xml_nodes_by_name('root', tree)[0]
        libraries_dom = NodeletInfo.get_xml_nodes_by_name('library', root)
        logger.debug(f"{len(libraries_dom)}")
        for library_dom in libraries_dom:
            path = library_dom.getAttribute('path')
            class_doms = NodeletInfo.get_xml_nodes_by_name('class', library_dom)
            assert len(class_doms) == 1
            class_dom = class_doms[0]
            class_name = class_dom.getAttribute('name')
            class_type = class_dom.getAttribute('type')
            base_class = class_dom.getAttribute('base_class_type')
            description_dom = NodeletInfo.get_xml_nodes_by_name('description', class_dom)
            description = None
            if len(description_dom) == 1:
                description = "\n".join(n.data for n in description_dom[0].childNodes if n.nodeType == n.TEXT_NODE)
            libraries.add(NodeletLibrary(path=path,
                                         class_name=class_name,
                                         class_type=class_type,
                                         base_class=base_class,
                                         description=description))
        return NodeletInfo(libraries=libraries)

    @classmethod
    def get_xml_nodes_by_name(cls, tag_name, tree):
        return [n for n in tree.childNodes if n.nodeType == n.ELEMENT_NODE and n.tagName == tag_name]
