# -*- coding: utf-8 -*-
__all__ = ('ParsoVisitor',)

import parso


class ParsoVisitor:
    """Based on https://docs.python.org/3/library/ast.html#ast.NodeVisitor"""
    def visit(self, node: parso.tree.NodeOrLeaf) -> None:
        visit_method_name = f'visit_{node.type}'
        visit_method = getattr(self, visit_method_name, None)
        if callable(visit_method):
            visit_method(node)

    def generic_visit(self, node: parso.tree.NodeOrLeaf) -> None:
        for child in node.children:
            self.visit(child)
