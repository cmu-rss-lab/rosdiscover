# -*- coding: utf-8 -*-
#
# tricky bits:
# - where is a given name defined? [maintain a lookup table?]
# - alias analysis!
#
# some sort of exotic visitor is _probably_ the best way to handle this
from typing import List, Optional, Tuple

import attr
import parso


@attr.s
class Environment:
    pass


def main():
    filename = 'prepare_simulated_robot_pick_place.py'
    with open(filename, 'r') as f:
        file_contents = f.read()

    node_module = parso.parse(file_contents, version='2.7')
    environment = Environment()
    evaluate(node_module, environment)


def evaluate(node: parso.tree.NodeOrLeaf, env: Environment):
    # dispatch to appropriate method based on type
    # print(f"@ dispatching: {node.type}")
    if node.type == 'file_input':
        return evaluate_module(node, env)
    elif node.type == 'simple_stmt':
        return evaluate_simple_stmt(node, env)
    elif node.type == 'suite':
        return evaluate_suite(node, env)
    elif node.type == 'import_name':
        return evaluate_import_name(node, env)
    elif node.type == 'import_from':
        return evaluate_import_from(node, env)
    elif node.type == 'expr_stmt':
        return evaluate_expr_stmt(node, env)
    elif node.type == 'if_stmt':
        return evaluate_if_stmt(node, env)
    elif node.type in ('endmarker', 'newline'):
        return evaluate_noop(node, env)
    elif node.type == 'power':
        return evaluate_power(node, env)
    else:
        print(f"UNKNOWN TYPE: {node.type}")


def evaluate_noop(node: parso.tree.NodeOrLeaf, env: Environment):
    return


def evaluate_power(node: parso.tree.NodeOrLeaf, env: Environment):
    print(node)


def evaluate_expr_stmt(node: parso.tree.NodeOrLeaf, env: Environment):
    print(f"attempting to evaluate expr stmt: {node}")
    print(node.children)


def evaluate_if_stmt(node: parso.tree.NodeOrLeaf, env: Environment):
    # find each branch and its corresponding condition, if any.
    branches: List[Tuple[parso.tree.NodeOrLeaf, Optional[parso.tree.NodeOrLeaf]]] = []
    for index, child in enumerate(node.children):
        if child in ('if', 'elif'):
            condition = node.children[index + 1]
            branch = node.children[index + 3]
            branches.append((branch, condition))
        elif child ==  'else':
            branch = node.children[index + 2]
            branches.append((branch, None))

    for branch, condition in branches:
        # TODO attempt to evaluate the branch condition in the current
        # environment
        print(f"evaluating branch condition: {condition}")
        is_satisfied = True
        if is_satisfied:
            evaluate(branch, env)


def evaluate_import_name(node: parso.tree.NodeOrLeaf, env: Environment):
    defined_names = [name.value for name in node.get_defined_names()]
    for module_name in defined_names:
        print(f"$ importing {module_name}")


def evaluate_import_from(node: parso.tree.NodeOrLeaf, env: Environment):
    imported_names = [name.value for name in node.get_defined_names()]
    from_name = '.'.join(name.value for name in node.get_from_names())
    for module_name in imported_names:
        print(f"$ importing {module_name} from {from_name}")


def evaluate_module(node: parso.tree.NodeOrLeaf, env: Environment):
    for child in node.children:
        evaluate(child, env)


def evaluate_suite(node: parso.tree.NodeOrLeaf, env: Environment):
    for child in node.children:
        evaluate(child, env)


def evaluate_simple_stmt(node: parso.tree.NodeOrLeaf, env: Environment):
    for child in node.children:
        evaluate(child, env)


if __name__ == '__main__':
    main()
