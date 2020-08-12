# -*- coding: utf-8 -*-
import parso


def main():
    filename = 'prepare_simulated_robot_pick_place.py'
    with open(filename, 'r') as f:
        file_contents = f.read()

    node_module = parso.parse(file_contents, version='2.7')
    evaluate(node_module)


def evaluate(node: parso.tree.NodeOrLeaf):
    # dispatch to appropriate method based on type
    print(f"@ dispatching: {node.type}")
    if node.type == 'file_input':
        return evaluate_module(node)
    elif node.type == 'simple_stmt':
        return evaluate_simple_stmt(node)
    elif node.type == 'import_name':
        return evaluate_import_name(node)
    elif node.type == 'import_from':
        return evaluate_import_from(node)
    elif node.type == 'expr_stmt':
        return evaluate_expr_stmt(node)
    elif node.type == 'if_stmt':
        return evaluate_if_stmt(node)
    elif node.type in ('endmarker', 'newline'):
        return evaluate_noop(node)
    else:
        print(f"UNKNOWN TYPE: {node.type}")


def evaluate_noop(node: parso.tree.NodeOrLeaf):
    return


def evaluate_expr_stmt(node: parso.tree.NodeOrLeaf):
    print(f"attempting to evaluate expr stmt: {node}")


def evaluate_if_stmt(node: parso.tree.NodeOrLeaf):
    for branch_condition_node in node.get_test_nodes():
        print(f"evaluating branch condition: {branch_condition_node}")


def evaluate_import_name(node: parso.tree.NodeOrLeaf):
    defined_names = [name.value for name in node.get_defined_names()]
    for module_name in defined_names:
        print(f"$ importing {module_name}")


def evaluate_import_from(node: parso.tree.NodeOrLeaf):
    imported_names = [name.value for name in node.get_defined_names()]
    from_name = '.'.join(name.value for name in node.get_from_names())
    for module_name in imported_names:
        print(f"$ importing {module_name} from {from_name}")


def evaluate_module(node: parso.tree.NodeOrLeaf):
    for child in node.children:
        evaluate(child)


def evaluate_simple_stmt(node: parso.tree.NodeOrLeaf):
    for child in node.children:
        evaluate(child)


if __name__ == '__main__':
    main()
