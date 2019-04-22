"""Alpha-beta Pruning.

Author: GAO Yixu

Input: two lines
    First line: two integers, rule and n
        rule: int, 0 or 1, 1 for MAX node and 0 for MIN node
        n: int, the depth of the tree, including leafs
    Second line: a nested list stands for the game tree

    E.g.
        1 4
        [[[-20,12],[8,16]],[[-18,-7],[-9,-1]]]

Output: two lines
    First line: float, the result for mini-max search
    Second line: floats, pruned nodes in order

    E.g.
        12
        -9 -1

Usage:
    Command line: python alpha_beta_pruning.py < test.in.txt > test.out.txt
"""


class Node:
    """Node of the tree.

    Attributes:
        rule: int, 0 or 1, 1 for MAX node and 0 for MIN node
        successor: list of Node representing children of the current node
        is_leaf: bool, whether the node is a leaf or not
        value: value of the node
        visited: bool, visited or not

    Hint:
        We use this class to construct a tree in construct_tree method.
    """
    def __init__(self, rule=0, successor=None, is_leaf=False, value=None):
        if successor is None:
            successor = []
        self.rule = 'max' if rule == 1 else 'min'
        self.successor = successor
        self.is_leaf = is_leaf
        self.value = value
        self.visited = False


def get_value(node, alpha, beta):
    """Get value for the given node.

    Args:
        node: class Node object
        alpha: float
        beta: float

    Returns:
        value of the node

    Hint:
        You can use max_value and min_value to construct this function.
    """
    # TODO: Begin your code
    # TODO: End your code


def max_value(node, alpha, beta):
    """Get value for the given MAX node.

    Args:
        node: class Node object
        alpha: float
        beta: float

    Returns:
        value of the node

    Hint:
        You can use get_value to construct this function.
    """
    # TODO: Begin your code
    # TODO: End your code


def min_value(node, alpha, beta):
    """Get value for the given MIN node.

    Args:
        node: class Node object
        alpha: float
        beta: float

    Returns:
        value of the node

    Hint:
        You can use get_value to construct this function.
    """
    # TODO: Begin your code
    # TODO: End your code


def get_unvisited_nodes(node):
    """Get unvisited nodes for the tree.

    Args:
        node: class Node object, root node of the current tree (or leaf)

    Returns:
        float list of values of the unvisited nodes.
    """
    unvisited = []
    if node.successor:
        for successor in node.successor:
            unvisited += get_unvisited_nodes(successor)
    else:
        if not node.visited:
            unvisited.append(node.value)
    return unvisited


def construct_tree(n, tree, rule):
    """Construct a tree using given information and return the root node.

    Args:
        n: int, the height of tree
        tree: the input tree described with list nested structure
        rule: int, root node's type, 1 for max, 0 for min

    Returns:
        root node

    Hint: tree structure example
        root_node:
            rule: 1 (MAX node)
            is_leaf: False
            value: 5
            visited: bool, visited or not
            successor: [child1, child2, child3, ...]
                and each child has similar structure of root_node
    """
    node = Node(rule=rule)
    successors = []
    if n == 1:  # leaf
        for t in tree:
            successors.append(Node(rule=1-rule, is_leaf=True, value=t))
    else:  # sub-tree
        for t in tree:
            successors.append(construct_tree(n-1, t, 1-rule))
    node.successor = successors
    return node


def main():
    while True:
        try:
            rule, n = map(int, input().strip().split())
            tree = eval(input().strip())
            root_node = construct_tree(n-1, tree, rule)
            print(get_value(root_node, float("-inf"), float("inf")))
            # print out unvisited nodes
            print(' '.join(
                [str(node) for node in get_unvisited_nodes(root_node)]))
        except EOFError:
            break


if __name__ == '__main__':
    main()
