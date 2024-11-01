from dataclasses import dataclass
from typing import Dict, Set


@dataclass
class UnionFindNode:
    """Class for keeping Union Find Node data"""

    value: int  # Value of node
    parent: int  # Parent node value
    rank: int  # Higher bound on height


class UnionFind:
    def __init__(self):
        self.nodes: Dict[int, UnionFindNode] = {}
        self.roots: Set[int] = set()

    def make_set(self, x: int) -> None:
        if x not in self.nodes:
            self.nodes[x] = UnionFindNode(x, x, 0)
            self.roots.add(x)

    def find(self, x: int) -> int:
        """Return root of set x belongs to. None if x does not exist in any set

        Parameter
        ---------
        x: int
            id of node to get root of

        Return
        -------
        root: int
            id of root of set containing node id x

        """
        # Check if x exist in any set
        if x not in self.nodes:
            raise ValueError(f'Key {x} not found')

        # Search for root and path compression
        cur_node = self.nodes[x]
        while cur_node.parent != cur_node.value:
            parent_node = self.nodes[cur_node.parent]
            cur_node.parent = self.nodes[
                parent_node.parent
            ].value  # update to grand parent
            cur_node = parent_node

        return cur_node.value

    def union(self, x: int, y: int) -> None:
        a = self.nodes[self.find(x)]
        b = self.nodes[self.find(y)]

        if a.value != b.value:
            # Union if x and y is not part of same set
            if a.rank < b.rank:
                a, b = b, a  # Swap to make a at least as large as b
                x, y = y, x

            b.parent = a.value  # Re-parent

            # b is not root anymore
            # print(f'[Union {x},{y}] {a.value=} {b.value=} Set before: {self.roots}')
            self.roots.remove(b.value)
            # self.roots.remove(y)
            # print(f'[Union {x},{y}] {a.value=} {b.value=} Set After: {self.roots}')

            if a.rank == b.rank:
                a.rank += 1  # Re calculate rank

    def get_roots(self) -> Set[int]:
        return self.roots
