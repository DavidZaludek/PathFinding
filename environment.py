class Node:
    def __init__(self, row, col):
        self.col = col
        self.row = row
        self.visited = False
        self.edges = []
        return

    def add_edge(self, edge):
        self.edges.append(edge)
        return

    def __repr__(self):
        return f"{self.row},{self.col}"


class Edge:
    def __init__(self, node_a, node_b):
        self.nodeB = node_b
        self.nodeA = node_a
        return

    def __repr__(self):
        return f"{self.nodeA} -> {self.nodeB}"


class Environment:
    def __init__(self, size_x=8, size_y=8):
        # initialize grid of nodes.
        self.grid = [[Node(row, col) for row in range(size_x)] for col in range(size_y)]

        # initialize edges.
        indexes = [(row, col) for row in range(size_x) for col in range(size_y)]
        self.edges = {node_a: {node_b: self.init_edge(node_a, node_b) for node_b in indexes} for node_a in indexes}

        self.red_node = self.grid[0][0]  # red field
        self.blue_node = self.grid[size_x - 1][size_y - 1]  # blue field

        return

    def init_edge(self, node_a, node_b):
        (a_x, a_y) = node_a
        (b_x, b_y) = node_b

        if abs(a_x - b_x) + abs(a_y - b_y) == 1:
            tmp_edge = Edge(self.grid[a_x][a_y], self.grid[b_x][b_y])
            self.grid[a_x][a_y].add_edge(tmp_edge)
            return tmp_edge

        return None
