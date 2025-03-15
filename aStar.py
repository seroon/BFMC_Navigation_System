import heapq
import config
from parser import parse_graphml


def astar(graph_file, start, goal, heuristic):
    """
    A* algorithm to find the shortest path in a graph.

    Args:
        graph_file: Path to the GraphML file describing the graph.
        start: ID of the start node.
        goal: ID of the goal node.
        heuristic: Function to estimate cost from a node to the goal.

    Returns:
        A tuple containing the shortest path and its total cost.
    """
    # Parse the graph
    result = parse_graphml(graph_file)
    if not result:
        raise ValueError("Failed to parse the graph file.")

    nodes = result['nodes']
    edges = result['edges']

    # Initialize data structures
    open_set = []  # Priority queue for nodes to explore
    heapq.heappush(open_set, (0, start))  # (priority, node_id)
    came_from = {}  # Tracks the path
    g_score = {node: float('inf') for node in nodes}
    g_score[start] = 0
    f_score = {node: float('inf') for node in nodes}
    f_score[start] = heuristic(start, goal, nodes)

    while open_set:
        _, current = heapq.heappop(open_set)

        # Check if the goal is reached
        if current == goal:
            path = reconstruct_path(came_from, current)
            return path, g_score[goal]

        # Explore neighbors
        for edge in edges:
            source, target, _ = edge
            if source != current:
                continue

            # Tentative g_score for neighbor
            tentative_g_score = g_score[current] + distance_between(nodes, current, target)
            if tentative_g_score < g_score[target]:
                came_from[target] = current
                g_score[target] = tentative_g_score
                f_score[target] = tentative_g_score + heuristic(target, goal, nodes)
                if target not in [n[1] for n in open_set]:
                    heapq.heappush(open_set, (f_score[target], target))

    raise ValueError("No path found from start to goal.")


def astar_with_mandatory_nodes(graph_file, start, goal, mandatory_nodes, heuristic):
    """
    A* algorithm to find the shortest path in a graph including mandatory nodes.

    Args:
        graph_file: Path to the GraphML file describing the graph.
        start: ID of the start node.
        goal: ID of the goal node.
        mandatory_nodes: List of mandatory nodes that must be included in the path.
        heuristic: Function to estimate cost from a node to the goal.

    Returns:
        A tuple containing the shortest path and its total cost.
    """
    points_to_visit = [start] + mandatory_nodes + [goal]
    current_node = start
    visited = set([current_node])
    path = []
    total_cost = 0

    # Greedily choose the nearest unvisited mandatory node
    while len(visited) < len(points_to_visit):
        nearest_node = None
        min_cost = float('inf')

        # Find the nearest unvisited node from the current node
        for next_node in points_to_visit:
            if next_node not in visited:
                path_segment, cost = astar(graph_file, current_node, next_node, heuristic)
                if cost < min_cost:
                    min_cost = cost
                    nearest_node = next_node
                    segment = path_segment

        # Add the nearest node to the path and update cost
        if nearest_node:
            path.extend(segment[1:])  # Avoid duplicating nodes
            visited.add(nearest_node)
            total_cost += min_cost
            current_node = nearest_node

    return path, total_cost


def reconstruct_path(came_from, current):
    """Reconstructs the path from start to goal."""
    path = []
    while current in came_from:
        path.append(current)
        current = came_from[current]
    path.append(current)
    return path[::-1]


def distance_between(nodes, node_a, node_b):
    """Calculate the Euclidean distance between two nodes."""
    coords_a = nodes[node_a]
    coords_b = nodes[node_b]
    return ((coords_a['d0'] - coords_b['d0'])**2 + (coords_a['d1'] - coords_b['d1'])**2)**0.5


def heuristic(node, goal, nodes):
    """Heuristic function: Euclidean distance to the goal."""
    return distance_between(nodes, node, goal)

# Example usage
graph_file = 'Competition_track_graph.graphml'
mandatory_nodes = config.mandatory_nodes

try:
    path, cost = astar_with_mandatory_nodes(graph_file, config.start_node, config.goal_node, mandatory_nodes, heuristic)
    print(f"Shortest path: {path}")
    print(f"Total cost: {cost}")
except ValueError as e:
    print(e)