import numpy as np
import math
import time
import matplotlib.pyplot as plt
import config
from matplotlib import animation
from carNavigation import calculate_steering_angle, update_car_position, recalculate_heading
from aStar import astar, heuristic, astar_with_mandatory_nodes
from parser import parse_graphml


# Functions: calculate_steering_angle, update_car_position, recalculate_heading (as provided)

def dynamic_visualization(graph_file, path, nodes, wheelbase, time_step, speed):
    # Initialize car's starting position
    car_x, car_y = nodes[start_node]['d0'], nodes[start_node]['d1']
    theta = 0.0  # Initial heading angle in radians
    heading_update_interval = 2
    update_counter = 0

    # Set up the figure
    fig, ax = plt.subplots()
    ax.set_aspect('equal')
    ax.set_xlim(-1, 22)  # Adjust based on map dimensions
    ax.set_ylim(-1, 16)  # Adjust based on map dimensions
    ax.set_title("Dynamic Car Navigation")
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")

    # Plot the nodes and the path
    for node_id, coords in nodes.items():
        ax.plot(coords['d0'], coords['d1'], 'bo', markersize=3)  # Plot nodes
    path_coords = [(nodes[node]['d0'], nodes[node]['d1']) for node in path]
    path_x, path_y = zip(*path_coords)
    ax.plot(path_x, path_y, 'g--', label="Planned Path")  # Path

    # Car representation
    car_plot, = ax.plot([], [], 'ro', markersize=8, label="Car Position")  # Car position
    trajectory, = ax.plot([], [], 'r-', linewidth=1, label="Trajectory")  # Car trajectory
    car_trajectory_x, car_trajectory_y = [], []

    target_idx = [1]  # Mutable list to track target index

    def update(frame):
        nonlocal car_x, car_y, theta, target_idx, update_counter

        if update_counter % heading_update_interval == 0:
            target_x, target_y = nodes[path[target_idx[0]]]['d0'], nodes[path[target_idx[0]]]['d1']
            theta = recalculate_heading(car_x, car_y, target_x, target_y)

        if target_idx[0] >= len(path):  # Stop if all targets are reached
            return car_plot, trajectory

        target_node = path[target_idx[0]]
        target_x, target_y = nodes[target_node]['d0'], nodes[target_node]['d1']

        # Calculate steering angle
        steering_angle = calculate_steering_angle(car_x, car_y, theta, target_x, target_y, wheelbase)

        # Update car position and heading
        car_x, car_y, theta = update_car_position(car_x, car_y, theta, steering_angle, speed, wheelbase, time_step)

        # Calculate distance to target
        distance = math.sqrt((target_x - car_x)**2 + (target_y - car_y)**2)

        # Check if target reached
        if distance < 0.03:  # Threshold
            target_idx[0] += 1

        # Update car position and trajectory
        car_plot.set_data([car_x], [car_y])
        car_trajectory_x.append(car_x)
        car_trajectory_y.append(car_y)
        trajectory.set_data(car_trajectory_x, car_trajectory_y)

        update_counter += 1

        return car_plot, trajectory

    ani = animation.FuncAnimation(fig, update, frames=5000000000, interval=5, blit=True, repeat=False)
    plt.legend()
    plt.show()

if __name__ == "__main__":
    graph_file = "Competition_track_graph.graphml"

    start_node = config.start_node
    goal_node = config.goal_node
    mandatory_nodes = config.mandatory_nodes

    # Car parameters
    wheelbase = config.wheelbase  # meters
    speed = config.speed  # meters per second
    time_step = config.time_step  # seconds

    # Get the shortest path using A* algorithm
    try:
        path, _ = astar_with_mandatory_nodes(graph_file, start_node, goal_node, mandatory_nodes, heuristic)
    except ValueError as e:
        print(f"Error: {e}")
        exit()

    # Parse the graph to retrieve node coordinates
    graph_data = parse_graphml(graph_file)
    if not graph_data:
        print("Failed to parse the graph file.")
        exit()

    nodes = graph_data['nodes']

    # Run the dynamic visualization
    dynamic_visualization(graph_file, path, nodes, wheelbase, time_step, speed)
