import numpy as np
import math
import time
import config
from aStar import astar, heuristic,astar_with_mandatory_nodes
from parser import parse_graphml

def calculate_steering_angle(car_x, car_y, car_heading, target_x, target_y, wheelbase):
    """
    Calculate the steering angle for the car to reach a target point.
    """
    # Calculate the relative position of the target point
    dx = target_x - car_x
    dy = target_y - car_y

    # Transform the target point to the car's coordinate frame
    local_x = dx * np.cos(-car_heading) - dy * np.sin(-car_heading)
    local_y = dx * np.sin(-car_heading) + dy * np.cos(-car_heading)

    # Handle special cases:
    if abs(local_y) < 1e-3:  # Target directly in front or behind
        if local_x > 0:
            return np.pi / 2  # Turn right
        elif local_x < 0:
            return -np.pi / 2  # Turn left
        else:
            return 0  # No steering
    elif abs(local_x) < 1e-3:  # Target directly above or below
        if local_y > 0:
            return 0  # Drive forward
        else:
            return np.pi  # Drive backward

    # Calculate the turning radius R
    R = (local_x**2 + local_y**2) / (2 * local_y) if local_y != 0 else np.inf

    # Steering angle (Ackermann)
    steering_angle = np.arctan(wheelbase / R) if R != np.inf else 0
    return steering_angle

def update_car_position(car_x, car_y, theta, steering_angle, speed, wheelbase, time_step):
    """
    Update the car's position and heading angle based on its current state and control inputs.
    """
    # Update the car's position
    car_x += speed * math.cos(theta) * time_step
    car_y += speed * math.sin(theta) * time_step

    # Update the car's heading angle
    theta += (speed / wheelbase) * math.tan(steering_angle) * time_step

    # Normalize theta to be within -pi to pi for consistency
    theta = (theta + math.pi) % (2 * math.pi) - math.pi

    return car_x, car_y, theta

def recalculate_heading(car_x, car_y, target_x, target_y):
    """Recalculate the car's heading based on its position and the target."""
    dx = target_x - car_x
    dy = target_y - car_y
    return math.atan2(dy, dx)

if __name__ == "__main__":
    graph_file = "Competition_track_graph.graphml"

    start_node = config.start_node
    goal_node = config.goal_node
    mandatory_nodes = config.mandatory_nodes

    # Car parameters
    theta = 0.0  # Initial heading angle in radians
    wheelbase = config.wheelbase # meters
    speed = config.speed # meters per second
    time_step = config.time_step # seconds

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

    # Initialize car's starting position
    car_x, car_y = nodes[start_node]['d0'], nodes[start_node]['d1']
    print(f"Starting navigation from Node {start_node} at position ({car_x:.2f}, {car_y:.2f})\n")

    # Navigate along the shortest path
    for idx, target_node in enumerate(path[1:]):  # Skip the starting node
        target_x, target_y = nodes[target_node]['d0'], nodes[target_node]['d1']
        print(f"Target {idx + 1}: Node {target_node}, coordinates ({target_x}, {target_y})")

        while True:
            # Calculate the steering angle to the next target
            steering_angle = calculate_steering_angle(car_x, car_y, theta, target_x, target_y, wheelbase)

            # Calculate the distance to the target
            distance = math.sqrt((target_x - car_x)**2 + (target_y - car_y)**2)

            # If the car is close enough to the target, move to the next node
            if distance < 0.03:
                print(f"Reached Target {idx + 1}: Node {target_node} at ({target_x:.2f}, {target_y:.2f})")
                break

            # Update car's position and heading angle
            car_x, car_y, theta = update_car_position(car_x, car_y, theta, steering_angle, speed, wheelbase, time_step)

            # Recalculate heading periodically
            if idx % 2 == 0:
                theta = recalculate_heading(car_x, car_y, target_x, target_y)


            # Print the current state
            print(f"Current position: x={car_x:.2f}, y={car_y:.2f}, heading={math.degrees(theta):.2f}°, "
                  f"steering angle={math.degrees(steering_angle):.2f}°, distance to target={distance:.2f} m")

            # Optional: Delay for simulation
            time.sleep(time_step)

    print("Navigation complete!")
