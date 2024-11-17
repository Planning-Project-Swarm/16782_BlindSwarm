import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np
from itertools import cycle
import re

# Define colors for each robot, and set obstacles to pure black
robot_colors = cycle(plt.cm.tab20(np.linspace(0, 1, 20)))
obstacle_color = "black"

file_path = 'plan.txt'
robot_data = {}  # Dictionary to store data for each robot
obstacle_data = []  # List to store obstacle points

# Parse the file for each obstacle and robot's data
with open(file_path, 'r') as file:
    current_robot = None
    current_obstacle = None
    parsing_obstacles = True  # Flag to know when we are reading obstacle data

    for line in file:
        line = line.strip()  # Remove any leading/trailing whitespace

        if line.startswith("Obstacle"):
            # Start a new obstacle
            current_obstacle = []
            obstacle_data.append(current_obstacle)
        elif line.startswith("Robot"):
            # Switch to robot data
            current_robot = line.strip()
            robot_data[current_robot] = []
            parsing_obstacles = False
        else:
            # Process coordinate lines
            components = line.split()
            if len(components) >= 2:
                try:
                    x, y = map(float, components[:2])  # Take only the first two values as x, y
                    if parsing_obstacles and current_obstacle is not None:
                        current_obstacle.append((x, y))
                    elif not parsing_obstacles and current_robot is not None:
                        robot_data[current_robot].append((x, y))
                except ValueError:
                    print(f"Skipping invalid line: {line}")

# Set up the figure and axis
fig, ax = plt.subplots()
all_x = [point[0] for points in robot_data.values() for point in points] + \
        [pt[0] for obstacle in obstacle_data for pt in obstacle]
all_y = [point[1] for points in robot_data.values() for point in points] + \
        [pt[1] for obstacle in obstacle_data for pt in obstacle]

# Set equal aspect ratio to maintain x and y scales
ax.set_aspect('equal', adjustable='box')
ax.set_xlim(min(all_x) - 1, max(all_x) + 1)
ax.set_ylim(min(all_y) - 1, max(all_y) + 1)
ax.set_title("Robot Trajectories with Obstacles")
ax.set_xlabel("X Position")
ax.set_ylabel("Y Position")

# Plot obstacles as pure black polygons
for obstacle_points in obstacle_data:
    obstacle_x, obstacle_y = zip(*obstacle_points)
    ax.fill(obstacle_x, obstacle_y, color=obstacle_color, edgecolor='black', linewidth=1)

# Add a single legend entry for obstacles
ax.plot([], [], color=obstacle_color, label='Obstacle')

# Initialize lines and markers for each robot
lines = {}
start_markers = {}
goal_markers = {}
for robot, color in zip(robot_data.keys(), robot_colors):
    line, = ax.plot([], [], lw=2, label=robot, color=color)
    start_marker, = ax.plot([], [], marker='o', color=color, markersize=8, label=f"{robot} Start")
    goal_marker, = ax.plot([], [], marker='X', color=color, markersize=8, label=f"{robot} Goal")
    lines[robot] = line
    start_markers[robot] = start_marker
    goal_markers[robot] = goal_marker

# Define the update function for animation
def update(num):
    for robot, line in lines.items():
        points = robot_data[robot]
        if num < len(points):
            x_values, y_values = zip(*points[:num+1])
            line.set_data(x_values, y_values)
            start_markers[robot].set_data(points[0])  # Set start point
            goal_markers[robot].set_data(points[-1])  # Set goal point
    return [*lines.values(), *start_markers.values(), *goal_markers.values()]

# Create the animation without displaying it
ani = animation.FuncAnimation(
    fig, update, frames=max(len(points) for points in robot_data.values()), 
    blit=True, repeat=False
)

# Add the legend outside the plot area on the right
ax.legend(loc="center left", bbox_to_anchor=(1, 0.5), fontsize="small", frameon=True)

# Adjust figure size and layout to make room for the legend on the right
fig.tight_layout(pad=3)

# Save the animation as robot_trajectories.mp4
ani.save("robot_trajectories.mp4", writer="ffmpeg", fps=30)

# Close the plot to avoid displaying it
plt.close(fig)

