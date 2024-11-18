import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np
from itertools import cycle
import argparse

#TODO:
# Read in the plan file. This contains the plan for each robot, which is a sequence of x-y positions and orientations at each timestep t = dt.
# Plot the initial poses of all robots and obstacles.   
# Plot the plan for each robot. This is a sequence of x-y positions and orientations at each timestep t = dt.

def readMapFile(filename):
    with open(filename, 'r') as file:
        lines = file.readlines()
    
    robots = []
    obstacles = []
    goals = []
    
    for line in lines:
        parts = line.strip().split(',')
        if parts[0] == 'robot':
            robots.append({
                'id': int(parts[1]),
                'x': float(parts[2]),
                'y': float(parts[3]),
                'orientation': float(parts[4])
            })
        elif parts[0] == 'obstacle':
            obstacles.append({
                'x1': float(parts[1]),
                'y1': float(parts[2]),
                'x2': float(parts[3]),
                'y2': float(parts[4])
            })
        elif parts[0] == 'goal':
            goals.append({
                'id': int(parts[1]),
                'x': float(parts[2]),
                'y': float(parts[3]),
                'orientation': float(parts[4])
            })
    
    return robots, obstacles, goals

def readPlanFile(filename):
    with open(filename, 'r') as file:
        lines = file.readlines()
    
    plans = {}
    
    for line in lines:
        parts = line.strip().split(',')
        robot_id = int(parts[0])
        if robot_id not in plans:
            plans[robot_id] = []
        plans[robot_id].append({
            't': float(parts[1]),
            'x': float(parts[2]),
            'y': float(parts[3]),
            'orientation': float(parts[4])
        })
    
    return plans

def livePlot(robots, obstacles, goals, plans):
    fig, ax = plt.subplots()
    ax.set_aspect('equal', adjustable='box')
    ax.set_xlim(0, 20)
    ax.set_ylim(0, 20)
    ax.set_title("Robot Trajectories with Obstacles")
    ax.set_xlabel("X Position")
    ax.set_ylabel("Y Position")
    
    # Plot obstacles as pure black polygons
    for obstacle in obstacles:
        ax.fill([obstacle['x1'], obstacle['x2'], obstacle['x2'], obstacle['x1']], [obstacle['y1'], obstacle['y1'], obstacle['y2'], obstacle['y2'], obstacle['y1']], color='black', edgecolor='black', linewidth=1)
    
    # Plot goals
    for goal in goals:
        ax.plot(goal['x'], goal['y'], 'ro')
    
    # Plot initial robot positions as triangles
    for robot in robots:
        pentagon = plt.Polygon(get_pentagon(robot['x'], robot['y'], robot['orientation']), color='blue')
        ax.add_patch(triangle)
    
    colormap = plt.cm.get_cmap('hsv', len(plans))
    robot_colors = cycle([colormap(i) for i in range(len(plans))])
    robot_lines = {}
    for robot_id in plans.keys():
        color = next(robot_colors)
        line, = ax.plot([], [], color=color, label=f'Robot {robot_id}')
        robot_lines[robot_id] = line

    def init():
        for line in robot_lines.values():
            line.set_data([], [])
        return robot_lines.values()
    def update(frame):
        for robot_id, plan in plans.items():
            if frame < len(plan):
                robot = plan[frame]
                pentagon = plt.Polygon(get_house_shape(robot['x'], robot['y'], robot['orientation']), color=robot_lines[robot_id].get_color())
                robot_lines[robot_id].set_data([], [])
                ax.add_patch(triangle)
        return robot_lines.values()
        return robot_lines.values()

    ani = animation.FuncAnimation(fig, update, frames=range(max(len(plan) for plan in plans.values())), init_func=init, blit=True, repeat=False)
    plt.legend()

def get_house_shape(x, y, orientation, size=0.5):
    arrow = plt.Arrow(x, y, size * np.cos(orientation), size * np.sin(orientation), width=size, color='blue')
    return arrow

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Visualize robot plans.')
    parser.add_argument('map_file', type=str, help='Path to the map file')
    parser.add_argument('plan_file', type=str, nargs='?', default=None, help='Path to the plan file (optional)')
    args = parser.parse_args()

    robots, obstacles, goals = readMapFile(args.map_file)
    
    fig, ax = plt.subplots()
    ax.set_aspect('equal', adjustable='box')
    ax.set_xlim(0, 20)
    ax.set_ylim(0, 20)
    ax.set_xlabel("X Position")
    ax.set_ylabel("Y Position")
    
    for obstacle in obstacles:
        ax.fill([obstacle['x1'], obstacle['x2'], obstacle['x2'], obstacle['x1']], [obstacle['y1'], obstacle['y1'], obstacle['y2'], obstacle['y2'], obstacle['y1']], color='black', edgecolor='black', linewidth=1)
    
    for goal in goals:
        ax.plot(goal['x'], goal['y'], 'ro')
    
    for robot in robots:
        triangle = get_house_shape(robot['x'], robot['y'], robot['orientation'])
        ax.add_patch(triangle)
    
    if args.plan_file:
        plans = readPlanFile(args.plan_file)
        livePlot(robots, obstacles, goals, plans)
    else:
        ax.set_title("Robot Initial Positions and Obstacles")
        plt.show()

