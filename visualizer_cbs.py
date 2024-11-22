import matplotlib.pyplot as plt
import matplotlib.animation as animation
import argparse
import os

class Visualizer:
    def __init__(self):
        """Initialize the Visualizer without storing area and obstacles."""
        pass

    def viz_paths(self, paths, area, obstacles, save_animation=False, output_file="cbs_visualization.mp4"):
        """
        Visualize paths and optionally save as an animation.

        :param paths: Dictionary of robot paths. Keys are robot IDs, values are lists of [x, y, th, t].
        :param area: [x_min, x_max, y_min, y_max]
        :param obstacles: List of obstacles, each represented as a dict with keys 'x1', 'y1', 'x2', 'y2'
        :param save_animation: Boolean, whether to save the animation as MP4.
        :param output_file: Name of the output MP4 file if save_animation is True.
        """
        fig, ax = plt.subplots()

        # Plot the area boundary
        ax.plot([area[0], area[1], area[1], area[0], area[0]],
                [area[2], area[2], area[3], area[3], area[2]], "-k")
        ax.set_xlim(area[0] - 1, area[1] + 1)
        ax.set_ylim(area[2] - 1, area[3] + 1)
        ax.set_aspect('equal', adjustable='box')
        ax.grid(True)

        # Plot obstacles
        for obs in obstacles:
            rect_x = [obs['x1'], obs['x2'], obs['x2'], obs['x1'], obs['x1']]
            rect_y = [obs['y1'], obs['y1'], obs['y2'], obs['y2'], obs['y1']]
            ax.fill(rect_x, rect_y, "gray")

        # Initialize paths and handles
        path_lines = {}
        start_points = {}
        end_points = {}

        for path_id, path in paths.items():
            path_lines[path_id], = ax.plot([], [], '-o', label=f"Robot {path_id}")
            start_points[path_id] = ax.plot([], [], "or")[0]
            end_points[path_id] = ax.plot([], [], "xr")[0]

        def update(frame):
            for path_id, path in paths.items():
                if len(path) > frame:
                    x_coords = [point[0] for point in path[:frame+1]]
                    y_coords = [point[1] for point in path[:frame+1]]
                    path_lines[path_id].set_data(x_coords, y_coords)
                    start_points[path_id].set_data(path[0][0], path[0][1])
                    end_points[path_id].set_data(path[-1][0], path[-1][1])
            return list(path_lines.values()) + list(start_points.values()) + list(end_points.values())

        # Determine the maximum length of all paths
        max_frames = max(len(path) for path in paths.values())

        # Create the animation
        ani = animation.FuncAnimation(fig, update, frames=max_frames, blit=True)

        # Adjust legend position
        if save_animation:
            ani.save(output_file, writer='ffmpeg', fps=5)
            print(f"Video saved as {output_file}")
        else:
            # Position the legend outside of the plot
            plt.legend(loc='center left', bbox_to_anchor=(1.05, 0.5), fontsize=10)
            plt.tight_layout(rect=[0, 0, 0.85, 1])  # Adjust layout to fit legend
            plt.show()


def read_map_file(file_path):
    """
    Read map information from a file.

    :param file_path: Path to the map file.
    :return: robots, goals, obstacles
    """
    robots = []
    goals = []
    obstacles = []

    with open(file_path, 'r') as file:
        for line in file:
            parts = line.strip().split(',')
            if parts[0] == 'robot':
                robots.append({
                    'id': int(parts[1]),
                    'x': float(parts[2]),
                    'y': float(parts[3]),
                    'orientation': float(parts[4])
                })
            elif parts[0] == 'goal':
                goals.append({
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
            else:
                raise ValueError(f"Invalid line in map file: '{line}'")
    return robots, goals, obstacles


def read_plan_file(file_path):
    """
    Read plan information from a file.

    :param file_path: Path to the plan file.
    :return: Dictionary with paths for each robot.
    """
    paths = {}

    with open(file_path, 'r') as file:
        for line in file:
            line = line.strip()
            if not line:
                continue  # Skip empty lines

            # Split the line into robot ID and path data
            if ':' not in line:
                raise ValueError(f"Invalid line in plan file: '{line}'")
            
            robot_id_str, path_data = line.split(':', 1)
            robot_id = int(robot_id_str.replace('Robot', '').strip())

            # Parse the path data
            path_points = path_data.split(';')
            paths[robot_id] = []
            for point in path_points:
                if point.strip():  # Skip empty segments
                    coords = point.split(',')
                    if len(coords) != 4:
                        raise ValueError(f"Invalid path point: '{point}' in Robot {robot_id}")
                    
                    x, y, th, t = map(float, coords)
                    paths[robot_id].append([x, y, th, t])

    return paths


def main():
    parser = argparse.ArgumentParser(description="Visualize paths of robots on a map.")
    parser.add_argument('input_map', type=str, help="Name of the input map file (located in 'maps' folder).")
    parser.add_argument('input_plan', type=str, help="Name of the input plan file (located in 'ouput' folder).")
    args = parser.parse_args()

    maps_folder = "maps"
    map_file_path = os.path.join(maps_folder, args.input_map)

    output_folder = "output"
    plan_file_path = os.path.join(output_folder, args.input_plan)

    # map_filename = 'map.txt'
    # Read the map and plan
    robots, goals, obstacles = read_map_file(map_file_path)
    paths = read_plan_file(plan_file_path)

    # Define the area (can be determined from map or manually set)
    area = [0, 20, 0, 20]

    # Initialize the Visualizer and create the video
    visualizer = Visualizer()
    visualizer.viz_paths(paths, area, obstacles, save_animation=True, output_file="cbs_visualization.mp4")


if __name__ == "__main__":
    main()