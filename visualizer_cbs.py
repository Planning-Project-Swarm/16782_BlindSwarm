import matplotlib.pyplot as plt
import matplotlib.animation as animation
import argparse
import os
import math
from swarm_io import SwarmIO

class Visualizer:
    def __init__(self):
        """Initialize the Visualizer without storing area and obstacles."""
        pass

    def viz_paths(self, paths, area, obstacles, robot_radius, save_animation=False, output_file="cbs_visualization.mp4"):
        """
        Visualize paths and optionally save as an animation with larger resolution and adjusted layout.

        :param paths: Dictionary of robot paths. Keys are robot IDs, values are lists of [x, y, th, t].
        :param area: [x_min, x_max, y_min, y_max]
        :param obstacles: List of obstacles, each represented as a dict with keys 'x1', 'y1', 'x2', 'y2'
        :param robot_radius: Radius of each robot for visualization.
        :param save_animation: Boolean, whether to save the animation as MP4.
        :param output_file: Name of the output MP4 file if save_animation is True.
        """
        # Set a larger figure size for better visibility
        fig, ax = plt.subplots(figsize=(10, 8))  # Adjust size for better detail

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
        orientation_arrows = {}
        robot_circles = {}

        for path_id, path in paths.items():
            # Assign color to path line
            path_lines[path_id], = ax.plot([], [], '-o', label=f"Robot {path_id}")
            start_points[path_id] = plt.Circle((0, 0), robot_radius, color=path_lines[path_id].get_color(), alpha=0.5)
            end_points[path_id] = plt.Circle((0, 0), robot_radius, color=path_lines[path_id].get_color(), alpha=0.5)
            robot_circles[path_id] = plt.Circle((0, 0), robot_radius, color=path_lines[path_id].get_color(), alpha=0.8)

            # Add circles to the plot
            ax.add_patch(start_points[path_id])
            ax.add_patch(end_points[path_id])
            ax.add_patch(robot_circles[path_id])

            orientation_arrows[path_id] = None  # Placeholder for arrows

        # Add text for displaying current time
        time_text = ax.text(0.05, 0.95, '', transform=ax.transAxes, fontsize=12, verticalalignment='top')

        def update(frame):
            for path_id, path in paths.items():
                if len(path) > frame:
                    x_coords = [point[0] for point in path[:frame + 1]]
                    y_coords = [point[1] for point in path[:frame + 1]]
                    theta = [point[2] for point in path[:frame + 1]]  # Get orientations up to the current frame

                    # Update path lines
                    path_lines[path_id].set_data(x_coords, y_coords)

                    # Update start point
                    start_points[path_id].center = (path[0][0], path[0][1])

                    # Update end point
                    end_points[path_id].center = (path[-1][0], path[-1][1])

                    # Update robot circle for current position
                    robot_circles[path_id].center = (path[frame][0], path[frame][1])

                    # Draw an arrow for the last/current position at this frame
                    current_x, current_y, current_theta = path[frame][0], path[frame][1], path[frame][2]
                    if orientation_arrows[path_id]:
                        orientation_arrows[path_id].remove()  # Remove the previous arrow
                    orientation_arrows[path_id] = ax.arrow(
                        current_x, current_y,  # Start position of the arrow
                        robot_radius * math.cos(current_theta),  # X-component of arrow length
                        robot_radius * math.sin(current_theta),  # Y-component of arrow length
                        head_length=robot_radius,  # Length of the arrowhead
                        head_width=robot_radius,  # Width of the arrowhead
                        color=path_lines[path_id].get_color(),  # Match arrow color to the path
                        alpha=0.8
                    )

            # Update time text
            current_time = frame
            time_text.set_text(f"Time: {current_time:.1f}")

            return list(path_lines.values()) + list(start_points.values()) + list(end_points.values()) + list(orientation_arrows.values()) + list(robot_circles.values()) + [time_text]

        # Determine the maximum length of all paths
        max_frames = max(len(path) for path in paths.values())

        # Create the animation
        ani = animation.FuncAnimation(fig, update, frames=max_frames, blit=True)

        # Adjust layout and save animation
        if save_animation:
            print(f"Saving video as {output_file}...")
            plt.legend(loc='center left', bbox_to_anchor=(1.05, 0.5), fontsize=10)  # Move legend closer
            plt.subplots_adjust(left=0.1, right=0.85)  # Adjust plot area for legend
            ani.save(output_file, writer='ffmpeg', fps=5, dpi=200)  # Higher resolution for details
            print(f"Video saved as {output_file}")
        else:
            plt.legend(loc='center left', bbox_to_anchor=(1.05, 0.5), fontsize=10)  # Move legend closer
            plt.subplots_adjust(left=0.1, right=0.85)  # Adjust plot area for legend
            plt.show()

def main():
    parser = argparse.ArgumentParser(description="Visualize paths of robots on a map.")
    parser.add_argument('map_file', type=str, help="Name of the input map file (located in 'maps' folder).")
    parser.add_argument('plan_file', type=str, help="Name of the input plan file (located in 'ouput' folder).")
    args = parser.parse_args()

    maps_folder = "maps"
    map_file_path = os.path.join(maps_folder, args.map_file)
    if not os.path.exists(map_file_path):
        raise FileNotFoundError(f"Map file '{map_file_path}' not found.")

    plan_folder = "output"
    plan_file_path = os.path.join(plan_folder, args.plan_file)
    if not os.path.exists(plan_file_path):
        raise FileNotFoundError(f"Plan file '{plan_file_path}' not found.")

    swarm_io = SwarmIO()

    robots, goals, obstacles = swarm_io.read_map_file(map_file_path)
    paths = swarm_io.read_cbs_plan_file(plan_file_path)

    # Define the area (can be determined from map or manually set)
    area = [0, 20, 0, 20]
    robot_radius = 0.5

    # Initialize the Visualizer and create the video
    visualizer = Visualizer()
    visualizer.viz_paths(paths, area, obstacles, robot_radius, save_animation=False, output_file="cbs_visualization.mp4")
    visualizer.viz_paths(paths, area, obstacles, robot_radius, save_animation=True, output_file="cbs_visualization.mp4")


if __name__ == "__main__":
    main()