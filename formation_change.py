    """
    The drone formation transitions from one configuration to another, aiming to maintain a circular formation as much as possible during the process. 
    If obstacles are encountered, the circular configuration can temporarily dissolve to navigate around them.
    """

import os
import argparse
import math
from cbs_rrt import read_map_file, cbs, RRT, Visualizer

def generate_circle_positions(center, radius, num_drones):
    """
    Generate target positions for a circular formation.
    :param center: The center of the circle (x, y)
    :param radius: The radius of the circle
    :param num_drones: The number of drones
    :return: Target positions for the circular formation as a list of tuples [(x1, y1), (x2, y2), …]
    """
    positions = []
    angle_step = 2 * math.pi / num_drones
    for i in range(num_drones):
        theta = i * angle_step
        x = center[0] + radius * math.cos(theta)
        y = center[1] + radius * math.sin(theta)
        positions.append((x, y))
    return positions

def interpolate_positions(start_positions, end_positions, steps): #use this if interpolation is needed? or just delete it
    interpolated_positions = []
    for step in range(1, steps + 1):
        fraction = step / steps
        intermediate_positions = [
            (
                start[0] + fraction * (end[0] - start[0]),
                start[1] + fraction * (end[1] - start[1])
            )
            for start, end in zip(start_positions, end_positions)
        ]
        interpolated_positions.append(intermediate_positions)
    return interpolated_positions

def main():
    """
    1. Input maps
    2. Target formation generation: Generate circular formation target points and interpolate to generate intermediate states from the initial to the target.
    3. Multi-stage path planning: Call <<CBS-RRT>> at the intermediate target point of each step to generate a collision-free path.In case of obstacles, the CBS algorithm automatically adjusts the path to ensure safety.
    4. visualization
    """
if __name__ == "__main__":
    main()