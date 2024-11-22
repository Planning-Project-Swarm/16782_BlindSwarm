import os
import argparse
import matplotlib.pyplot as plt
from collections import defaultdict
from visualizer_cbs import Visualizer
from rrt_mp import RRT

import math
def read_map_file(file_path):
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

def cbs(robots, goals, obstacles):

    def detect_conflicts(paths, robot_radius):
        sorted_conflicts = defaultdict(list)
        conflicts = defaultdict(list)
        robot_ids = list(paths.keys())
        for i in range(len(robot_ids)):
            r1_id = robot_ids[i]
            for j in range(i + 1, len(robot_ids)):
                r2_id = robot_ids[j]
                path1 = paths[r1_id]
                path2 = paths[r2_id]
                
                # Iterate through the timestamps and detect conflicts
                for t in range(min(len(path1), len(path2))):
                    x1, y1, _, _ = path1[t]
                    x2, y2, _, _ = path2[t]
                    
                    # Check if the robots are within collision distance
                    if math.hypot(x1 - x2, y1 - y2) <= 2 * robot_radius:
                        print("Conflict detected: robot {} and robot {} at time {}".format(r1_id, r2_id, t))
                        # Add the conflict to both robots in the defaultdict
                        conflicts[r1_id].append((r2_id, t))
                        conflicts[r2_id].append((r1_id, t))

        # Sort the conflicts acrroiding to robot_id
        for key in sorted(conflicts.keys()):
            sorted_conflicts[key] = sorted(conflicts[key], key=lambda x: (x[0], x[1]))
                
        return sorted_conflicts

    def generate_constraints(conflicts, paths):
        constraints = defaultdict(list)
        for r1_id, conflicts in conflicts.items():
            for conflict in conflicts:
                r2_id, time = conflict
                # x, y, t
                constraints[r1_id].append([paths[r1_id][time][0], paths[r1_id][time][1], time])
        return constraints

    # paths is a dictionary of robot_id to list of (x, y, th, t)
    cbs_paths = {robot['id']: [] for robot in robots}
    constraints = []
    area_side_len = [0, 20]
    fly_area = area_side_len * 2
    # robot_radius = 0.8
    robot_radius = 0.5
    max_attempts_to_resolve_conflicts = 5
    viz = Visualizer()

    for robot in robots:
        goal = next(g for g in goals if g['id'] == robot['id'])
        # this rrt is a single robot planner
        rrt = RRT(
            start=[robot['x'], robot['y'], robot['orientation']],
            goal=[goal['x'], goal['y'], goal['orientation']],
            rand_area=area_side_len,
            obstacle_list=obstacles,
            play_area=fly_area,
            robot_radius=robot_radius
        )
        # x y th t
        rrt.planning(animation=False)

        if rrt.path is None:
            print("Cannot find path for robot ", robot['id'])
        else:
            print("Found path for robot ", robot['id'])
            # rrt.draw_graph()
            cbs_paths[robot['id']] = rrt.path


    # detect and resolve conflicts
    for i in range(max_attempts_to_resolve_conflicts):
        conflicts = detect_conflicts(cbs_paths, robot_radius)
        if not conflicts:
            print("No conflicts found")
            break

        constraints = generate_constraints(conflicts, cbs_paths)

        # print(constraints)
        # constraint_path is a list of x, y
        for robot_id, constraint_path in constraints.items():
            print("Replanning robot ", robot_id)
            robot = next(r for r in robots if r['id'] == robot_id)
            goal = next(g for g in goals if g['id'] == robot_id)

            rrt = RRT(
                start=[robot['x'], robot['y'], robot['orientation']],
                goal=[goal['x'], goal['y'], goal['orientation']],
                rand_area=area_side_len,
                obstacle_list=obstacles,
                play_area=fly_area,
                robot_radius=robot_radius
            )
            rrt.planning(animation=False, constraints=constraint_path)
            cbs_paths[robot_id] = rrt.path

        viz.viz_paths(cbs_paths, fly_area, obstacles, False)

    # visualize the final result
    viz.viz_paths(cbs_paths, fly_area, obstacles, False)
    # save final result as mp4
    viz.viz_paths(cbs_paths, fly_area, obstacles, True, "cbs_visualization.mp4")

    return cbs_paths


def write_output_file(output_path, paths):
    with open(output_path, 'w') as file:
        for robot_id, path in paths.items():
            path_str = '; '.join([f'{x:.2f},{y:.2f},{th:.2f},{t:.2f}' for x, y, th, t in path])
            file.write(f'Robot {robot_id}: {path_str}\n')


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Run CBS for a single map file.")
    parser.add_argument('input_file', type=str, help="Name of the input map file (located in 'maps' folder).")
    args = parser.parse_args()

    maps_folder = "maps"
    input_file_path = os.path.join(maps_folder, args.input_file)

    if not os.path.exists(input_file_path):
        raise FileNotFoundError(f"Input file '{input_file_path}' not found.")
    
    output_folder = "output"
    if not os.path.exists(output_folder):
        os.makedirs(output_folder)
    base_name = os.path.splitext(os.path.basename(args.input_file))[0]
    output_file = os.path.join(output_folder, f"{base_name}_output.txt")

    robots, goals, obstacles = read_map_file(input_file_path)
    
    paths = cbs(robots, goals, obstacles)
    
    write_output_file(output_file, paths)
    print(f"Processed file '{args.input_file}' success。 Output saved to '{output_file}'.")
