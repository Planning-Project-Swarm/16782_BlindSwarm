import os
import argparse
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

    def detect_conflicts(paths):
        for r1_id, path1 in paths.items():
            for r2_id, path2 in paths.items():
                # avoid recompare
                if r1_id >= r2_id:
                    continue
                for t in range(min(len(path1), len(path2))):
                    x1, y1, _, _ = path1[t]
                    x2, y2, _, _ = path2[t]

                    # Check if the robots are within collision distance
                    if math.hypot(x1 - x2, y1 - y2) <= 2 * 0.8:
                        return (r1_id, r2_id, t)  # Conflict detected
        return None

    def resolve_conflict(conflict, paths, constraints):
        r1_id, r2_id, time = conflict
        constraints.append((r1_id, time, paths[r1_id][time]))
        constraints.append((r2_id, time, paths[r2_id][time]))
        return constraints
########################################################################################
    # def RRT(robot, goal, constraints):#this is an example of planner, write RRT here
    #     path = []
    #     for t in [i / 10.0 for i in range(11)]:
    #         x = robot['x'] + t * (goal['x'] - robot['x'])
    #         y = robot['y'] + t * (goal['y'] - robot['y'])
    #         path.append((x, y, t))
    #     return path
########################################################################################

    # paths is a dictionary of robot_id to list of (x, y, th, t)
    paths = {robot['id']: [] for robot in robots}
    constraints = []

    for robot in robots:
        goal = next(g for g in goals if g['id'] == robot['id'])
        # this rrt is a single robot planner
        rrt = RRT(
            start=[robot['x'], robot['y'], robot['orientation']],
            goal=[goal['x'], goal['y'], robot['orientation']],
            rand_area=[0, 20],
            obstacle_list=obstacles,
            play_area=[0, 20, 0, 20],
            robot_radius=0.8
        )
        # x y th t
        rrt.planning(animation=False)

        if rrt.path is None:
            print("Cannot find path for robot ", robot['id'])
        else:
            print("Found path for robot ", robot['id'])
            rrt.draw_graph()
            paths[robot['id']] = rrt.path

    # detect and resolve conflicts
    while True:
        conflict = detect_conflicts(paths)
        if not conflict:
            print("No conflicts found")
            break
        print("Conflict detected: robot {} and robot {} at time {}".format(*conflict))
        constraints = resolve_conflict(conflict, paths, constraints)

        for robot_id, _, _ in constraints:
            print("Replanning robot ", robot_id)
            robot = next(r for r in robots if r['id'] == robot_id)
            goal = next(g for g in goals if g['id'] == robot_id)
            path = rrt.planning(animation=False, constraints=constraints)
            paths[robot_id] = path
            
    return paths


def write_output_file(output_path, paths):
    with open(output_path, 'w') as file:
        for robot_id, path in paths.items():
            path_str = '; '.join([f'{x:.2f},{y:.2f},{th:.2f},{t:.2f}' for x, y, th, t in path])
            # print(path_str)
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
    # print(robots)
    # print(goals)
    # print(obstacles)
    
    paths = cbs(robots, goals, obstacles)
    
    write_output_file(output_file, paths)
    print(f"Processed file '{args.input_file}' success。 Output saved to '{output_file}'.")
