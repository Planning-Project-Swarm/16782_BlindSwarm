import os
import argparse
from collections import defaultdict
from visualizer_cbs import Visualizer
from rrt_mp import RRT
import math

class CBS:

    def __init__(self,
                 starts,
                 goals,
                 obstacle_list,
                 rand_area,
                 robot_radius=0.5,
                 max_attempts_to_resolve_conflicts = 10
                 ):
        self.robots = starts
        self.goals = goals
        self.obstacles = obstacle_list
        self.rand_area = rand_area
        self.robot_radius = robot_radius
        self.max_attempts_to_resolve_conflicts = max_attempts_to_resolve_conflicts
        self.paths = {}

    def planning(self):

        def detect_conflicts(paths, robot_radius):
            conflicts = defaultdict(list)
            robot_ids = list(paths.keys())

            for i, r1_id in enumerate(robot_ids):
                for r2_id in robot_ids[i + 1:]:
                    # print("Checking conflicts between robot {} and robot {}".format(r1_id, r2_id))

                    path1, path2 = paths[r1_id], paths[r2_id]
                    len1, len2 = len(path1), len(path2)
                    max_len = max(len1, len2)

                    for t in range(max_len):
                        # Get positions for the current time or the last available position
                        x1, y1, _, _ = path1[t] if t < len1 else path1[-1]
                        x2, y2, _, _ = path2[t] if t < len2 else path2[-1]
                        # print("Robot {} at ({}, {}) and robot {} at ({}, {})".format(r1_id, x1, y1, r2_id, x2, y2))

                        # Check if the robots are within collision distance
                        # using 1.8 instead of 2 to allow for some buffer
                        if math.hypot(x1 - x2, y1 - y2) <= 1.8 * robot_radius:
                            print("\033[93m[WARN]\033[0m Conflict detected: robot {} and robot {} at time {}".format(r1_id, r2_id, t))
                            conflicts[r1_id].append((r2_id, t))
                            conflicts[r2_id].append((r1_id, t))

            # Sort conflicts by robot ID and timestamp
            return {key: sorted(value, key=lambda x: (x[0], x[1])) for key, value in conflicts.items()}

        def generate_constraints(conflicts, paths):
            constraints = defaultdict(list)
            for r1_id, r1_conflicts in conflicts.items():
                for conflict in r1_conflicts:
                    r2_id, time = conflict
                    path1 = paths[r1_id]
                    len1 = len(path1)

                    # If the conflict time exceeds the path length, use the final position
                    x, y, _, _ = path1[time] if time < len1 else path1[-1]

                    # Add the constraint with the position and time
                    constraints[r1_id].append([x, y, time])
            return constraints


        # paths is a dictionary of robot_id to list of (x, y, th, t)
        self.paths = {robot['id']: [] for robot in self.robots}
        constraints = []

        viz = Visualizer()

        for robot in self.robots:
            goal = next(g for g in self.goals if g['id'] == robot['id'])
            # this rrt is a single robot planner
            rrt = RRT(
                start=[robot['x'], robot['y'], robot['orientation']],
                goal=[goal['x'], goal['y'], goal['orientation']],
                rand_area=self.rand_area,
                obstacle_list=self.obstacles,
                play_area=self.rand_area * 2,
                robot_radius=self.robot_radius
            )
            # x y th t
            rrt.planning(animation=False)

            if rrt.path is None:
                print("\033[93m[WARN]\033[0m Cannot find path for robot", robot['id'])
            else:
                print("Found path for robot ", robot['id'])
                self.paths[robot['id']] = rrt.path


        # detect and resolve conflicts
        for i in range(self.max_attempts_to_resolve_conflicts):
            conflicts = detect_conflicts(self.paths, self.robot_radius)
            if not conflicts:
                print("\033[92mNo conflicts found\033[0m")
                break

            constraints = generate_constraints(conflicts, self.paths)

            # print(constraints)
            # constraint_path is a list of x, y
            for robot_id, constraint_path in constraints.items():
                print("Replanning robot ", robot_id)
                robot = next(r for r in self.robots if r['id'] == robot_id)
                goal = next(g for g in self.goals if g['id'] == robot_id)

                rrt = RRT(
                    start=[robot['x'], robot['y'], robot['orientation']],
                    goal=[goal['x'], goal['y'], goal['orientation']],
                    rand_area=self.rand_area,
                    obstacle_list=self.obstacles,
                    play_area=self.rand_area * 2,
                    robot_radius=self.robot_radius
                )
                rrt.planning(animation=False, constraints=constraint_path)
                if rrt.path is None:
                    print("\033[91m[ERROR]\033[0m Cannot find path for robot", robot_id)
                else:
                    print("Found path for robot ", robot_id)
                    self.paths[robot_id] = rrt.path

            # viz.viz_paths(self.paths, self.rand_area * 2, self.obstacles, self.robot_radius, True, "cbs_conflict_viz_{}.mp4".format(i))
            viz.viz_paths(self.paths, self.rand_area * 2, self.obstacles, self.robot_radius, False)


        # visualize the final result
        viz.viz_paths(self.paths, self.rand_area * 2, self.obstacles, self.robot_radius, False)
        # save final result as mp4
        viz.viz_paths(self.paths, self.rand_area * 2, self.obstacles, self.robot_radius, True, "cbs_final_viz.mp4")

        
    def write_output_file(self, output_path):
        with open(output_path, 'w') as file:
            for robot_id, path in self.paths.items():
                path_str = '; '.join([f'{x:.2f},{y:.2f},{th:.2f},{t:.2f}' for x, y, th, t in path])
                file.write(f'Robot {robot_id}: {path_str}\n')

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

    cbs = CBS(robots, goals, obstacles, [0, 20])
    cbs.planning()
    cbs.write_output_file(output_file)
    print(f"Processed file '{args.input_file}' success。 Output saved to '{output_file}'.")
