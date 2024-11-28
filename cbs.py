import os
import argparse

def read_map_file(file_path):
    robots = []
    goals = []

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
    return robots, goals


def cbs(robots, goals, rrt=None):
    paths = {robot['id']: [] for robot in robots}

    def detect_conflicts(paths):
        for r1_id, path1 in paths.items():
            for r2_id, path2 in paths.items():
                if r1_id >= r2_id:  # avoid recompare
                    continue
                for t in range(min(len(path1), len(path2))):
                    if path1[t][:3] == path2[t][:3]: 
                        return (r1_id, r2_id, t)  # return conflict
        return None

    def resolve_conflict(conflict, paths, constraints):
        r1_id, r2_id, time = conflict
        constraints.append((r1_id, time, paths[r1_id][time][:3])) 
        constraints.append((r2_id, time, paths[r2_id][time][:3]))  
        return constraints
########################################################################################
    def RRT(robot, goal, constraints):  # this is an example of planner, write RRT here
        path = []
        for t in [i / 10.0 for i in range(11)]:
            x = robot['x'] + t * (goal['x'] - robot['x'])
            y = robot['y'] + t * (goal['y'] - robot['y'])
            orientation = robot['orientation'] + t * (goal['orientation'] - robot['orientation'])
            if any(
                c_robot == robot['id'] and c_time == t and c_position[:3] == (x, y, orientation)
                for c_robot, c_time, c_position in constraints
            ):
                continue  
            path.append((x, y, orientation, t))
        return path
########################################################################################
    constraints = []

    for robot in robots:
        goal = next(g for g in goals if g['id'] == robot['id'])
        paths[robot['id']] = RRT(robot, goal, constraints)

    # detect and resolve conflicts
    while True:
        conflict = detect_conflicts(paths)
        if not conflict:
            break
        constraints = resolve_conflict(conflict, paths, constraints)
        # replan
        for robot_id, _, _ in constraints:
            robot = next(r for r in robots if r['id'] == robot_id)
            goal = next(g for g in goals if g['id'] == robot_id)
            paths[robot_id] = RRT(robot, goal, constraints)

    return paths


def write_output_file(output_path, paths):
    with open(output_path, 'w') as file:
        for robot_id, path in paths.items():
            path_str = ';'.join([f'{x:.2f},{y:.2f},{orientation:.2f}' for x, y, orientation, t in path]) + ';'
            file.write(f'{robot_id};{path_str}\n')



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

    robots, goals = read_map_file(input_file_path)
    
    paths = cbs(robots, goals)
    
    write_output_file(output_file, paths)
    print(f"Processed file '{args.input_file}' success. Output saved to '{output_file}'.")
