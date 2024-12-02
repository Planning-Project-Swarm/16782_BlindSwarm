import os

class SwarmIO:

    def __init__(self):
        pass

    def read_cbs_plan_file(self, file_path):
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
    
    def read_map_file(self, file_path):
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
                elif parts[0] == 'formation':
                    pass
                elif parts[0] == 'leader':
                    pass
                else:
                    raise ValueError(f"Invalid line in map file: '{line}'")
        return robots, goals, obstacles
    
    def write_cbs_output_file(self, paths, output_path):
        with open(output_path, 'w') as file:
            for robot_id, path in paths.items():
                path_str = '; '.join([f'{x:.2f},{y:.2f},{th:.2f},{t:.2f}' for x, y, th, t in path])
                file.write(f'Robot {robot_id}: {path_str}\n')

    def write_rrt_plan_file(self, filename, path):
        with open(filename, 'w') as f:
            for waypoint in path:
                # x, y, th, t
                # print(waypoint)
                f.write("{},{},{},{}\n".format(waypoint[0], waypoint[1], waypoint[2], waypoint[3]))

    def readFormation(self, file_path):
        formations = []
        leader = None
        with open(file_path, 'r') as file:
            for line in file:
                parts = line.strip().split(',')
                if parts[0] == 'formation':
                    formations.append({
                        'filename': parts[1],
                        'time': int(parts[2]),
                    })
                elif parts[0] == 'leader':
                    leader = {
                        'x': float(parts[1]),
                        'y': float(parts[2]),
                    }
        return formations, leader
    
    def read_formation_file(self, file_path):
        formation = []
        with open(file_path, 'r') as file:
            for line in file:
                parts = line.strip().split(',')
                formation.append({
                    'x': float(parts[2]),
                    'y': float(parts[3]),
                    'orientation': float(parts[4])
                })
        return formation