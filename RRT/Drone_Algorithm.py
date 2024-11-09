import random
from copy import deepcopy

# Import the RRT-Connect code
from RRT_Connect import RRTConnect

class DroneCBS:
    def __init__(self, starts, goals, obstacle_list, rand_area, bounds, drone_radius=0.3):
        """
        Initialize CBS with multiple drones.

        :param starts: List of start positions for each drone, e.g., [(x1, y1), (x2, y2), ...]
        :param goals: List of goal positions for each drone, e.g., [(gx1, gy1), (gx2, gy2), ...]
        :param obstacle_list: Obstacles in the environment.
        :param rand_area: Random sampling area for RRT.
        :param bounds: Play area bounds.
        :param drone_radius: Radius of each drone.
        """
        self.starts = starts
        self.goals = goals
        self.obstacle_list = obstacle_list
        self.rand_area = rand_area
        self.bounds = bounds
        self.drone_radius = drone_radius
        self.paths = []  # Holds final paths for each drone

    def plan_paths(self):
        """
        Plan paths for all drones using CBS and RRT-Connect.
        """
        initial_constraints = []
        for i in range(len(self.starts)):
            path = self.plan_path_rrt(i, initial_constraints)
            if path is None:
                print(f"No path found for drone {i} with initial planning.")
                return None
            self.paths.append(path)

        # Run CBS on paths to resolve conflicts
        conflict_free_paths = self.conflict_based_search(initial_constraints)
        return conflict_free_paths

    def plan_path_rrt(self, drone_index, constraints):
        """
        Plan path for a single drone with constraints using RRT-Connect.

        :param drone_index: Index of the drone to plan path for.
        :param constraints: List of constraints (e.g., time and position constraints).
        :return: Planned path or None if no path is found.
        """
        start = self.starts[drone_index]
        goal = self.goals[drone_index]
        rrt = RRTConnect(start, goal, self.obstacle_list, self.rand_area, play_area=self.bounds, robot_radius=self.drone_radius)
        path = rrt.planning(animation=False)

        if path is not None:
            # Apply constraints, if any, to path
            for constraint in constraints:
                if self.check_constraint_violations(path, constraint, drone_index):
                    print(f"Constraint violation detected for drone {drone_index}")
                    return None  # Replan if constraints are violated
        return path

    def check_constraint_violations(self, path, constraint, drone_index):
        """
        Check if path violates any constraint.
        """
        for point in path:
            time_step, pos = constraint
            if point == pos and path.index(point) == time_step:
                return True  # Constraint violation
        return False

    def conflict_based_search(self, initial_constraints):
        """
        Run CBS algorithm on the planned paths to handle conflicts.
        """
        open_list = [(deepcopy(self.paths), initial_constraints)]  # Each entry is (paths, constraints)

        while open_list:
            paths, constraints = open_list.pop(0)
            conflict = self.detect_conflicts(paths)

            if not conflict:
                return paths  # No conflicts, paths are found

            # Resolve conflict by adding constraints and replanning
            for agent, time_step, pos in conflict:
                new_constraints = constraints + [(time_step, pos)]
                new_paths = deepcopy(paths)
                
                new_path = self.plan_path_rrt(agent, new_constraints)
                if new_path:
                    new_paths[agent] = new_path
                    open_list.append((new_paths, new_constraints))
        
        print("No conflict-free paths found.")
        return None

    def detect_conflicts(self, paths):
        """
        Detect conflicts among paths.
        """
        max_length = max(len(p) for p in paths)

        for t in range(max_length):
            positions = {}
            for agent, path in enumerate(paths):
                pos = path[t] if t < len(path) else path[-1]

                if pos in positions:
                    return [(agent, t, pos), (positions[pos], t, pos)]
                positions[pos] = agent
        return None  # No conflict

    def visualize_paths(self):
        """
        Visualize paths for all drones (optional).
        """
        import matplotlib.pyplot as plt
        for i, path in enumerate(self.paths):
            x_coords = [x for x, y in path]
            y_coords = [y for x, y in path]
            plt.plot(x_coords, y_coords, label=f"Drone {i+1}")
        plt.legend()
        plt.xlabel("X")
        plt.ylabel("Y")
        plt.title("Drone Paths with CBS and RRT-Connect")
        plt.show()

if __name__ == '__main__':
    # Define start and goal positions for four drones
    starts = [(1, 1), (2, 2), (3, 3), (4, 4)]
    goals = [(10, 10), (11, 11), (12, 12), (13, 13)]
    obstacle_list = [(5, 5, 1), (6, 7, 1.5), (8, 9, 2)]
    rand_area = [-2, 15]
    bounds = [-2, 15, -2, 15]
    drone_radius = 0.5

    # Initialize CBS for multi-drone planning
    cbs = DroneCBS(starts, goals, obstacle_list, rand_area, bounds, drone_radius)
    final_paths = cbs.plan_paths()

    if final_paths:
        print("Conflict-free paths found for all drones.")
        cbs.visualize_paths()
    else:
        print("No solution found for conflict-free paths.")
