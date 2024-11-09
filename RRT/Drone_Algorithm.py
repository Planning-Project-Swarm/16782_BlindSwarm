import matplotlib.pyplot as plt
import random
from copy import deepcopy
from RRT_Connect import RRTConnect  # Assuming RRT_Connect.py contains the RRTConnect class

class DroneCBS:
    def __init__(self, starts, goals, obstacle_list, rand_area, bounds, drone_radius=0.5):
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
        initial_constraints = {}
        max_length = 0
        # Initial planning for all drones
        for i in range(len(self.starts)):
            path = self.plan_individual_path(i, initial_constraints, max_length=None)
            if path is None:
                print(f"No path found for drone {i} during initial planning.")
                return None
            self.paths.append(path)
            max_length = max(max_length, len(path))
        # Synchronize path lengths
        for i in range(len(self.paths)):
            if len(self.paths[i]) < max_length:
                for _ in range(max_length - len(self.paths[i])):
                    self.paths[i].append(self.paths[i][-1])  # Wait at the goal
        # Run CBS to resolve conflicts
        result = self.cbs(max_length)
        return result

    def plan_individual_path(self, agent_id, constraints, max_length):
        """
        Plan a path for a single drone considering constraints and adjust to max_length.
        """
        start = self.starts[agent_id]
        goal = self.goals[agent_id]
        rrt = RRTConnect(start=start, goal=goal,
                         obstacle_list=self.obstacle_list,
                         rand_area=self.rand_area,
                         play_area=self.bounds,
                         robot_radius=self.drone_radius,
                         max_iter=1000,        # Increased maximum iterations
                         goal_sample_rate=20)  # Increased goal sampling rate
        path = rrt.planning(animation=False)
        if path is None:
            return None
        # Apply constraints to the path
        if agent_id in constraints:
            if self.violates_constraints(path, constraints[agent_id]):
                return None
        # Extend path to max_length by adding goal positions
        while len(path) < max_length:
            path.append(path[-1])  # Stay at the goal
        return path

    def violates_constraints(self, path, agent_constraints):
        """
        Check if the path violates any constraints.
        """
        for constraint in agent_constraints:
            t, x, y = constraint
            if t < len(path):
                if path[t][0] == x and path[t][1] == y:
                    return True
            else:
                # Check if the agent stays at the goal position
                if path[-1][0] == x and path[-1][1] == y:
                    return True
        return False

    def detect_conflicts(self, paths, buffer=0.5):
        """
        Detect conflicts between paths with a buffer zone to prevent near-collisions.
        """
        max_path_length = max(len(p) for p in paths)
        for t in range(max_path_length):
            positions = {}
            for agent_id, path in enumerate(paths):
                if t < len(path):
                    pos = (path[t][0], path[t][1])
                else:
                    pos = (path[-1][0], path[-1][1])  # Wait at the goal
                # Check conflicts with a buffer
                for other_pos in positions.values():
                    if self.calc_distance(pos, other_pos) < buffer:
                        return {'time': t, 'agents': [positions[other_pos], agent_id], 'pos': pos}
                positions[pos] = agent_id
        return None

    def cbs(self, max_length):
        """
        Implement the CBS algorithm with relaxed constraints.
        """
        constraints = {}
        open_set = [{'paths': self.paths, 'constraints': constraints}]
        while open_set:
            node = open_set.pop(0)
            self.paths = node['paths']
            conflict = self.detect_conflicts(self.paths)
            if conflict is None:
                return self.paths  # Conflict-free paths found
            # Resolve conflict by adding buffer-based constraints
            for agent in conflict['agents']:
                new_constraints = deepcopy(node['constraints'])
                if agent not in new_constraints:
                    new_constraints[agent] = []
                # Adding temporal range constraint around conflict
                time_range = range(max(0, conflict['time'] - 1), conflict['time'] + 2)
                for t in time_range:
                    new_constraints[agent].append((t, conflict['pos'][0], conflict['pos'][1]))
                new_paths = deepcopy(self.paths)
                new_path = self.plan_individual_path(agent, new_constraints, max_length)
                if new_path:
                    new_paths[agent] = new_path
                    open_set.append({'paths': new_paths, 'constraints': new_constraints})
        print("Failed to find conflict-free paths after multiple attempts.")
        return None

    def calc_distance(self, pos1, pos2):
        """
        Calculate the Euclidean distance between two positions.
        """
        return math.hypot(pos1[0] - pos2[0], pos1[1] - pos2[1])

    def visualize_paths(self):
        """
        Visualize paths for all drones.
        """
        colors = ['r', 'g', 'b', 'c', 'm', 'y', 'k']
        plt.figure()
        for i, path in enumerate(self.paths):
            x_coords = [x for x, y in path]
            y_coords = [y for x, y in path]
            plt.plot(x_coords, y_coords, color=colors[i % len(colors)], label=f"Drone {i+1}")
            plt.scatter(self.starts[i][0], self.starts[i][1], marker='o', color=colors[i % len(colors)])
            plt.scatter(self.goals[i][0], self.goals[i][1], marker='*', color=colors[i % len(colors)])
        # Plot obstacles
        for (ox, oy, size) in self.obstacle_list:
            circle = plt.Circle((ox, oy), size, color='gray')
            plt.gca().add_patch(circle)
        plt.legend()
        plt.xlabel("X")
        plt.ylabel("Y")
        plt.title("Paths for Drones using CBS and RRT-Connect")
        plt.axis('equal')
        plt.grid(True)
        plt.show()
