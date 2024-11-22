import math
import random
import os
import argparse
import matplotlib.pyplot as plt
import numpy as np

show_animation = True

class RRT:
    """
    Class for RRT planning with motion primitives and rectangular obstacles
    """

    class Node:
        """
        RRT Node with heading angle (theta)
        """

        def __init__(self, x, y, theta=0.0, t = 0.0):
            self.x = x
            self.y = y
            # Heading angle
            self.theta = theta
            self.t = t
            self.path_x = []
            self.path_y = []
            self.parent = None

    class AreaBounds:

        def __init__(self, area):
            self.xmin = float(area[0])
            self.xmax = float(area[1])
            self.ymin = float(area[2])
            self.ymax = float(area[3])

    def __init__(self,
                 start,
                 goal,
                 obstacle_list,
                 rand_area,
                 expand_dis=0.5,  # Reduced for finer movements
                 path_resolution=0.1,  # Increased resolution
                 goal_sample_rate=10,  # Increased to sample goal more often
                 max_iter=2000,  # Increased to allow more iterations
                 play_area=None,
                 robot_radius=0.8,
                 random_seed=7
                 ):
        """
        Setting Parameter

        start:Start Position [x,y]
        goal:Goal Position [x,y]
        obstacleList:obstacle Positions [{'x1': x1, 'y1': y1, 'x2': x2, 'y2': y2},...]
        randArea:Random Sampling Area [min,max]
        play_area:stay inside this area [xmin,xmax,ymin,ymax]
        robot_radius: robot body modeled as circle with given radius

        """
        self.start = self.Node(start[0], start[1], start[2], 0.0)
        # dummy t
        self.end = self.Node(goal[0], goal[1], goal[2], 100.0)
        self.min_rand = rand_area[0]
        self.max_rand = rand_area[1]
        if play_area is not None:
            self.play_area = self.AreaBounds(play_area)
        else:
            self.play_area = None
        self.expand_dis = expand_dis
        self.path_resolution = path_resolution
        self.goal_sample_rate = goal_sample_rate
        self.max_iter = max_iter
        self.obstacle_list = obstacle_list
        self.node_list = []
        self.robot_radius = robot_radius

        # TODO: hard-coded discrete time step 
        self.dt = 1.0

        # Define motion primitives: [distance, angle_change (radians)]
        self.motion_primitives = [
            [self.expand_dis, 0],  # Move forward
            [self.expand_dis, np.deg2rad(15)],  # 15 degrees left
            [self.expand_dis, np.deg2rad(-15)],  # 15 degrees right
            [self.expand_dis, np.deg2rad(30)],  # 30 degrees left
            [self.expand_dis, np.deg2rad(-30)],  # 30 degrees right
            [self.expand_dis, np.deg2rad(45)],  # 45 degrees left
            [self.expand_dis, np.deg2rad(-45)],  # 45 degrees right
            [self.expand_dis, np.deg2rad(60)],  # 60 degrees left
            [self.expand_dis, np.deg2rad(-60)],  # 60 degrees right
        ]

        self.path = None
        random.seed(random_seed)

    def planning(self, animation=True, constraints=None):
        """
        RRT path planning with motion primitives

        animation: flag for animation on or off
        """

        self.node_list = [self.start]
        for i in range(self.max_iter):
            rnd_node = self.get_random_node()
            nearest_ind = self.get_nearest_node_index(self.node_list, rnd_node)
            nearest_node = self.node_list[nearest_ind]

            # Expand nodes using motion primitives
            new_nodes = []
            for motion in self.motion_primitives:
                new_node = self.apply_motion(nearest_node, motion)
                if self.check_if_outside_play_area(new_node, self.play_area) and \
                        self.check_collision(
                            new_node, self.obstacle_list, self.robot_radius):
                    if constraints is None or self.check_constraints(new_node, constraints):
                        new_nodes.append(new_node)
                        
            if not new_nodes:
                continue

            # Choose the new node that is closest to the random node
            dlist = [self.calc_distance(new_node, rnd_node) for new_node in new_nodes]
            min_ind = dlist.index(min(dlist))
            best_new_node = new_nodes[min_ind]
            self.node_list.append(best_new_node)

            # Check if goal is reached
            if self.calc_dist_to_goal(best_new_node.x, best_new_node.y) <= self.expand_dis:
                final_node = self.steer(best_new_node, self.end,
                                        self.expand_dis)
                if self.check_collision(
                        final_node, self.obstacle_list, self.robot_radius):
                    self.node_list.append(final_node)
                    self.path = self.generate_final_course()
                    return self.path
                else:
                    print("Goal is not reachable")

            if animation and i % 100 == 0:
                self.draw_graph(rnd_node, False)

        return None  # Cannot find path
    
    def reset(self):
        self.node_list = []
        self.obstacle_list = []
        self.start = None
        self.end = None
        self.path = None
        self.play_area = None
        
    def apply_motion(self, node, motion):
        """
        Apply a motion primitive to a node

        motion: [distance, angle_change]
        """
        expand_dis, angle_change = motion
        new_theta = node.theta + angle_change

        new_node = self.Node(node.x, node.y, new_theta, node.t + self.dt)
        new_node.parent = node

        # Compute new position
        new_node.x += expand_dis * math.cos(new_theta)
        new_node.y += expand_dis * math.sin(new_theta)

        # Store path for plotting
        new_node.path_x = [node.x, new_node.x]
        new_node.path_y = [node.y, new_node.y]

        return new_node

    def steer(self, from_node, to_node, extend_length=float("inf")):

        new_node = self.Node(from_node.x, from_node.y, from_node.theta, from_node.t + self.dt)
        d, theta = self.calc_distance_and_angle(new_node, to_node)

        angle_difference = self.normalize_angle(theta - from_node.theta)
        # Limit the angle change
        if abs(angle_difference) > np.deg2rad(60.0):
            angle_difference = np.clip(angle_difference, -np.deg2rad(60.0), np.deg2rad(60.0))

        new_node.theta += angle_difference

        # Move towards to_node
        move_distance = min(self.expand_dis, d)
        new_node.x += move_distance * math.cos(new_node.theta)
        new_node.y += move_distance * math.sin(new_node.theta)

        new_node.path_x = [from_node.x, new_node.x]
        new_node.path_y = [from_node.y, new_node.y]

        new_node.parent = from_node

        return new_node

    def generate_final_course(self):
        course_len = len(self.node_list)
        # path = [[self.end.x, self.end.y, cur_time]]
        path = []
        node = self.node_list[course_len - 1]
        while node.parent is not None:
            path.append([node.x, node.y, node.theta, node.t])
            node = node.parent
        path.append([node.x, node.y, node.theta, node.t])

        return path[::-1]  # Reverse path

    def calc_dist_to_goal(self, x, y):
        dx = x - self.end.x
        dy = y - self.end.y
        return math.hypot(dx, dy)

    def get_random_node(self):
        if random.randint(0, 100) > self.goal_sample_rate:
            rnd = self.Node(
                random.uniform(self.min_rand, self.max_rand),
                random.uniform(self.min_rand, self.max_rand),
                random.uniform(-math.pi, math.pi),
                100.0)
        else:  # goal point sampling
            rnd = self.Node(self.end.x, self.end.y, self.end.theta, 100.0)
        return rnd

    def draw_graph(self, rnd=None, show_path=False):
        plt.clf()
        # for stopping simulation with the esc key.
        plt.gcf().canvas.mpl_connect(
            'key_release_event',
            lambda event: [exit(0) if event.key == 'escape' else None])
        if rnd is not None:
            plt.plot(rnd.x, rnd.y, "^k")
            if self.robot_radius > 0.0:
                self.plot_circle(rnd.x, rnd.y, self.robot_radius, '-r')
        for node in self.node_list:
            if node.parent:
                plt.plot(node.path_x, node.path_y, "-g")
                # Plot heading direction
                self.plot_arrow(node.x, node.y, node.theta)

        for obs in self.obstacle_list:
            rect_x = [obs['x1'], obs['x2'], obs['x2'], obs['x1'], obs['x1']]
            rect_y = [obs['y1'], obs['y1'], obs['y2'], obs['y2'], obs['y1']]
            plt.fill(rect_x, rect_y, "gray")

        if self.play_area is not None:
            plt.plot([self.play_area.xmin, self.play_area.xmax,
                      self.play_area.xmax, self.play_area.xmin,
                      self.play_area.xmin],
                     [self.play_area.ymin, self.play_area.ymin,
                      self.play_area.ymax, self.play_area.ymax,
                      self.play_area.ymin],
                     "-k")

        plt.plot(self.start.x, self.start.y, "or")
        plt.plot(self.end.x, self.end.y, "xr")
        plt.axis("equal")
        plt.axis([self.min_rand - 5, self.max_rand + 5, self.min_rand - 5, self.max_rand + 5])
        plt.grid(True)
        plt.pause(0.01)
        if show_path:
            plt.plot([x for (x, y, th, t) in self.path], [y for (x, y, th, t) in self.path], '-r')
        plt.pause(0.01)
        plt.show()


    @staticmethod
    def plot_circle(x, y, size, color="-b"):  # pragma: no cover
        deg = list(range(0, 360, 5))
        deg.append(0)
        xl = [x + size * math.cos(np.deg2rad(d)) for d in deg]
        yl = [y + size * math.sin(np.deg2rad(d)) for d in deg]
        plt.plot(xl, yl, color)

    @staticmethod
    def plot_arrow(x, y, yaw, length=0.3, width=0.1):
        plt.arrow(x, y, length * math.cos(yaw), length * math.sin(yaw),
                  head_length=width, head_width=width)

    @staticmethod
    def get_nearest_node_index(node_list, rnd_node):
        dlist = [(node.x - rnd_node.x) ** 2 + (node.y - rnd_node.y) ** 2
                 for node in node_list]
        minind = dlist.index(min(dlist))

        return minind

    @staticmethod
    def check_if_outside_play_area(node, play_area):

        if play_area is None:
            return True  # no play_area was defined, every pos should be ok

        if node.x < play_area.xmin or node.x > play_area.xmax or \
                node.y < play_area.ymin or node.y > play_area.ymax:
            return False  # outside - bad
        else:
            return True  # inside - ok

    def check_constraints(self, node, constraints):
        for constraint in constraints:
            cx, cy, ct = constraint
            if abs(node.t - ct) < self.dt:
                if math.hypot(node.x - cx, node.y - cy) < self.robot_radius:
                    return False
        return True

    def check_collision(self, node, obstacleList, robot_radius):
        if node is None:
            return False

        for obs in obstacleList:
            rect = {
                'x_min': min(obs['x1'], obs['x2']) - robot_radius,
                'x_max': max(obs['x1'], obs['x2']) + robot_radius,
                'y_min': min(obs['y1'], obs['y2']) - robot_radius,
                'y_max': max(obs['y1'], obs['y2']) + robot_radius
            }

            for i in range(len(node.path_x) - 1):
                x1, y1 = node.path_x[i], node.path_y[i]
                x2, y2 = node.path_x[i + 1], node.path_y[i + 1]
                if self.line_intersects_rect(x1, y1, x2, y2, rect):
                    return False  # Collision

        return True  # Safe

    def line_intersects_rect(self, x1, y1, x2, y2, rect):
        # If the line segment is completely outside the rectangle, no collision
        if (max(x1, x2) < rect['x_min'] or min(x1, x2) > rect['x_max'] or
            max(y1, y2) < rect['y_min'] or min(y1, y2) > rect['y_max']):
            return False

        # If either end is inside the rectangle, collision
        if self.point_in_rect(x1, y1, rect) or self.point_in_rect(x2, y2, rect):
            return True

        # Check for intersection with rectangle sides
        # Line from (x1, y1) to (x2, y2)
        # Rectangle sides:
        # Left: x = rect['x_min']
        # Right: x = rect['x_max']
        # Bottom: y = rect['y_min']
        # Top: y = rect['y_max']

        # Check intersection with each side of the rectangle
        if (self.line_line_intersection(x1, y1, x2, y2, rect['x_min'], rect['y_min'], rect['x_min'], rect['y_max']) or
            self.line_line_intersection(x1, y1, x2, y2, rect['x_max'], rect['y_min'], rect['x_max'], rect['y_max']) or
            self.line_line_intersection(x1, y1, x2, y2, rect['x_min'], rect['y_min'], rect['x_max'], rect['y_min']) or
            self.line_line_intersection(x1, y1, x2, y2, rect['x_min'], rect['y_max'], rect['x_max'], rect['y_max'])):
            return True

        return False

    @staticmethod
    def point_in_rect(x, y, rect):
        return rect['x_min'] <= x <= rect['x_max'] and rect['y_min'] <= y <= rect['y_max']

    @staticmethod
    def line_line_intersection(x1, y1, x2, y2, x3, y3, x4, y4):
        # Compute the determinant
        denominator = (x1 - x2)*(y3 - y4) - (y1 - y2)*(x3 - x4)
        if denominator == 0:
            return False  # Lines are parallel

        # Compute the intersection point
        px = ((x1*y2 - y1*x2)*(x3 - x4) - (x1 - x2)*(x3*y4 - y3*x4)) / denominator
        py = ((x1*y2 - y1*x2)*(y3 - y4) - (y1 - y2)*(x3*y4 - y3*x4)) / denominator

        # Check if the intersection point is on both line segments
        if (min(x1, x2) <= px <= max(x1, x2) and
            min(y1, y2) <= py <= max(y1, y2) and
            min(x3, x4) <= px <= max(x3, x4) and
            min(y3, y4) <= py <= max(y3, y4)):
            return True
        else:
            return False

    @staticmethod
    def calc_distance_and_angle(from_node, to_node):
        dx = to_node.x - from_node.x
        dy = to_node.y - from_node.y
        d = math.hypot(dx, dy)
        theta = math.atan2(dy, dx)
        return d, theta

    @staticmethod
    def normalize_angle(angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi

        while angle < -math.pi:
            angle += 2.0 * math.pi

        return angle

    @staticmethod
    def calc_distance(node1, node2):
        dx = node1.x - node2.x
        dy = node1.y - node2.y
        return math.hypot(dx, dy)

def readMapFile(filename):
    with open(filename, 'r') as file:
        lines = file.readlines()

    robots = []
    obstacles = []
    goals = []

    for line in lines:
        parts = line.strip().split(',')
        if parts[0] == 'robot':
            robots.append({
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
        elif parts[0] == 'goal':
            goals.append({
                'id': int(parts[1]),
                'x': float(parts[2]),
                'y': float(parts[3]),
                'orientation': float(parts[4])
            })

    return robots, obstacles, goals

def writePlanFile(filename, path):
    # robot_id = 1  # Assuming robot ID is 1
    # dt = 1.0      # Time step

    with open(filename, 'w') as f:
        for waypoint in path:
            # x, y, th, t
            # print(waypoint)
            f.write("{},{},{},{}\n".format(waypoint[0], waypoint[1], waypoint[2], waypoint[3]))

def main():
    parser = argparse.ArgumentParser(description="Run RRT for a single map file.")
    parser.add_argument('input_file', type=str, help="Name of the input map file (located in 'maps' folder).")
    args = parser.parse_args()

    maps_folder = "maps"
    input_file_path = os.path.join(maps_folder, args.input_file)

    # map_filename = 'map.txt'
    robots, obstacles, goals = readMapFile(input_file_path)
    print("Start RRT path planning with motion primitives")

    # Assuming only one robot and one goal for now
    robot = robots[0]
    goal = goals[0]

    # print(robot)
    # print(goal)

    # Set Initial parameters
    rrt = RRT(
        start=[robot['x'], robot['y'], robot['orientation']],
        goal=[goal['x'], goal['y'], goal['orientation']],
        rand_area=[0, 20],
        obstacle_list=obstacles,
        play_area=[0, 20, 0, 20],
        robot_radius=0.03
    )
    path = rrt.planning(animation=show_animation)

    if path is None:
        print("Cannot find path")
    else:
        print("Found path!")
        writePlanFile(args.input_file.rstrip('.txt') + '_plan.txt', path)
        # Draw final path
        if show_animation:
            rrt.draw_graph(None, True)

if __name__ == '__main__':
    main()
