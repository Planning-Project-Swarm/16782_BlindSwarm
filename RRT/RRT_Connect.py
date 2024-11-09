import math
import random
import matplotlib.pyplot as plt
import numpy as np

show_animation = True


class RRTConnect:
    """
    Class for RRT-Connect planning
    """

    class Node:
        """
        RRT Node
        """

        def __init__(self, x, y):
            self.x = x
            self.y = y
            self.parent = None
            self.path_x = []
            self.path_y = []

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
                 expand_dis=3.0,
                 path_resolution=0.5,
                 goal_sample_rate=5,
                 max_iter=500,
                 play_area=None,
                 robot_radius=0.0,
                 ):
        """
        Setting Parameter

        start: Start Position [x, y]
        goal: Goal Position [x, y]
        obstacle_list: obstacle Positions [[x, y, size],...]
        rand_area: Random Sampling Area [min, max]
        play_area: stay inside this area [xmin, xmax, ymin, ymax]
        robot_radius: robot body modeled as circle with given radius
        """
        self.start = self.Node(start[0], start[1])
        self.end = self.Node(goal[0], goal[1])
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
        self.robot_radius = robot_radius

        # Initialize two trees
        self.tree_start = [self.start]
        self.tree_goal = [self.end]
        self.final_path = None  # To store the final path

    def planning(self, animation=True):
        """
        RRT-Connect path planning

        animation: flag for animation on or off
        """
        for i in range(self.max_iter):
            # Sample random node
            rnd_node = self.get_random_node()

            # Extend start tree towards random node
            new_node_start = self.extend(self.tree_start, rnd_node)
            if new_node_start:
                # Try to connect goal tree to the new node
                new_node_goal = self.connect(self.tree_goal, new_node_start)
                if new_node_goal:
                    # If trees are connected, generate final path
                    self.final_path = self.generate_final_course(new_node_start, new_node_goal)
                    if animation:
                        self.draw_graph(rnd_node)
                    return self.final_path

            # Swap trees for the next iteration
            self.tree_start, self.tree_goal = self.tree_goal, self.tree_start

            if animation and i % 5 == 0:
                self.draw_graph(rnd_node)

        return None  # Cannot find path

    def extend(self, tree, rnd_node):
        nearest_ind = self.get_nearest_node_index(tree, rnd_node)
        nearest_node = tree[nearest_ind]
        new_node = self.steer(nearest_node, rnd_node, self.expand_dis)

        if self.check_if_outside_play_area(new_node, self.play_area) and \
           self.check_collision(new_node, self.obstacle_list, self.robot_radius):
            tree.append(new_node)
            return new_node
        return None

    def connect(self, tree, target_node):
        """
        Try to connect the tree to the target node by continuously extending
        """
        while True:
            nearest_ind = self.get_nearest_node_index(tree, target_node)
            nearest_node = tree[nearest_ind]
            new_node = self.steer(nearest_node, target_node, self.expand_dis)

            if not self.check_collision(new_node, self.obstacle_list, self.robot_radius):
                return None

            tree.append(new_node)

            if self.calc_distance(new_node, target_node) < self.path_resolution:
                return new_node

            if not self.check_if_outside_play_area(new_node, self.play_area):
                return None

    def steer(self, from_node, to_node, extend_length=float("inf")):
        new_node = self.Node(from_node.x, from_node.y)
        new_node.parent = from_node

        d, theta = self.calc_distance_and_angle(new_node, to_node)

        extend_length = min(extend_length, d)

        n_expand = math.floor(extend_length / self.path_resolution)

        for _ in range(n_expand):
            new_node.x += self.path_resolution * math.cos(theta)
            new_node.y += self.path_resolution * math.sin(theta)
            new_node.path_x.append(new_node.x)
            new_node.path_y.append(new_node.y)

        d, _ = self.calc_distance_and_angle(new_node, to_node)
        if d <= self.path_resolution:
            new_node.x = to_node.x
            new_node.y = to_node.y
            new_node.path_x.append(new_node.x)
            new_node.path_y.append(new_node.y)

        return new_node

    def generate_final_course(self, node_start, node_goal):
        # Path from start to connection point
        path_start = []
        node = node_start
        while node.parent is not None:
            path_start.append([node.x, node.y])
            node = node.parent
        path_start.append([node.x, node.y])

        # Path from goal to connection point
        path_goal = []
        node = node_goal
        while node.parent is not None:
            path_goal.append([node.x, node.y])
            node = node.parent
        path_goal.append([node.x, node.y])

        # Combine paths
        path = path_start[::-1] + path_goal
        return path

    def calc_distance(self, node1, node2):
        dx = node1.x - node2.x
        dy = node1.y - node2.y
        return math.hypot(dx, dy)

    def get_random_node(self):
        if random.randint(0, 100) > self.goal_sample_rate:
            rnd = self.Node(
                random.uniform(self.min_rand, self.max_rand),
                random.uniform(self.min_rand, self.max_rand))
        else:
            rnd = self.Node(self.end.x, self.end.y)
        return rnd

    def draw_graph(self, rnd=None):
        plt.clf()
        if rnd is not None:
            plt.plot(rnd.x, rnd.y, "^k")
        # Draw start tree
        for node in self.tree_start:
            if node.parent:
                plt.plot(node.path_x, node.path_y, "-g")
        # Draw goal tree
        for node in self.tree_goal:
            if node.parent:
                plt.plot(node.path_x, node.path_y, "-r")
        # Draw obstacles
        for (ox, oy, size) in self.obstacle_list:
            self.plot_circle(ox, oy, size)
        # Draw final path if it exists
        if self.final_path is not None:
            plt.plot([x for (x, y) in self.final_path], [y for (x, y) in self.final_path], '-b', linewidth=3, label="Final Path")
            plt.legend()
        if self.play_area is not None:
            plt.plot([self.play_area.xmin, self.play_area.xmax,
                      self.play_area.xmax, self.play_area.xmin,
                      self.play_area.xmin],
                     [self.play_area.ymin, self.play_area.ymin,
                      self.play_area.ymax, self.play_area.ymax,
                      self.play_area.ymin],
                     "-k")
        plt.plot(self.start.x, self.start.y, "xr")
        plt.plot(self.end.x, self.end.y, "xr")
        plt.axis([self.min_rand, self.max_rand, self.min_rand, self.max_rand])
        plt.axis('equal')
        plt.grid(True)
        plt.pause(0.01)

    @staticmethod
    def plot_circle(x, y, size, color="-b"):
        deg = np.linspace(0, 360, 72)
        deg = np.append(deg, 0)
        xl = [x + size * math.cos(np.deg2rad(d)) for d in deg]
        yl = [y + size * math.sin(np.deg2rad(d)) for d in deg]
        plt.plot(xl, yl, color)

    @staticmethod
    def get_nearest_node_index(node_list, target_node):
        dlist = [(node.x - target_node.x) ** 2 + (node.y - target_node.y) ** 2 for node in node_list]
        min_index = dlist.index(min(dlist))
        return min_index

    @staticmethod
    def check_if_outside_play_area(node, play_area):
        if play_area is None:
            return True
        if node.x < play_area.xmin or node.x > play_area.xmax or \
           node.y < play_area.ymin or node.y > play_area.ymax:
            return False
        return True

    @staticmethod
    def check_collision(node, obstacle_list, robot_radius):
        if node is None:
            return False

        for (ox, oy, size) in obstacle_list:
            dx_list = [ox - x for x in node.path_x]
            dy_list = [oy - y for y in node.path_y]
            d_list = [dx ** 2 + dy ** 2 for (dx, dy) in zip(dx_list, dy_list)]

            if min(d_list) <= (size + robot_radius) ** 2:
                return False  # collision

        return True  # safe

    @staticmethod
    def calc_distance_and_angle(from_node, to_node):
        dx = to_node.x - from_node.x
        dy = to_node.y - from_node.y
        distance = math.hypot(dx, dy)
        theta = math.atan2(dy, dx)
        return distance, theta


def main(gx=6.0, gy=10.0):
    print("Start RRT-Connect planning")

    obstacle_list = [
        (5, 5, 1),
        (3, 6, 2),
        (3, 8, 2),
        (3, 10, 2),
        (7, 5, 2),
        (9, 5, 2),
        (8, 10, 1)
    ]  # [x, y, radius]

    # Set Initial parameters
    rrt_connect = RRTConnect(
        start=[0, 0],
        goal=[gx, gy],
        rand_area=[-2, 15],
        obstacle_list=obstacle_list,
        robot_radius=0.8
    )

    path = rrt_connect.planning(animation=show_animation)

    if path is None:
        print("Cannot find path")
    else:
        print("Found path!")
        # Draw final path
        if not show_animation:
            # If animation is off, draw the final path
            rrt_connect.draw_graph()


if __name__ == '__main__':
    main()
