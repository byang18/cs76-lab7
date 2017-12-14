import random
import math
from operator import itemgetter
from shapely.geometry import Polygon, Point
from display_planar import TrajectoryView
from cs1lib import *
from planarsim import *

START = (25, 25, 0)
GOAL = (200, 75, 0)
WINDOW_WIDTH = 400
WINDOW_HEIGHT = 400
GOAL_THRESHOLD = 10

DEFAULT_TIMESTEP = 50
DURATION = 0.2

# OBSTACLES = []
# OBSTACLES = [([114, 65], [114, 75], [120, 65], [120, 75])]
# OBSTACLES = [([114, 65], [114, 75], [120, 65], [120, 75]),
#              ([100, 50], [100, 55], [115, 50], [115, 55])]
OBSTACLES = [([114, 65], [114, 75], [120, 65], [120, 75]),
             ([100, 50], [100, 55], [115, 50], [115, 55]),
             ([130, 100], [130, 70], [135, 100], [135, 70])]



# An edge describes how to get from start to end given a control and duration
class SearchNode:
    def __init__(self, config, parent, control):
        self.config = config
        self.parent = parent
        self.control = control

    def __str__(self):
        return str(self.config)

    def get_config(self):
        return self.config

    def get_parent(self):
        return self.parent

    def get_control(self):
        return self.control

class RRT:

    def __init__(self):
        self.start_node = SearchNode(START, None, None)
        self.goal_node = SearchNode(GOAL, None, None)
        self.obstacles = []
        for obstacle in OBSTACLES:
            self.obstacles.append(Polygon(obstacle))

        self.tree = [self.start_node]
        self.solution = []

    def is_collision(self, config):

        point = Point(config[0], config[1])
        for obstacle in self.obstacles:
            if obstacle.contains(point):
                return True

        return False

    def make_random_configuration(self):

        random_x = random.randint(0, WINDOW_WIDTH)
        random_y = random.randint(0, WINDOW_HEIGHT)
        check = (random_x, random_y, 0)

        while self.is_collision(check):
            random_x = random.randint(0, WINDOW_WIDTH)
            random_y = random.randint(0, WINDOW_HEIGHT)
            check = (random_x, random_y, 0)

        random_theta = random.uniform(0, 2 * math.pi)
        return(tuple((random_x, random_y, random_theta)))

    def get_nearest_neighbor(self, config):

        min_distance = math.inf
        min_node = None

        for node in self.tree:
            distance = get_distance(config, node.get_config())
            if distance < min_distance:
                min_distance = distance
                min_node = node

        return min_node

    def no_collisions_trajectory(self, control, neighbor):

        prev_config = None
        current_config = [0, 0, 0]

        for i in range(0, DEFAULT_TIMESTEP):
            resulting_config = build_new_config(current_config, control, DURATION)

            x = neighbor.get_config()[0] + resulting_config[0]
            y = neighbor.get_config()[1] - resulting_config[1]
            theta = resulting_config[2] + (neighbor.get_config()[2] % math.pi)
            new_config = (x, y, theta)

            if not self.is_collision(new_config):
                prev_config = new_config
                current_config = resulting_config
            else:
                return prev_config

        return prev_config

    def build_tree(self, k):

        i = 0
        while i < k:

            new_config = self.make_random_configuration()
            neighbor = self.get_nearest_neighbor(new_config)

            for control in controls_rs:

                prev_config = self.no_collisions_trajectory(control, neighbor)

                if prev_config != None:
                    new_node = SearchNode(prev_config, neighbor, control)
                    self.tree.append(new_node)

                if self.near_goal(prev_config):
                    self.solution = self.backtrack(new_node)
                    return self.solution

            print("size is: {}".format(len(self.tree)))
            print("         neighbor is: {}".format(neighbor))
            i += 1

    def backtrack(self, node):
        path = [node.get_config()]
        next_node = node.get_parent()
        while next_node is not None:
            path.append(next_node.get_config())
            next_node = next_node.get_parent()
        return path[::-1]


    def near_goal(self, config):

        distance = get_distance(config, GOAL)
        if distance <= GOAL_THRESHOLD:
            return True
        return False

    def display(self):
        clear()

        for obstacle in OBSTACLES:
            set_stroke_color(1, 0, 0)
            set_fill_color(1, 0, 0)
            draw_polygon(obstacle)

        for i in range(len(self.tree)):
            if self.tree[i].get_parent() is not None:
                set_stroke_color(0, 0, 0)
                draw_line(self.tree[i].get_config()[0],
                          self.tree[i].get_config()[1],
                          self.tree[i].get_parent().get_config()[0],
                          self.tree[i].get_parent().get_config()[1])

        for i in range(len(self.solution) - 1):
            set_stroke_color(0, 1, 0)
            draw_line(self.solution[i][0],
                      self.solution[i][1],
                      self.solution[i + 1][0],
                      self.solution[i + 1][1])

def get_distance(config1, config2):
    if (config1 is not None and config2 is not None):
        point1 = Point(config1[0], config1[1])
        point2 = Point(config2[0], config2[1])
        return point1.distance(point2)
    return math.inf

def build_new_config(old, control, timestep):
    start = transform_from_config(old)
    resulting_transform = single_action(start, control, timestep)
    resulting_configuration = config_from_transform(resulting_transform)
    return resulting_configuration

if __name__ == '__main__':
    rrt = RRT()
    p = rrt.build_tree(100000)
    print(p)
    start_graphics(rrt.display)
