# using shapely for collision detection
import random
import math
# import copy
from operator import itemgetter
from shapely.geometry import Polygon, Point
from display_planar import TrajectoryView
from cs1lib import clear, start_graphics
from planarsim import *

START = (15, 15, math.pi/2)
GOAL = (170, 35, -math.pi/4)
WINDOW_WIDTH = 400
WINDOW_HEIGHT = 400

DEFAULT_TIMESTEP = 30
DURATION = 0.3
OBSTACLES = [Polygon(([0, 0], [0, 1], [1, 1], [1, 0]))]

# An edge describes how to get from start to end given a control and duration
class SearchNode:
    def __init__(self, config, parent, control):
        self.config = config
        self.parent = parent
        self.control = control
        # self.duration = duration

    def __str__(self):
        # s = "{} from {} with control {}".format(self.config, self.parent, self.control)
        return str(self.config)

    def get_config(self):
        return self.config

    def get_parent(self):
        return self.parent

    def get_control(self):
        return self.control

class RRT:

    def __init__(self):
        self.obstacles = OBSTACLES

        self.start = START
        self.start_node = SearchNode(START, None, None)

        self.goal = GOAL
        self.goal_node = SearchNode(GOAL, None, None)

        self.tree = [self.start_node]
        self.path = []

    # not sure if checks collisions properly
    # returns TRUE if there is a collision
    def is_collision(self, config):

        point = Point(config[0], config[1])
        for obstacle in self.obstacles:
            if obstacle.contains(point):
                return True

        return False

    # check if there is a collision in the trajectory
    # if there is, return True, if there is not, return False
    # def check_trajectory(self, config, random_control, timestep):
    #     t = timestep / 20
    #
    #     step = 0
    #     prev = config
    #     while step < 20:
    #         resulting_configuration = build_new_config(prev, random_control, t)
    #         check = Point(resulting_configuration[0], resulting_configuration[1]) # check if x or y coord is valid
    #
    #         # if there is a collision, trajectory is True
    #         if self.check_collision(check):
    #             return False
    #         prev = resulting_configuration
    #         step += 1
    #     return True

    # def new_valid_configuration(self):
    #     # start = transform_from_config([1, 2, .2])
    #     # print("config is: {}".format(config))
    #
    #     # Build a random config from a resulting config
    #     # if tuple(config) not in self.graph:
    #     #     self.graph[tuple(config)] = []
    #     # print("graph is {}: ".format(self.graph))
    #
    #     while True:
    #         random_control = random.choice(controls_rs)
    #         resulting_configuration = build_new_config(config, random_control, timestep)
    #
    #         # print("resulting_configuration is: {}".format(resulting_configuration))
    #
    #         # valid trajectory is a bool
    #         # if there is a collision, the trajectory is NOT valid (returns false)
    #         # otherwise, if there is no collision, the trajectory returns True
    #         valid_trajectory = self.check_trajectory(config,
    #                                                  random_control,
    #                                                  timestep)
    #
    #         # print(valid_trajectory)
    #         if valid_trajectory:
    #
    #
    #             # to_append = copy.deepcopy(self.graph[tuple(config)])
    #             edge = Edge(resulting_configuration, config, random_control) # need timestep?
    #             self.graph[tuple(resulting_configuration)] = edge
    #
    #             # print('get here')
    #             print('get here')
    #             break
    #
    #         print(valid_trajectory)

    def make_random_configuration(self):

        random_x = random.randint(0, WINDOW_WIDTH)
        random_y = random.randint(0, WINDOW_HEIGHT)
        check = (random_x, random_y, 0)

        while self.is_collision(check):
            random_x = random.randint(0, WINDOW_WIDTH)
            random_y = random.randint(0, WINDOW_HEIGHT)
            check = (random_x, random_y, 0)

        # print('no collision')

        random_theta = random.uniform(0, 2 * math.pi)
        return(tuple((random_x, random_y, random_theta)))

    # def get_nearest_neighbors(self, a_config):
    #     # print(config)
    #     distances = []
    #     start = Point(a_config[0], a_config[1])
    #     for point in self.graph:
    #         # print("point is {}".format(point))
    #         if(tuple((point[0], point[1])) == tuple((a_config[0], a_config[1]))):
    #             continue
    #         end = Point(point[0], point[1])
    #         distance = get_distance(start, end)
    #         distances.append(tuple((point, distance)))
    #
    #     l = sorted(distances, key=itemgetter(1), reverse=True)
    #     l = l[:15]
    #     to_return = []
    #     for item in l:
    #         to_return.append(item[0])
    #
    #     # print(" neighbors: {}".format(to_return))
    #     return to_return

    def get_nearest_neighbor(self, config):

        min_distance = math.inf
        min_node = None

        for node in self.tree:
            distance = get_distance(config, node.get_config())
            if distance < min_distance:
                min_distance = distance
                min_node = node

        return min_node

    def build_tree(self, k):

        i = 0
        while True:

            new_config = self.make_random_configuration()
            neighbor = self.get_nearest_neighbor(new_config)

            for control in controls_rs:
                prev_config = None
                resulting_transform = sample_trajectory([control], [DURATION], DURATION, DEFAULT_TIMESTEP)

                for transform in resulting_transform:

                    # resulting_transform = single_action(neighbor.get_config(), control, 0.3)
                    # resulting_configuration = config_from_transform(resulting_transform)

                    x, y, theta = config_from_transform(transform)

                    new_x = neighbor.get_config()[0] + x
                    new_y = neighbor.get_config()[1] - y
                    new_theta = theta + (neighbor.get_config()[2] % math.pi)
                    new_config = (new_x, new_y, new_theta)

                    if not self.is_collision(new_config):
                        prev_config = new_config
                    else:
                        break

                new_node = SearchNode(prev_config, neighbor, control)
                self.tree.append(new_node)

                if self.near_goal(prev_config):
                    path = self.backtrack(new_node)
                    return path

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

        distance = get_distance(config, self.goal)
        if distance <= 20:
            return True
        return False

    # def get_nearest_to_start(self, start_config):
    #     min_distance = math.inf
    #     min_config = random.choice(list(self.graph.keys()))
    #     for a_config in self.graph:
    #         start = Point(a_config[0], a_config[1])
    #         end = Point(start_config[0], start_config[1])
    #         distance = get_distance(start, end)
    #         if distance < min_distance:
    #             print("gggg")
    #             min_distance = distance
    #             min_config = a_config
    #
    #     return min_config
    #
    # def find_path(self, goal_config):
    #     current = self.get_nearest_to_goal(goal_config)
    #     end = self.get_nearest_to_start(self.starting)
    #
    #     print("START")
    #     print("current: {}".format(current))
    #     print("new_start_config: {}".format(end))
    #     print("num nodes: {}".format(len(self.graph)))
    #     # print("keys: {}".format(self.graph.keys()))
    #
    #     edges = []
    #     list_controls = []
    #     visited = set()
    #
    #     i = 0
    #     while current != end and i < 15:
    #         # if current not in visited:
    #         print(current)
    #         print(i)
    #         edge = self.graph[tuple(current)]
    #         edges.append(edge)
    #         current = edge.get_parent()
    #         print("new_start_config: {}".format(end))
    #         i += 1
    #
    #     print("edges: {}".format(edges))
    #     edges.reverse()
    #     for edge in edges:
    #         list_controls.append(edge.control)
    #
    #     return list_controls

    # def get_trajectory(self, controls, starting):
    #
    #     timing = []
    #     for control in controls:
    #         timing.append(1)
    #
    #     t = sample_trajectory(controls, timing, len(timing), 40, starting)
    #     return t


def get_distance(config1, config2):
    point1 = Point(config1[0], config1[1])
    point2 = Point(config2[0], config2[1])
    return point1.distance(point2)

def build_new_config(old, control, timestep):
    start = transform_from_config(old)
    resulting_transform = single_action(start, control, timestep)
    resulting_configuration = config_from_transform(resulting_transform)
    return resulting_configuration

# def display():
#     clear()
#     tview.draw()

if __name__ == '__main__':
    # starting = [30, 30, 0]
    rrt = RRT()
    # start_config = [1, 2, .2]
    # print(rrt.new_valid_configuration(start_config, 5))


    p = rrt.build_tree(1000)
    print(p)
    # for c in rrt.graph:
    #     print("Config: {}".format(c))
    #     print("         " + str(rrt.graph[c]))
    #
    # goal = [370, 370, 0]
    # controls = rrt.find_path(goal)
    # print("controls: {}".format(controls))
    # t = rrt.get_trajectory(controls, goal)
    # #
    # tview = TrajectoryView(t, 400, 400, 40)
    # #
    # start_graphics(display, width=800, height=800)
