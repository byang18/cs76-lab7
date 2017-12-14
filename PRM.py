import random
import math
from collections import deque
from operator import itemgetter
from shapely.geometry import Point, LineString, Polygon
from robot import Robot

PI = math.pi
START = (PI, 1.5 * PI, PI/2, 1.5 * PI)
GOAL = (0, PI/2, 1.5 * PI, PI/2)
BUFFER_RADIUS = 3
TIMESTEP = 20
NUM_VERTICES = 100
OBSTACLES = [Point(0, 50),
             Point(0, -50)]

class SearchNode:

    def __init__(self, state, parent=None):
        self.state = state
        self.parent = parent

    def __str__(self):
        return("Node is " + str(self.state) + ", Parent is " + str(self.parent))

    def get_state(self):
        return self.state

    def get_parent(self):
        return self.parent

class PRM:

    def __init__(self, robot):
        self.robot = robot
        self.path = []
        self.road_map = {}

        self.obstacles = OBSTACLES

        self.configs = []

    def bfs_search(self, node=None):

        queue = deque()
        visited = {}
        path = []

        if node is None:
            node = SearchNode(tuple(START))
            queue.appendleft(node)
            while len(queue) > 0:
                current = queue.pop()
                current_state = tuple(current.get_state())
                print(current_state)
                visited[tuple(current_state)] = current
                if tuple(current_state) == tuple(GOAL):
                    path = self.backtrack(current)
                    break

                successors = self.road_map[current_state]
                for state in successors:
                    if tuple(state) not in visited:
                        add_node = SearchNode(tuple(state), current)
                        queue.appendleft(add_node)

        print("SOLUTION:")
        return path

    def backtrack(self, node):
        path = [node.get_state()]
        next_node = node.get_parent()
        while next_node is not None:
            path.append(next_node.get_state())
            next_node = next_node.get_parent()
        return path[::-1]

    def nearest_neighbors(self, config):
        distances = []
        for neighbor in self.road_map:

            if neighbor == config:
                continue

            distance = get_sum_angular_distance(config, neighbor)
            distances.append(tuple((neighbor, distance)))

        l = sorted(distances, key=itemgetter(1))
        l = l[:15]
        to_return = []
        for item in l:
            to_return.append(item[0])

        return to_return

    def add_vertex(self, config):

        if config not in self.road_map:
            self.road_map[config] = []

        neighbors = self.nearest_neighbors(config)
        for neighbor in neighbors:
            if self.no_collision(config, neighbor):
                l = self.road_map[config]
                l.append(neighbor)
                self.road_map[config] = l

                l = self.road_map[neighbor]
                l.append(config)
                self.road_map[neighbor] = l

    def no_collision(self, start_config, goal_config, draw=False):

        prev_config = list(start_config)

        for i in range(len(start_config)):

            difference = angular_distance(goal_config[i], start_config[i])

            if ((start_config[i] + difference) % (2 * PI)) == goal_config[i]:
                step = difference/TIMESTEP
            else:
                step = -difference/TIMESTEP

            for t in range(TIMESTEP):
                # for drawing purposes
                if draw:
                    self.configs.append(tuple(prev_config))

                prev_config[i] = prev_config[i] + step
                self.robot.update_configuration(prev_config)
                if self.detect_collision(self.robot):
                    return False

            prev_config[i] = start_config[i]

        return True

    def detect_collision(self, robot):

        for obstacle in self.obstacles:
            for link in robot.links:
                if link.intersection(obstacle.buffer(BUFFER_RADIUS)):
                    return True

        return False

    def generate_valid_configuration(self):
        config = self.robot.generate_random_configuration()
        while self.detect_collision(self.robot):
            config = self.robot.generate_random_configuration()
        return config

    def build_roadmap(self, start, end, k):

        i = 0
        while (i < k):
            config = self.generate_valid_configuration()
            self.add_vertex(config)
            print(i)
            i += 1

        self.add_vertex(start)
        self.add_vertex(end)

def get_sum_angular_distance(config1, config2):
    i = 0
    s = 0
    for deg in list(config1):
        distance = angular_distance(deg, config2[i])
        s += distance
        i += 1
    return s

def ad_helper(end, start):
    return abs(float(end) - float(start))

def angular_distance(end, start):

    d = ad_helper(end, start)
    return min(d, (2 * math.pi) - d)

if __name__ == "__main__":
    r = Robot(START)
    prm = PRM(r)

    prm.build_roadmap(START, GOAL, NUM_VERTICES)
    print(prm.road_map)

    for j in prm.road_map:
        print(len(prm.road_map[j]))

    print(prm.bfs_search())
