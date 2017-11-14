import random
import math
from collections import deque
from operator import itemgetter
# from SearchNode import SearchNode
from robot import Robot
from environment_driver import Environment

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

    def __init__(self, e):
        self.env = e
        self.starting_config = self.env.robot.thetas
        self.path = []
        self.graph = {}

    def bfs_search(self, node=None):

        queue = deque()
        visited = {}
        path = []
        # goal_state = self.e.robot.goal_state
        # print(tuple(self.env.robot.goal_state))

        if node is None:
            node = SearchNode(tuple(self.starting_config))
            queue.appendleft(node)
            while len(queue) > 0:
                current = queue.pop()
                current_state = tuple(current.get_state())
                # print(tuple(current_state))
                # print("current: " + str(current.state))
                visited[tuple(current_state)] = current
                if tuple(current_state) == tuple(self.env.robot.goal_state):
                    path = self.backtrack(current)
                    break
                successors = self.graph[tuple(current_state)]
                for state in successors:
                    # print("     suc: " + str(state))
                    if tuple(state) not in visited:
                        add_node = SearchNode(tuple(state), current)
                        queue.appendleft(add_node)

        print("SOLUTION:")
        return path
        #     count = search_problem.get_count()
        #     solution = SearchSolution(search_problem, "BFS", path, count)
        #
        # return solution

    def backtrack(self, node):
        path = [node.get_state()]
        next_node = node.get_parent()
        while next_node is not None:
            path.append(next_node.get_state())
            next_node = next_node.get_parent()
        return path[::-1]

    def generate_vertices(self):

        self.graph[tuple(self.starting_config)] = []
        self.graph[tuple(self.env.robot.goal_state)] = []

        for i in range(0, 100):
            config = tuple(self.env.generate_valid_configuration())
            if config not in self.graph:
                self.graph[config] = []

    def fill_neighbors(self, k=15):

        m = 0
        for config in self.graph:

            no_collisions = []

            for end_config in self.graph:

                if(config == end_config):
                    continue

                # if it's good then add to the list
                is_collision, diff = self.env.check_motion(config, end_config, 10)

                if is_collision:
                    continue

                m += 1
                if (m % 50 == 0):
                    print(m)

                no_collisions.append(tuple((end_config, diff)))
                # no_collisions.append(end_config)

            no_collisions.sort(key=itemgetter(1), reverse=True)

            to_add = []
            for i in range(min(k, len(no_collisions))):
                to_add.append(no_collisions[i][0])
            # print(no_collisions)
            self.graph[config] = to_add

if __name__ == "__main__":
    end_angles = [math.radians(10), math.radians(355), math.radians(90), math.radians(355)]
    # starting_angles = [math.radians(270), math.radians(10), math.radians(270), math.radians(10)]
    starting_angles = [3.774016428646744, 1.0914423299586218, 5.4175286331656896, 0.22380400583459362]
    # angles = [315, 270]
    # angles = [270, 270]
    robot_test = Robot(starting_angles, end_angles, 50)
    environment_test = Environment(robot_test, 5)
    prm = PRM(environment_test)
    prm.generate_vertices()
    # print(prm.graph)
    # print(prm.starting_config)
    prm.fill_neighbors()
    print(prm.graph)
    print(prm.bfs_search())
