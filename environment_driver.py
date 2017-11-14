import time
from shapely.geometry import Point, LineString, Polygon
from robot import Robot
from graphics import Graphics
from cs1lib import *
# from PRM import PRM


# collisions, obstacles
class Environment:

    def __init__(self, robot, buffer_radius=5):
        self.robot = robot
        self.test_robot = Robot([0, 0])
        self.buffer_radius = buffer_radius

        self.configs = []

        self.obstacles = [Point(50, 50),
                          Point(-50, -50),
                          Point(50, -50),
                          Point(-50, 50)]

    def detect_collision(self, robot):

        # collision = False

        for obstacle in self.obstacles:
            for link in robot.links:
                if link.intersection(obstacle.buffer(self.buffer_radius)):
                    return True

        return False

    def generate_valid_configuration(self):
        config = self.robot.generate_random_configuration()
        while self.detect_collision(self.robot):
            self.robot.generate_random_configuration()
        return config

    def check_motion(self, start_config, goal_config, timestep):

        # config: list of DEGREES
        # print("config 1: {}".format(start_config))
        # print("config 2: {}".format(goal_config))

        increment_by = []
        sum_distances = 0
        flip = 1

        for i in range(len(start_config)):

            difference = float(goal_config[i]) - float(start_config[i])

            if abs(difference) > 180:
                flip = -1

            increment_by.append((flip * float(difference))/timestep)
            flip = 1
            sum_distances += abs(difference)

        # incremented_config = []
        prev_config = list(start_config)
        # self.configs.append(tuple(prev_config))
        # print("first: {}".format(self.configs))

        for t in range(timestep):
            # print(prev_config)

            # incremented_config.append(tuple(prev_config))

            self.test_robot.update_configuration(prev_config)
            if self.detect_collision(self.test_robot):
                if(tuple(start_config) == (0, 90)):
                    print("COLLISION")
                    print("     start config: {}".format(start_config))
                    print("     prev config: {}".format(prev_config))
                    print("     goal config: {}".format(goal_config))
                # for config in self.configs:
                #     print(config)
                return (True, -1)

            self.configs.append(tuple(prev_config))
            # print("appending: {}".format(prev_config))
            # print("     configs right now:")
            # for config in self.configs:
            #     print("     {}".format(config))

            # i represents the angle
            for i in range(len(start_config)):
                prev_config[i] += float(increment_by[i])


        return (False, sum_distances)
            # check if range is good

        # # for each angle in the configuration
        # for i in range(len(start_config)):
        #
        #     # if negative, go backwards--CHECK THIS
        #     difference = goal_config[i] - start_config[i]
        #     print("difference: {}".format(difference))
        #
        #     step = float(difference/100)
        #     incremented_config = float(start_config[i])
        #     for timestep in range(100):
        #         incremented_config += float(step)
        #         print("new config: {}".format(incremented_config))

            # check if the difference is forward or backward for timestep

if __name__ == "__main__":
    # angles = [0, 90]
    # angles = [270, 270]
    # robot_test = robot(angles, 100)
    # environment_test = environment(robot_test, 3)

    angles = [0, 90]
    robot_test = Robot(angles, 100)
    environment_test = Environment(robot_test, 5)
    environment_test.check_motion([12, 295], [270, 270], 100)
    # environment_test.generate_valid_configuration()
    graphics = Graphics(environment_test, environment_test.configs)
    graphics.render()
