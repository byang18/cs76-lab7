import time
import math
from shapely.geometry import Point, LineString, Polygon
from robot import Robot
from graphics import Graphics
from cs1lib import *
# from PRM import PRM


# collisions, obstacles
class Environment:

    def __init__(self, robot, buffer_radius=5):
        self.robot = robot
        self.buffer_radius = buffer_radius

        self.configs = []

        # self.obstacles = []
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

        for i in range(len(start_config)):

            difference, forward = get_min_travel(goal_config[i], start_config[i])

            if forward:
                step = difference/timestep
                # print(step)
            else:
                step = -difference/timestep

            increment_by.append(step)
            sum_distances += abs(difference)

        # print(len(start_config))
        # print(increment_by)

        # incremented_config = []
        prev_config = list(start_config)
        # self.configs.append(tuple(prev_config))
        # print("first: {}".format(self.configs))
        for t in range(timestep):
            # print(prev_config)

            # incremented_config.append(tuple(prev_config))

            self.robot.update_configuration(prev_config)
            self.configs.append(tuple(prev_config))
            if self.detect_collision(self.robot):
                if(tuple(start_config) == (0, math.radians(90))):
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
                dummy = prev_config[i]
                dummy += float(increment_by[i])

                if(forward and dummy > 6.27):

                    # dummy should be reset to 0
                    prev_config[i] = (dummy - 6.27)
                elif((not forward) and dummy < 0):

                    # dummy should be set to 6.27
                    prev_config[i] = (dummy + 6.27)
                else:
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

def angular_distance(end, start):
    # print("end is {}".format(end))
    # print("start is {}".format(start))
    # print("return is {}".format(float(end) - float(start)))
    # print("------")
    return float(end) - float(start)

def get_min_travel(end, start):

    # true is forwards
    # false is backwards

    d = angular_distance(end, start)
    test = min(d, (2 * math.pi) - d)

    if d == test:
        return(test, True)
    else:
        return(test, False)

if __name__ == "__main__":
    # angles = [0, 90]
    # angles = [270, 270]
    # robot_test = robot(angles, 100)
    # environment_test = environment(robot_test, 3)

    # starting_angles = [0, math.radians(90)]
    # end_angles = [math.radians(270), math.radians(270)]

    starting_angles = [0.17453292519943295, 6.19591884457987, 1.5707963267948966, 6.19591884457987]
    end_angles = [3.774016428646744, 1.0914423299586218, 5.4175286331656896, 0.22380400583459362]
    # starting_angles = [math.radians(270), math.radians(10), math.radians(270), math.radians(10)]
    # end_angles = [1.327213366381409, 0.9768762655660873]
    # starting_angles = [5.847508897435841, 0.6093042316591071]


    robot_test = Robot(starting_angles, end_angles, 50)
    environment_test = Environment(robot_test, 5)
    environment_test.check_motion(starting_angles, end_angles, 100)
    # environment_test.generate_valid_configuration()
    # graphics = Graphics(environment_test)
    # graphics.render()
