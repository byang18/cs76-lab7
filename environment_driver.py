import time
import math
from shapely.geometry import Point, LineString, Polygon
from robot import Robot
from graphics import Graphics
from cs1lib import *
# from PRM import PRM

PI = math.pi
OBSTACLES = [Point(0, 50),
             Point(0, -50)]

START = (5.267611642257664, 0.9573728460702193, 4.74136323376864, 3.171974378676522)
GOAL = (0, 1.5707963267948966, 4.71238898038469, 1.5707963267948966)


class Environment:

    def __init__(self, robot, buffer_radius=5):
        self.robot = robot
        self.buffer_radius = buffer_radius

        self.configs = []

        self.obstacles = OBSTACLES

    def detect_collision(self, robot):

        for obstacle in self.obstacles:
            for link in robot.links:
                if link.intersection(obstacle.buffer(self.buffer_radius)):
                    return True

        return False

    def generate_valid_configuration(self):
        config = self.robot.generate_random_configuration()
        while self.detect_collision(self.robot):
            config = self.robot.generate_random_configuration()
        return config

    def check_motion(self, start_config, goal_config, timestep):

        increment_by = []
        sum_distances = 0

        for i in range(len(start_config)):

            difference, forward = get_min_travel(goal_config[i], start_config[i])

            if forward:
                step = difference/timestep

            else:
                step = -difference/timestep

            increment_by.append(step)
            sum_distances += abs(difference)

        prev_config = list(start_config)
        for t in range(timestep):

            self.robot.update_configuration(prev_config)
            self.configs.append(tuple(prev_config))
            if self.detect_collision(self.robot):
                if(tuple(start_config) == (0, math.radians(90))):
                    return (True, -1)

            self.configs.append(tuple(prev_config))

            # i represents the angle
            for i in range(len(start_config)):
                dummy = prev_config[i]
                dummy += float(increment_by[i])

                if(forward and dummy > 6.27):
                    prev_config[i] = (dummy - 6.27)
                elif((not forward) and dummy < 0):
                    prev_config[i] = (dummy + 6.27)
                else:
                    prev_config[i] += float(increment_by[i])


        return (False, sum_distances)


def angular_distance(end, start):
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


    starting_angles = START
    end_angles = GOAL

    robot_test = Robot(starting_angles)
    environment_test = Environment(robot_test, 5)
    print(environment_test.check_motion(starting_angles, end_angles, 100))

    graphics = Graphics(environment_test)
    graphics.render()
