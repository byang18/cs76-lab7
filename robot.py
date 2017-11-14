import math
import random
from shapely.geometry import Point, LineString, Polygon

MAX_ANGLE = 360

class Robot:

    # def __init__(self, theta1, theta2, length1, length2, theta3=0, theta4=0):
    def __init__(self, start_thetas, goal_state=[0, 0], length=100):
        # self.theta1 = math.radians(theta1)
        # self.theta2 = math.radians(theta2)
        # self.theta3 = math.radians(theta3)
        # self.theta4 = math.radians(theta4)

        self.num_angles = len(start_thetas)
        self.goal_state = goal_state
        # self.start_angles = start_angles
        self.length = length
        self.thetas = []
        self.x_values = []
        self.y_values = []
        self.links = []

        self.update_configuration(start_angles)

    def update_configuration(self, config):

        # be careful! angles should be the same length as before
        self.thetas = []
        for angle in config:
            self.thetas.append(math.radians(angle))

        self.x_values = self.build_x_values()
        self.y_values = self.build_y_values()
        self.links = self.build_links()

    def build_x_values(self):
        values = []
        cum_theta = 0
        prev_x = 0
        for theta in self.thetas:
            cum_theta += theta
            x = prev_x + (self.length * math.cos(cum_theta))
            values.append(x)
            prev_x = x

        return values

    def build_y_values(self):
        values = []
        cum_theta = 0
        prev_y = 0
        for theta in self.thetas:
            cum_theta += theta
            y = prev_y + (self.length * math.sin(cum_theta))
            values.append(y)
            prev_y = y

        return values

    def build_links(self):

        links = []
        start_point = (0, 0)

        for i in range(0, len(self.x_values)):
            end_point = (self.x_values[i], self.y_values[i])
            links.append(LineString([(start_point), (end_point)]))
            start_point = end_point

        return links

    def generate_random_configuration(self):
        config = []

        for i in range(0, self.num_angles):
            config.append(random.randrange(0, 360))
        self.update_configuration(config)
        return config

    # def getX1(self):
    #     return self.length * math.cos(self.thetas[0])
    #
    # def getY1(self):
    #     return self.length * math.sin(self.thetas[0])
    #
    # def getX2(self):
    #     return (self.getX1() +
    #             self.length * math.cos(self.thetas[0] + self.thetas[1]))
    #
    # def getY2(self):
    #     return (self.getY1() +
    #             self.length * math.sin(self.thetas[0] + self.thetas[1]))

    # def get_link1(self):
    #     link = LineString([(0, 0), (self.getX1(), self.getY1())])
    #     return link
    #
    # def get_link2(self):
    #     link = LineString([(self.getX1(), self.getY1()), (self.getX2(), self.getY2())])
    #     return link

    # 3R and 4R -- later?
    # def getX3(self):
    #     return (self.getX2() +
    #             self.length * math.cos(self.theta1 + self.theta2 + self.theta3))
    #
    # def getY3(self):
    #     return (self.getY2() +
    #             self.length * math.sin(self.theta1 + self.theta2 + self.theta3))
    #
    # def getX4(self):
    #     return (self.getX3() +
    #             self.length * math.cos(self.theta1 + self.theta2 + self.theta3 + self.theta4))
    #
    # def getY4(self):
    #     return (self.getY3() +
    #             self.length * math.sin(self.theta1 + self.theta2 + self.theta3 + self.theta4))

if __name__ == "__main__":
    # angles = [0, 45]
    angles = [270, 270]
    robot_test = Robot(angles, 100)
    print(robot_test.x_values)
    print(robot_test.y_values)
    print(robot_test.links)
    # print("y1: {}".format(robot_test.getY1()))
    # print("x2: {}".format(robot_test.getX2()))
    # print("y2: {}".format(robot_test.getY2()))
