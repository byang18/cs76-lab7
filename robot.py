import math
import random
from shapely.geometry import Point, LineString, Polygon

MAX_ANGLE = 2 * math.pi

class Robot:

    def __init__(self, start_thetas, length=50):

        self.num_angles = len(start_thetas)
        self.length = length

        self.thetas = start_thetas
        self.x_values = []
        self.y_values = []
        self.links = []

        self.update_configuration(start_thetas)

    def update_configuration(self, thetas):

        self.thetas = thetas
        self.x_values = self.build_x_values()
        self.y_values = self.build_y_values()
        self.links = self.build_links()

        return self.links

    def build_x_values(self):
        values = []
        cum_theta = 0
        prev_x = 0
        for theta in list(self.thetas):
            cum_theta += theta
            x = prev_x + (self.length * math.cos(cum_theta))
            values.append(x)
            prev_x = x

        return values

    def build_y_values(self):
        values = []
        cum_theta = 0
        prev_y = 0
        for theta in list(self.thetas):
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
            config.append(random.uniform(0, MAX_ANGLE))
        self.update_configuration(tuple(config))
        return tuple(config)

if __name__ == "__main__":
    angles = [270, 270]
    robot_test = Robot(angles, 100)
    print(robot_test.x_values)
    print(robot_test.y_values)
    print(robot_test.links)
