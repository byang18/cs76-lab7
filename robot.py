import math
from shapely.geometry import Point, LineString, Polygon

class robot:

    def __init__(self, theta1, theta2, length1, length2, theta3=0, theta4=0):
        self.theta1 = math.radians(theta1)
        self.theta2 = math.radians(theta2)
        self.theta3 = math.radians(theta3)
        self.theta4 = math.radians(theta4)

        self.length1 = length1
        self.length2 = length2

    def getX1(self):
        return self.length1 * math.cos(self.theta1)

    def getY1(self):
        return self.length1 * math.sin(self.theta1)

    def getX2(self):
        return (self.getX1() +
                self.length2 * math.cos(self.theta1 + self.theta2))

    def getY2(self):
        return (self.getY1() +
                self.length2 * math.sin(self.theta1 + self.theta2))

    def get_link1(self):
        link = LineString([(0, 0), (self.getX1(), self.getY1())])
        return link

    def get_link2(self):
        link = LineString([(self.getX1(), self.getY1()), (self.getX2(), self.getY2())])
        return link

    # 3R and 4R -- later?
    def getX3(self):
        return (self.getX2() +
                self.length2 * math.cos(self.theta1 + self.theta2 + self.theta3))

    def getY3(self):
        return (self.getY2() +
                self.length2 * math.sin(self.theta1 + self.theta2 + self.theta3))

    def getX4(self):
        return (self.getX3() +
                self.length2 * math.cos(self.theta1 + self.theta2 + self.theta3 + self.theta4))

    def getY4(self):
        return (self.getY3() +
                self.length2 * math.sin(self.theta1 + self.theta2 + self.theta3 + self.theta4))

if __name__ == "__main__":
    robot_test = robot(0, 90, 100, 100)
    # robot_test = robot(270, 270, 100, 100)
    print("x1: {}".format(robot_test.getX1()))
    print("y1: {}".format(robot_test.getY1()))
    print("x2: {}".format(robot_test.getX2()))
    print("y2: {}".format(robot_test.getY2()))
