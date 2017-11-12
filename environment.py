from robot import robot
from shapely.geometry import Point, LineString, Polygon

# collisions, obstacles
class environment:

    def __init__(self, r, buffer_radius):
        self.r = r
        self.buffer_radius = buffer_radius

        self.obstacles = [Point(50, 50),
                          Point(-50, -50),
                          Point(50, -50),
                          Point(-50, 50)]

    def detect_collision(self):

        # collision = False

        for obstacle in self.obstacles:

            if self.r.get_link1().intersection(obstacle.buffer(self.buffer_radius)):
                return True

            if self.r.get_link2().intersection(obstacle.buffer(self.buffer_radius)):
                return True

        return False

if __name__ == "__main__":
    robot_test = robot(0, 90, 100, 100)
    environment_test = environment(robot_test, 3)
    print(environment_test.detect_collision())
