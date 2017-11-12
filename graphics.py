from robot import robot
from environment import environment
from cs1lib import *

WINDOW_WIDTH = 200
WINDOW_HEIGHT = 200

class graphics:

    def __init__(self, env):
        self.env = env
        self.r = env.r

        # robot
        self.center_x = 200
        self.center_y = 200

    def draw_configuration(self):

        if self.env.detect_collision():
            set_fill_color(1, 0, 0, .5)
            draw_rectangle(0, 0, 400, 400)

        set_fill_color(1, 1, 1)

        set_stroke_color(0, 1, 0)
        x1 = self.r.getX1()
        y1 = self.r.getY1()

        x2 = self.r.getX2()
        y2 = self.r.getY2()

        draw_line(self.center_x,
                  self.center_y,
                  self.center_x + x1,
                  self.center_y - y1)

        draw_line(self.center_x + x1,
                  self.center_y - y1,
                  self.center_x + x2,
                  self.center_y - y2)

        set_stroke_color(0, 0, 0)

        for obstacle in self.env.obstacles:

            draw_circle(self.center_x + obstacle.x,
                        self.center_y + obstacle.y,
                        self.env.buffer_radius)

        # draw_circle(self.center_x + 50,
        #             self.center_y - 50,
        #             3)
        #
        # draw_circle(self.center_x + 50,
        #             self.center_y + 50,
        #             3)
        #
        # draw_circle(self.center_x - 50,
        #             self.center_y + 50,
        #             3)
        #
        # draw_circle(self.center_x - 50,
        #             self.center_y - 50,
        #             3)

    def main(self):
        clear()
        self.draw_configuration()

if __name__ == "__main__":
    # robot_test = robot(0, 90, 100, 100)
    robot_test = robot(270, 270, 100, 100)
    environment_test = environment(robot_test, 5)
    render = graphics(environment_test)
    start_graphics(render.main)
