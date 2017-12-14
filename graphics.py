import math
from robot import Robot
from prm import PRM
from cs1lib import *

WINDOW_WIDTH = 200
WINDOW_HEIGHT = 200
BUFFER_RADIUS = 3
PI = math.pi

# START = (3.141592653589793, 4.71238898038469, 1.5707963267948966, 4.71238898038469)
# START = (3.238762669143777, 0.780248312425674, 1.6863333594653087, 4.590840931466817)
# START = (4.485570585480902, 0.6805452727332207, 4.695385895866597, 0.43120575681221857)
# START = (1.168125383616999, 0.4721068358472472, 4.5951156505179505, 0.013159449777925387)
START = (0, 1.5707963267948966, 4.71238898038469, 1.5707963267948966)

class Graphics:

    def __init__(self, prm):
        self.prm = prm
        self.robot = prm.robot

    def draw_configuration(self):

        if self.prm.detect_collision(self.robot):
            set_fill_color(1, 0, 0, .5)
            draw_rectangle(0, 0, 400, 400)

        set_fill_color(1, 1, 1)

        set_stroke_color(0, 1, 0)

        start_x = WINDOW_WIDTH
        start_y = WINDOW_HEIGHT

        for i in range(len(self.robot.x_values)):

            end_x = WINDOW_WIDTH + self.robot.x_values[i]
            end_y = WINDOW_HEIGHT - self.robot.y_values[i]

            draw_line(start_x, start_y, end_x, end_y)

            start_x = end_x
            start_y = end_y

        set_stroke_color(0, 0, 0)

        for obstacle in self.prm.obstacles:

            draw_circle(WINDOW_WIDTH + obstacle.x,
                        WINDOW_HEIGHT + obstacle.y,
                        BUFFER_RADIUS)

    def main(self):
        clear()
        self.draw_configuration()

if __name__ == "__main__":
    r = Robot(START)
    prm = PRM(r)
    graphics = Graphics(prm)
    start_graphics(graphics.main)
