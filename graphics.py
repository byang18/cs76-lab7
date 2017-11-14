# from robot import robot
# from environment_driver import environment
# from PRM import PRM
import time
from cs1lib import *

WINDOW_WIDTH = 200
WINDOW_HEIGHT = 200
iteration = 0

class Graphics:

    def __init__(self, env):
        self.env = env
        self.robot = env.robot
        print(self.robot.x_values)

        self.configs = env.configs
        print('PRINTING CONFIGS')
        print(len(self.configs))
        for config in self.configs:
            print(config)

    def update_robot(self, config):
        self.robot.update_configuration(config)

    def draw_configuration(self):

        if self.env.detect_collision(self.robot):
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

        for obstacle in self.env.obstacles:

            draw_circle(WINDOW_WIDTH + obstacle.x,
                        WINDOW_HEIGHT + obstacle.y,
                        self.env.buffer_radius)

    def animate(self):

        global iteration

        clear()
        config = self.configs[len(self.configs) - 1]
        if(iteration < len(self.configs)):
            config = list(self.configs[iteration])
            print('rendering: {}'.format(config))
        self.update_robot(config)
        self.draw_configuration()
        iteration += 1

    def main(self):
        clear()
        self.draw_configuration()

    def render(self):
        start_graphics(self.animate)
        # start_graphics(self.main)

    # def render(self):
    #     start_graphics(self.main())

# if __name__ == "__main__":
#     angles = [0, 90]
#     # angles = [315, 270]
#     angles = [270, 270]
#     robot_test = robot(angles, 100)
#     environment_test = environment(robot_test, 5)
#     graphics(environment_test).render()
#     render_graphics = graphics(environment_test)
#     start_graphics(render_graphics.main)
