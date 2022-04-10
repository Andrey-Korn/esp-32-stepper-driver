import os
import serial
os.environ['PYGAME_HIDE_SUPPORT_PROMPT'] = "hide"
import pygame
import time

sample_delay = 0.05

class ps4_controller(object):
    controller = None
    button_data = None
    axis_data = None

    def __init__(self):
        pygame.init()
        pygame.joystick.init()
        self.controller = pygame.joystick.Joystick(0)
        self.controller.init()

    def listen(self):

        # init axis data
        if not self.axis_data:
            self.axis_data = [0, 0]

        while True:
            try:
                for event in pygame.event.get():
                    if event.type == pygame.JOYAXISMOTION:
                        # value = event.value 
                        if event.axis == 3:
                            self.axis_data[0] = round(event.value, 2)
                        if event.axis == 4:
                            self.axis_data[1] = round(event.value, 2)

                time.sleep(sample_delay)
                print(f'{self.axis_data[0]},{self.axis_data[1]}')

            except KeyboardInterrupt:
                quit()


ps4 = ps4_controller()
ps4.listen()
