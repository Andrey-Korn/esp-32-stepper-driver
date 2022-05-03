import os
from numpy import angle
import serial
os.environ['PYGAME_HIDE_SUPPORT_PROMPT'] = "hide"
import pygame
import time

sample_delay = 0.002

class ps4_controller(object):
    controller = None
    button_data = None
    axis_data = None
    angle_packet = ''

    def __init__(self):
        pygame.init()
        pygame.joystick.init()
        self.controller = pygame.joystick.Joystick(0)
        self.controller.init()

        # start serial connection to ESP-32
        self.esp32 = serial.Serial(port='/dev/ttyUSB0', baudrate=115200, timeout=0.1)
    
    def serial_write(self, str_to_write):
        self.esp32.write(bytes(str_to_write, 'utf-8'))
        pass

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
                            self.axis_data[1] = -round(event.value, 2)

                # save angle with this format 
                self.angle_packet = f'<{self.axis_data[0]},{self.axis_data[1]}>'
                print(self.angle_packet)
                self.serial_write(self.angle_packet)
                
                time.sleep(sample_delay)

            except KeyboardInterrupt:
                quit()


ps4 = ps4_controller()
ps4.listen()
