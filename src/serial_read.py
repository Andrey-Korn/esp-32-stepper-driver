# print out angles sent to /dev/ttyUSB0 as stdin

from machine import Timer
import sys, time, select
serial_poll = select.poll()
serial_poll.register(sys.stdin, select.POLLIN)



def serial_callback(t):
    stream = serial_poll.poll(1)
    if stream == []:
        # print('stdin empty!')
        pass
    else:
        data = sys.stdin.readline()
        data = data.strip('\n')
        data = data.split(',')
        data[0] = round(float(data[0]), 2)
        data[1] = round(float(data[1]), 2)
        print(f'{data[0]} {data[1]}')


tim0 = Timer(0)
tim0.init(period=50, callback=serial_callback)

while(1):
    pass
