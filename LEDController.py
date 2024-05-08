import time

from pyfirmata import ArduinoMega, util
board = ArduinoMega('COM8')

r = board.get_pin("d:8:p")
g = board.get_pin("d:9:p")
b = board.get_pin("d:10:p")

def set_color(color):
    r.write(color[0])
    g.write(color[1])
    b.write(color[2])

i = 0
step = 1
while True:
    if i >= 255:
        step = -1
    if i <= 1:
        step = 1
    i += step
    set_color((1, 1, 0))
    time.sleep(0.001)
