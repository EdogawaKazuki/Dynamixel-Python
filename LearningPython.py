from math import *
t = 0

# define your total time for part one, in seconds
duration = 2

# write down the x, y, z equations in terms of t
x = -sin(t)
y = 5
z = 15 + 30 * t * t - 20 * t * t * t

# if you want to control the light
if 0.25 <= t <= 0.5 or 0.75 <= t <= 2:
    light = [1, 0, 0]
else:
    light = [0, 0, 1]