from control import *
import matplotlib.pyplot as plt
import sys

s = tf('s')

# In this script, the Load Frequency Control system of a single
# area is defined by its Transfer Function. The system converges

# Parameters
Kps, Tps = 1, 20
Kg, Tg = 1, 0.8
Kt, Tt = 1, 0.3
R = 2.4

# Transfer function of Generator
generator = Kps / (1 + Tps*s)

# Transfer functions of Governor and Turbine
governor = Kg / (1 + Tg*s)
turbine = Kt / (1 + Tt*s)

# Actuator as a combination Governor and Turbine
actuator = series(governor, turbine)

# Feedback loop
droop = -1/R
Kp = -1
Ki = -0.8
Kd = -0.1
pid = Kp + Ki/s + Kd*s
controller = parallel(droop, pid)

# Interconnections
sys1 = -series(controller, actuator)
single_area = feedback(generator, sys1)

response = step_response(single_area, T = 500)

plt.plot(response.time, response.outputs)

plt.show()