'''
Run any control algorithm with encryption and an overflow trap

Limitations:
    - Requires tuning
    - Requires knowledge of max value in the entire algorithm's runs and if that max value isn't even close, there is more range for attack
    - Requires algorithm that can be encrypted by Dyer's without overflow
'''

from overflowTrap import EncController
from test_trap import TestFDIA
import math
import numpy as np

# Choose Trap tuning values here!
trap_obj = EncController(max_num=1000, deg=3, num_vars=3, delta = .001)

# State
# x=100 # Dummy d=4 test

var_dict = {
    "v_r": 1,
    "theta_e": 1,
    "cos_theta_e": math.cos(1),
    "sin_theta_e": math.sin(1),
    "k_x": 1,
    "x_e": 1,
    "w_r": 1,
    "v_r": 1,
    "k_y": 1,
    "y_e": 1,
    "k_theta": 1
}

# Control algorithm nominal
# u = x**deg # Dummy d=4 test

# turtlebot drive
u = np.array([[var_dict["v_r"]*math.cos(var_dict["theta_e"]) + var_dict["k_x"]*var_dict["x_e"]],
     [var_dict["w_r"] + var_dict["v_r"]*(var_dict["k_y"]*var_dict["y_e"] + var_dict["k_theta"]*math.sin(var_dict["theta_e"]))]])

# Test detector
TestFDIA(trap_obj,var_dict,u,num_tests=100, beta_range=[1,50])
