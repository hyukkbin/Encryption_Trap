'''
Run any control algorithm with encryption and an overflow trap
'''

from overflowTrap import EncController
from test_trap import TestFDIA

deg=4
trap_obj = EncController(max_num=100000000, deg=deg, num_vars=1)

# State
x=100

# Control algorithm
u = x**deg

# Test detector
TestFDIA(trap_obj,x,u, num_tests=100)
