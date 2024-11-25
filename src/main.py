'''
Run any control algorithm with encryption and an overflow trap

Limitations:
    - Requires tuning
    - Requires knowledge of max value in the entire algorithm's runs and if that max value isn't even close, there is more range for attack
    - Requires algorithm that can be encrypted by Dyer's without overflow (not too much depth)
'''

from overflowTrap import EncController
from apply_detector import ApplyDetector
import math
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
             
def Run():
    
    # Parameters to set:
    delta = 10000
    data = 'src/experimentdata.csv'
    max_num = MaxNum(delta, data) # find the max number in the nominal unattacked case
    degree = 3
    num_vars = 1
    
    test_multiple = False
    num_tests_per_data = 100
    beta_range = 1 #[1,50]
    scenario = "scale" # for data testing
    
    # Tune these for the trap!
    trap_obj = EncController(max_num=max_num, deg=degree, num_vars=num_vars, delta=1/delta)

    # Unpack data and allocate it to dictionary of variables
    data = pd.read_csv(data)
    if scenario == "reflection":
        idx_jump = 10
    elif scenario == "scale":
        idx_jump = 20
    else: idx_jump = 0

    # This loop represents the collection of data from sensors and feeding it into the controller and applying the encryption trap
    num_detections = 0
    t_vec = []
    x_vec = []
    y_vec = []
    theta_vec = []
    x_vec_nominal = []
    y_vec_nominal = []
    theta_vec_nominal = []
    for idx, row in data.iloc[2:].iterrows():
        row = row.tolist()

        var_dict = {
            "x_e": float(row[3+idx_jump]),
            "y_e": float(row[4+idx_jump]),
            "theta_e": float(row[5+idx_jump]),
            "cos_theta_e": math.cos(float(row[5+idx_jump])),
            "sin_theta_e": math.sin(float(row[5+idx_jump])),
            "v_r": float(row[6+idx_jump]),
            "w_r": float(row[7+idx_jump]),
            "k_x": 100,
            "k_y": 2000,
            "k_theta": 10
        }

        # Nominal/Expected u value
        u = np.array([[var_dict["v_r"]*math.cos(var_dict["theta_e"]) + var_dict["k_x"]*var_dict["x_e"]],
            [var_dict["w_r"] + var_dict["v_r"]*(var_dict["k_y"]*var_dict["y_e"] + var_dict["k_theta"]*math.sin(var_dict["theta_e"]))]])
        
        # Test detector
        num_detections += ApplyDetector(trap_obj,var_dict,u,num_tests=num_tests_per_data, beta_range=beta_range,test_multiple = test_multiple)
        
        # Plotting data
        t_vec.append(idx-2)
        x_vec_nominal.append(float(row[0]))
        y_vec_nominal.append(float(row[1]))
        theta_vec_nominal.append(float(row[2]))
        x_vec.append(float(row[0+idx_jump]))
        y_vec.append(float(row[1+idx_jump]))
        theta_vec.append(float(row[2+idx_jump]))
    
    print(f'of {idx-2} tests, {num_detections} attacks detected') 
    
    if scenario is not None:
        plt.subplot(3,1,1)
        plt.plot(t_vec,x_vec)
        plt.plot(t_vec,x_vec_nominal)
        
        plt.subplot(3,1,2)
        plt.plot(t_vec,y_vec)
        plt.plot(t_vec,y_vec_nominal)

        plt.subplot(3,1,3)
        plt.plot(t_vec,theta_vec)
        plt.plot(t_vec,theta_vec_nominal)
        
    else:
        plt.subplot(3,1,1)
        plt.plot(t_vec,x_vec_nominal)
        
        plt.subplot(3,1,2)
        plt.plot(t_vec,y_vec_nominal)

        plt.subplot(3,1,3)
        plt.plot(t_vec,theta_vec_nominal)

    plt.show()                
                
# Determine the highest quantity in the control loop
def MaxNum(delta, data):
    
    # Unpack data and allocate it to dictionary of variables
    data = pd.read_csv(data)

    curr_max = float('-inf')
    for idx, row in data.iloc[2:].iterrows():
        row = row.tolist()

        var_dict = {
            "x_e": float(row[3]),
            "y_e": float(row[4]),
            "theta_e": float(row[5]),
            "cos_theta_e": math.cos(float(row[5])),
            "sin_theta_e": math.sin(float(row[5])),
            "v_r": float(row[6]),
            "w_r": float(row[7]),
            "k_x": 100,
            "k_y": 2000,
            "k_theta": 10
        }

        # Turtlebot drive test control
        u = np.array([[(var_dict["v_r"]*delta)*(math.cos(var_dict["theta_e"]*delta)) + (var_dict["k_x"]*delta)*(var_dict["x_e"]*delta)],
            [(var_dict["w_r"]*delta**2) + (var_dict["v_r"]*delta)*(var_dict["k_y"]*(var_dict["y_e"]*delta) + var_dict["k_theta"]*(math.sin(var_dict["theta_e"]*delta)))]])
        
        # Save max value if one is found
        for i in u:
            if abs(i) > curr_max:
                curr_max = i[0]
                
    return curr_max

if __name__ == "__main__":
    Run()

# State
# var_dict = {
# "x":100} # Dummy d=4 test

# Control algorithm nominal
# u = x**deg # Dummy d=4 test