'''
This function runs iterations over the encryption trap in order to test its accuracy with respect to the expected probability of detection
'''
import numpy as np

def ApplyDetector(trap_obj,var_dict,u,num_tests,beta_range,test_multiple = False):
    
    # Test one Beta
    if test_multiple is False:
        return trap_obj.ApplyEncCalc_WithEncTrap(var_dict,u,beta=beta_range)
    
    # Test lots of Betas
    else:
        detect_array = np.zeros((beta_range[1],2))
        
        # Loop through many betas for certain numbers of attacks
        for beta in range(beta_range[0],beta_range[1]):
            num_FDIA_detected = 0
            for _ in range(num_tests):
                    num_FDIA_detected += trap_obj.ApplyEncCalc_WithEncTrap(var_dict,u, beta)
            detect_array[beta, :] = [beta,num_FDIA_detected]

        print(f'# detected FDIA: \n {detect_array}')

