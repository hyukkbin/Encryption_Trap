'''
This function runs iterations over the encryption trap in order to test its accuracy with respect to the expected probability of detection
'''

def TestFDIA(trap_obj,x,u,num_tests):
    num_FDIA_detected = 0
    for _ in range(num_tests):
        num_FDIA_detected += trap_obj.ApplyEncCalc_WithEncTrap(x,u)

    print(f'# detected FDIA: {num_FDIA_detected}')