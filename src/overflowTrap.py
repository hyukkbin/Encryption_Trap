from dyersmat import *
from tuning import Tuning
import math

class EncController():
    def __init__(self, max_num, deg, num_vars, delta):
        '''
        This class encrypts any controller and adds an overflow trap
        
        Args:
            max_num: estimated maximum quantity within the controller's operation. This may take some tuning as testing is performed
            deg: degree of depth of controller
            num_vars: number of variables in controller
        '''
        
        # Step 1: predetermining max plaintext value X including encoding factor
        self.X = max_num # Largest expected value in the control algorithm
        self.d = deg # Degree of depth in the control algorithm
        self.zeta = num_vars # number of variables in control algorithm final polynomial
        self.trap = False # True if there has been an attack
        
        # Initialize encryption parameters
        # Generate minimum p and k values to allow the algorithm to run without overflow. This does not mean these are the exact max values that the algorithm will use
        self.lam, self.rho, self.rho_, self.p_min, self.kappa_min = Tuning(self.X, self.d, self.zeta)
        self.delta = delta
        
        # Generate new lambda for trap and corresponding mod
        self.new_lam = self.lam + 10
        self.kappa, self.p = keygen(self.rho, self.rho_, self.new_lam)
        self.mod = pgen(self.new_lam, self.rho_, self.p)
        
    def ApplyEncCalc_WithEncTrap(self,state_dict,u,beta):
        '''
        Application of encryption and overflow trap
        
        Args:
            x: states
            u: expected output for a control cycle. Just for comparison
            beta: attack value if attack
            
        Returns:
            bool: if attack was detected
        '''
        
        # Encrypted Control Execution... change this to the controller you need
        enc_u = self.EncControl(state_dict)
        
        #FDIA
        enc_u = self.attack(enc_u, beta, attack_type="mult_invar")

        # Trap
        self.OverflowTrap(enc_u)
        
        # decrypting final u
        dec_u = mat_dec(enc_u, self.kappa, self.p, self.delta**2)
        
        # printing stuff
        print(f'Expected u: {u}')
        print(f'Actual u: {dec_u}')

        if self.trap:
            print('FDIA Detected')
            return 1 # returns for counting
        else:
            print('No FDIA Detected')
            return 0 # returns for counting

    def EncControl(self,var_dict):
        '''
        Perform the encrypted control calculations
        
        Args: 
            Variable dictionary of needed values
            
        Returns:
            Encrypted commands
        '''
        
        # Encrypt each part of var dictionary
        enc_var_dict = {}
        for item in var_dict:
            
            # This is specific to this test case where the w_r needs to have a deeper encoding factor to be added to the rest of the eqn
            if item == 'k_y' or item == 'k_theta':
                unique_delta = 1
            elif item =='w_r':
                unique_delta = self.delta**2
            else: 
                unique_delta=self.delta
            
            # Encrypt each needed variable
            enc_var_dict[item] = enc(var_dict[item], self.kappa, self.p, self.mod, unique_delta)
        
        # Calculate control
        enc_u = np.array([
            [add(
                mult(
                    enc_var_dict["v_r"],
                    enc_var_dict["cos_theta_e"],
                self.mod), 
                mult(
                    enc_var_dict["k_x"],
                    enc_var_dict["x_e"],
                self.mod), 
            self.mod)],
            [add(
                enc_var_dict["w_r"],
                mult(
                    enc_var_dict["v_r"],
                    add(
                        mult(
                            enc_var_dict["k_y"],
                            enc_var_dict["y_e"], 
                        self.mod),
                        mult(
                            enc_var_dict["k_theta"],
                            enc_var_dict["sin_theta_e"],
                        self.mod),
                    self.mod),
                self.mod),
            self.mod)]])
        
        return enc_u
        
        # Dummy d=4 test
        # enc_u = mult(mult(mult(enc_var_dict['x'], enc_var_dict['x'], self.mod), enc_var_dict['x'], self.mod), enc_var_dict['x'], self.mod)
    
    def attack(self, enc_u, beta, attack_type):
        '''
        FDIA function
        
        Args:
            enc_u: encrypted highest value in the algorithm (assumption: u is the largest quantity)
            attack+type: type of injected attack
            
        Returns:
            enc_u attacked
        '''
        
        if attack_type == "add":
            return add(enc_u, beta, self.mod)

        if attack_type == "mult_invar":
            return enc_u*beta

        if attack_type == "mult_var":
            beta = int(np.ceil(np.abs(2 * math.sin(math.pi * s.t / 4))))
            return mult(enc_u, beta, self.mod)

        if attack_type == "quad":
            return add(mult(enc_u, enc_u, self.mod), enc_u, self.mod)
        
    def OverflowTrap(self, enc_u):
        '''
        Determine if there has been an attack with the overflow trap
        
        Args:
            enc_u: encrypted highest value in the algorithm (assumption: u is the largest quantity)
        '''
        
        # Calculate probability of detection
        prob_detect = (self.p-self.p_min) / self.p
        print(f'Prob detection: {round(prob_detect*100,2)}%')
        
        # Encryption trap logic
        if any(enc_u % self.p > self.p_min) or any((enc_u % self.p) % self.kappa > self.kappa_min):
            self.trap = True
        else: self.trap = False
        return enc_u