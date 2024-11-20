from dyersmat import *
from tuning import Tuning
import math

class EncController():
    def __init__(self, max_num, deg, num_vars):
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
        self.delta = 1
        
        # Generate new lambda for trap and corresponding mod
        self.new_lam = self.lam + 10
        self.kappa, self.p = keygen(self.rho, self.rho_, self.new_lam)
        self.mod = pgen(self.new_lam, self.rho_, self.p)
        
    def ApplyEncCalc_WithEncTrap(self,x,u):
        '''
        Application of encryption and overflow trap
        
        Args:
            x: states
            u: expected output for a control cycle. Just for comparison
            
        Returns:
            bool: if attack was detected
        '''
        
        # Encrypted Control Execution
        enc_x = enc(x, self.kappa, self.p, self.mod, self.delta)
        enc_u = mult(mult(mult(enc_x, enc_x, self.mod), enc_x, self.mod),enc_x,self.mod)

        #FDIA
        enc_u = self.attack(enc_u, attack_type="mult_invar")

        # Trap
        self.OverflowTrap(enc_u)
        
        # decrypting final u
        dec_u = dec(enc_u, self.kappa, self.p, self.delta**self.d)
        
        # printing stuff
        print(f'Expected u: {u}')
        print(f'Actual u: {dec_u}')

        if self.trap:
            print('FDIA Detected')
            return 1 # returns for counting
        else:
            print('No FDIA Detected')
            return 0

    def attack(self, enc_u, attack_type):
        '''
        FDIA function
        
        Args:
            enc_u: encrypted highest value in the algorithm (assumption: u is the largest quantity)
            attack+type: type of injected attack
        '''
        
        if attack_type is "add":
            beta = 200
            return add(enc_u, beta, self.mod)

        if attack_type is "mult_invar":
            beta = 15
            return enc_u*beta

        if attack_type is "mult_var":
            beta = int(np.ceil(np.abs(2 * math.sin(math.pi * s.t / 4))))
            return mult(enc_u, beta, self.mod)

        if attack_type is "quad":
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
        
        if enc_u % self.p > self.p_min or (enc_u % self.p) % self.kappa > self.kappa_min:
            self.trap = True
        return enc_u