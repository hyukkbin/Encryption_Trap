from dyersmat import *
import math

def Tuning(X, d, zeta):

    '''
    Tune encryption parameters based on the Enc MMRAC paper
    
    Args:
        X: largest plaintext value, including encryption
        d: highest degree of controller polynomial
        zeta: number of variables in control polynomial
        
    Returns:
        lam: lambda value for Dyers
        rho: rho value for Dyers
        rho_: rho prime value for Dyers
        p_min: minimum value that p can be
        kappa_min: minimum value that kappa can be
    '''
    
    # Step 2: determine b and M (M is highest value of bit length b)
    b = math.ceil(math.log2(X+1))
    M = (2**b) - 1
    
    # Uncomment this for more restrictive detection
    M=X
    
    # Step 3: determine kappa_min
    kappa_min = ((zeta+1)**d)*(M**d)

    # Step 6: finding nu and setting rho and rho prime (rho_)
    nu = math.log2(kappa_min+1) 
    rho = 32
    rho_ = math.ceil(nu)+rho
    
    # Step 4: determine p_min
    # kappa, _ = keygen(rho, rho_, bit_length=None) # Can use kappa for less restrictive detection
    p_min = ((zeta+0.5)**d)*(M+(kappa_min**2))**d # Can modify 0.5 to adjust restriction on detection
    
    # Step 5: determine lambda_min
    lam = int(math.ceil(math.log2(p_min+1)))

    return lam, rho, rho_, p_min, kappa_min