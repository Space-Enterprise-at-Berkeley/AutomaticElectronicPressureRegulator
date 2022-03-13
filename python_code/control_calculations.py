
"""
Computes things for tuning PID constants
All computations here are done in SI units, inputs and outputs might go through unit conversion
Note that c_v subsumes k_1

Governing equation:
p_dot = alpha * c_v - p * v_ldot

Feedforward:
c_v = p * v_ldot / alpha
"""

PSI_TO_PASCAL = 6894.76
PASCAL_TO_PSI = 0.000145038
TIME_TO_S = 1e-3 # milliseconds to s

def compute_alpha(k_3, p_h, v_l):
    alpha = k_3 * p_h / v_l
    return alpha

def compute_k3(mu = 28.97e-3, R = 8.315, T = 293):
    """
    p*v = n*R*T = (m/mu)*R*T = m*k_3
    k_3 = R*T/mu
    """
    k3 = R * T / mu
    return k3

def psi_to_pascal(psi):
    return psi * PSI_TO_PASCAL

def pascal_to_psi(pascal):
    return pascal * PASCAL_TO_PSI

def compute_feedforward_factor(v_ldot, k_3, grad):
    """
    Computes ff_factor
    theta_ff = theta_0 + (p_setpoint/p_H) * ff_factor
    """
    return (v_ldot) / (k_3 * grad)

def compute_cv_grad(grad_raw, v_h, k_2):
    """
    grad_raw is dimensionless c_v from excel sheets
    output is gradient k1*cv / motor angle (in encoder counts)
    """
    grad_s = grad_raw / TIME_TO_S
    return grad_s * (v_h / k_2)

# measured values

# piecewise intersection of k1*cv/angle graph
intersect = 800 # in encoder counts

# volume of high pressure tank
v_h = 0.0068 # in cubic meters; 6.8L

# k2 = k3
k_2 = compute_k3()
k_3 = compute_k3()

# Note that v_l in equations refer to volume of ullage (air), NOT vol of tank
flow_duration = 10 # in s
v_l = 0.0137 # 13.7 liters
v_ldot_nom = v_l / flow_duration

# gradient of k1*cv/angle graph
# vertical axis is in psi/microsec
gradient_raw = 1.45e-6 # in fucked up units
gradient = compute_cv_grad(gradient_raw, v_h, k_2)

alpha = compute_alpha(k_3, 750, v_l)

ff_factor = compute_feedforward_factor(v_ldot_nom, k_3, gradient)
print(ff_factor)