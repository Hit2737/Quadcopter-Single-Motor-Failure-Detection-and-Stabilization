import numpy as np

def R_a2_theta(theta):
    return np.array([
        [np.cos(theta), 0, np.sin(theta)],
        [0, 1, 0],
        [-np.sin(theta), 0, np.cos(theta)]
    ])

def R_a1_phi(phi):
    return np.array([
        [1, 0, 0],
        [0, np.cos(phi), -np.sin(phi)],
        [0, np.sin(phi), np.cos(phi)]
    ])

def R_a3_psi(psi):
    return np.array([
        [np.cos(psi), -np.sin(psi), 0],
        [np.sin(psi), np.cos(psi), 0],
        [0, 0, 1]
    ])

def skew_symmetric(vector):
    return np.array([
        [0, -vector[2], vector[1]],
        [vector[2], 0, -vector[0]],
        [-vector[1], vector[0], 0]
    ])

class PIDController:
    def __init__(self, KP, KD, KI, size = 3, integral_limit = None):
        self.KP = KP
        self.KD = KD
        self.KI = KI

        # Initialize Errors
        self.integral = np.zeros(size)
        self.integral_limit = integral_limit

    def update(self, error, derivative_error, dt):
        # Update integral term
        self.integral += error * dt
        
        if self.integral_limit:
            self.integral = np.clip(self.integral, -self.integral_limit, self.integral_limit)

        # PID output
        output = self.KP * error + self.KI * self.integral + self.KD * derivative_error

        return output