import matplotlib.pyplot as plt
import numpy as np
import math

H = 0.4
L1 = 5
L2 = 4
L3 = 3


class Tractor:
    def __init__(self, dt, psi = 0):
        self.psi   = psi
        self.dt    = dt
        self.front = [      0 , 0]
        self.rear  = [   - L1 , 0]
        self.hitch = [-H - L1 , 0]
        

    def step(self, steering_angle, v):
        ds = v * self.dt
        d_psi = math.asin(ds * math.sin(math.pi - steering_angle) / L1);

        self.psi += d_psi

        if self.psi > math.pi:
            self.psi -= 2 * math.pi
        elif self.psi < -math.pi:
            self.psi += 2 * math.pi

        new_front = [self.front[0] + ds * math.cos(steering_angle),
                     self.front[1] + ds * math.sin(steering_angle)]
        new_rear  = [new_front[0] - L1 * math.cos(self.psi), 
                     new_front[1] - L1 * math.sin(self.psi)]
        new_hitch = [new_front[0] - (L1 + H) * math.cos(self.psi), 
                     new_front[1] - (L1 + H) * math.sin(self.psi)]
        
        self.front = new_front
        self.rear = new_rear
        self.hitch = new_hitch

    def print(self):
        print(self.front)
        print(self.rear)
        print(self.hitch)

class Sprayer:
    def __init__(self, hitch, psi = 0):
        self.psi   = psi
        self.hitch = hitch
        self.kink  = [self.hitch[0] + L2 * math.cos(self.psi),
                      self.hitch[1] + L2 * math.sin(self.psi)]
        self.axle  = [0,0]

    def step(self, beta, hitch):
        new_hitch = hitch
        new_kink  = []
        new_axle  = []

        diag = math.sqrt(L2**2 + L3**2 - 2 * L2 * L3 * math.cos(math.pi - beta))
        tau1 =  math.asin(L3 * math.sin(math.pi - beta) / diag)
        tau2 =  math.asin(L2 * math.sin(math.pi - beta) / diag)

        if L3 == 0: # wheelsteer
            A = np.array([[self.kink[0], 1], [self.kink[0] + math.cos(self.psi + beta), 1]])
            b = np.array([self.kink[1], self.kink[1] + math.sin(self.psi + beta)])
        else:
            A = np.array([[self.kink[0], 1], [self.axle[0], 1]])
            b = np.array([self.kink[1], self.axle[1]])

        result = np.linalg.solve(A,b)
            
        m = result[0]
        c = result[1]
       

    def print(self):
        print(self.hitch)
        print(self.kink)
        print(self.axle)

if __name__ == "__main__":
    t = Tractor(0.01)
    s = Sprayer(t.hitch)
    t.print()
    s.print()
    t.step(0 * math.pi / 180, 3)
    s.step(0 * math.pi / 180, t.hitch)
    t.print()
    s.print()
    pass
