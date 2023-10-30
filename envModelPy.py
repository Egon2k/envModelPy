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

        # cycle with hitch_new as center (hitchX, hitchY) and radius diagSprayer
        # formula: (x - x0)^2 + (y - y0)^2 = r^2
        x = [0,0]
        x[0] = (self.hitch[0] + math.sqrt(- c**2 - 2 * c * self.hitch[0] * m + 2 * c * self.hitch[1] - self.hitch[0]**2 * m**2 + 2 * self.hitch[0] * self.hitch[1] * m - self.hitch[1]**2 + m**2 * diag**2 + diag**2) - c * m + self.hitch[1] * m)/(m**2 + 1);
        x[1] = (self.hitch[0] - math.sqrt(- c**2 - 2 * c * self.hitch[0] * m + 2 * c * self.hitch[1] - self.hitch[0]**2 * m**2 + 2 * self.hitch[0] * self.hitch[1] * m - self.hitch[1]**2 + m**2 * diag**2 + diag**2) - c * m + self.hitch[1] * m)/(m**2 + 1);

        y = [0,0]
        y[0] = (c + self.hitch[0] * m + self.hitch[1] * m**2 + m * math.sqrt(- c**2 - 2 * c * self.hitch[0] * m + 2 * c * self.hitch[1] - self.hitch[0]**2 * m**2 + 2 * self.hitch[0] * self.hitch[1] * m - self.hitch[1]**2 + m**2 * diag**2 + diag**2)) / (m**2 + 1)
        y[1] = (c + self.hitch[0] * m + self.hitch[1] * m**2 - m * math.sqrt(- c**2 - 2 * c * self.hitch[0] * m + 2 * c * self.hitch[1] - self.hitch[0]**2 * m**2 + 2 * self.hitch[0] * self.hitch[1] * m - self.hitch[1]**2 + m**2 * diag**2 + diag**2)) / (m**2 + 1)

        new_axle = [0,0]
        if (math.sqrt(abs(self.axle[0] - x[0])**2 + abs(self.axle[1] - y[0])**2) < math.sqrt(abs(self.axle[0] - x[1])**2 + abs(self.axle[1] - y[1])**2)):
            new_axle[0] = x[0]
            new_axle[1] = y[0]
        else:
            new_axle[0] = x[1]
            new_axle[1] = y[1]

        diag_angle = math.atan2((new_hitch[1] - new_axle[1]), (new_hitch[0] - new_axle[0]))

        new_kink = [0,0]
        new_kink[0] = self.hitch[0] - L2 * math.cos(diag_angle - tau1)
        new_kink[1] = self.hitch[1] - L2 * math.sin(diag_angle - tau1)

        if L3 == 0: # wheelsteer
            new_psi = diag_angle
        else:
            new_psi = math.atan((new_axle[1] - new_kink[1]) / (new_axle[0] - new_kink[0]))

        d_psi = (self.psi - new_psi) % math.pi
        ds = math.sqrt((self.axle[0] - new_axle[0])**2 + (self.axle[1] - new_axle[1])**2)


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
