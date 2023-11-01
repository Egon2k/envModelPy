import matplotlib.pyplot as plt
import numpy as np
import math

H = 0.4
L1 = 5
L2 = 3
L3 = 2

dt = 0.01
t_sim = 5
n_steps_shown = 20
zoom = 20

def r(val):
    return round(val, 10)


class Tractor:
    def __init__(self, dt, psi = 0):
        self.psi   = psi
        self.dt    = dt
        self.front = [      0 , 0]
        self.rear  = [   - L1 , 0]
        self.hitch = [-H - L1 , 0]
        

    def step(self, v, steering_angle = 0):
        ds = v * self.dt
        d_psi = math.asin(ds * math.sin(math.pi - steering_angle) / L1)

        new_psi = self.psi + d_psi

        if new_psi > math.pi:
            new_psi -= 2 * math.pi
        elif new_psi < -math.pi:
            new_psi += 2 * math.pi

        new_front = [self.front[0] + ds * math.cos(steering_angle + self.psi),
                     self.front[1] + ds * math.sin(steering_angle + self.psi)]
        new_rear  = [new_front[0] - L1 * math.cos(new_psi), 
                     new_front[1] - L1 * math.sin(new_psi)]
        new_hitch = [new_front[0] - (L1 + H) * math.cos(new_psi), 
                     new_front[1] - (L1 + H) * math.sin(new_psi)]
        
        self.psi = new_psi
        self.front = new_front
        self.rear = new_rear
        self.hitch = new_hitch

    def print(self):
        print("Tractor")
        print(f'front: {r(self.front[0])}, {r(self.front[1])}')
        print(f'rear: {r(self.rear[0])}, {r(self.rear[1])}')
        print(f'hitch: {r(self.hitch[0])}, {r(self.hitch[1])}')

class Trailer:
    def __init__(self, hitch, psi = 0, beta = 0):
        self.psi   = psi
        self.hitch = hitch
        self.pivot = [self.hitch[0] - L2 * math.cos(self.psi),
                      self.hitch[1] - L2 * math.sin(self.psi)]
        self.axle  = [self.pivot[0] - L3 * math.cos(self.psi + beta),
                      self.pivot[1] - L3 * math.sin(self.psi + beta)]

    def step(self, hitch, beta = 0):
        new_hitch = hitch
        new_pivot = []
        new_axle  = []

        # diag = distance between hitch and axle (for L3 = 0 => diag = L2)
        diag = math.sqrt(L2**2 + L3**2 - 2 * L2 * L3 * math.cos(math.pi - beta))
        tau1 =  math.asin(L3 * math.sin(math.pi - beta) / diag)
        tau2 =  math.asin(L2 * math.sin(math.pi - beta) / diag)

        if L3 == 0: # wheelsteer
            A = np.array([[self.pivot[0], 1], [self.pivot[0] + math.cos(self.psi + beta), 1]])
            b = np.array([self.pivot[1], self.pivot[1] + math.sin(self.psi + beta)])
        else:
            A = np.array([[self.pivot[0], 1], [self.axle[0], 1]])
            b = np.array([self.pivot[1], self.axle[1]])

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

        new_pivot = [0,0]
        new_pivot[0] = self.hitch[0] - L2 * math.cos(diag_angle - tau1)
        new_pivot[1] = self.hitch[1] - L2 * math.sin(diag_angle - tau1)

        if L3 == 0: # wheelsteer
            new_psi = diag_angle
        else:
            new_psi = math.atan((new_axle[1] - new_pivot[1]) / (new_axle[0] - new_pivot[0]))

        d_psi = (self.psi - new_psi) % math.pi
        ds = math.sqrt((self.axle[0] - new_axle[0])**2 + (self.axle[1] - new_axle[1])**2)

        self.hitch = new_hitch
        self.pivot = new_pivot
        self.axle = new_axle


    def print(self):
        print("Trailer")
        print(f'hitch: {r(self.hitch[0])}, {r(self.hitch[1])}')
        print(f'pivot: {r(self.pivot[0])}, {r(self.pivot[1])}')
        print(f'axle: {r(self.axle[0])}, {r(self.axle[1])}')

def plot(tractor, trailer1):
    plt.plot([tractor.front[0], tractor.rear[0]], [tractor.front[1], tractor.rear[1]], linestyle="-", color="red")
    plt.plot([tractor.rear[0], tractor.hitch[0]], [tractor.rear[1], tractor.hitch[1]], linestyle="-", color="black")
    plt.plot([trailer1.hitch[0], trailer1.pivot[0]], [trailer1.hitch[1], trailer1.pivot[1]], linestyle="-", color="green")
    plt.plot([trailer1.pivot[0], trailer1.axle[0]], [trailer1.pivot[1], trailer1.axle[1]], linestyle="-", color="blue")

if __name__ == "__main__":
    tractor = Tractor(dt)
    trailer1 = Trailer(tractor.hitch)
   
    plt.xlim(-zoom, zoom)
    plt.ylim(-zoom, zoom)
    plt.grid()
    plt.gca().set_aspect('equal')

    n_steps = math.ceil(t_sim / dt)
    
    for i in range(n_steps):
        if i % (n_steps / n_steps_shown) == 0:
            plot(tractor, trailer1)
    
        tractor.step(v = 4, steering_angle = 15 * math.pi/180)
        trailer1.step(hitch = tractor.hitch, beta = 10 * math.pi / 180)

    plt.show()
    