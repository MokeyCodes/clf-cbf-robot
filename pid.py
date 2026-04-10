import matplotlib.pyplot as plt
import math
class RobotController:
    def __init__(self, Kp_dist, Kp_theta, repulse_gain):
        self.Kp_dist = Kp_dist
        self.Kp_theta = Kp_theta
        self.repulse_gain = repulse_gain
    
    def update(self, x, y, theta, target_x, target_y, dt, obstacles):
        if obstacles is None:
            obstacles = []
        error_x = target_x - x
        error_y = target_y - y
        distance = math.sqrt((error_x ** 2 + error_y**2))

        repulse_x, repulse_y = 0.0, 0.0
        for obstacle in obstacles:
            away_x = x - obstacle[0] 
            away_y = y - obstacle[1]

            obs_dist = math.sqrt(away_x**2 + away_y**2)
            obs_surface = obs_dist - obstacle[2]
            if 0.001 < (obs_surface) < obstacle[3]:
                strength = (obstacle[3] - obs_surface) / obstacle[3]
                repulse_x = (away_x / obs_dist) * strength * self.repulse_gain
                repulse_y = (away_y / obs_dist) * strength * self.repulse_gain

        
        target_theta = math.atan2(error_y + repulse_y, error_x + repulse_x)
        error_theta = self.wrap_angle((target_theta - theta))

        omega = self.Kp_theta * error_theta
        theta += omega * dt
        theta = self.wrap_angle(theta)
        
        if abs(error_theta) > 0.3:
            v = 0
        else:
            v = min((self.Kp_dist * distance), 3.0)

        x += v * math.cos(theta) * dt
        y += v * math.sin(theta) * dt


        return x, y, theta
    
    def wrap_angle(self, angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle
     

x, y, theta = 0.0, 0.0, math.radians(150.0)
target_x, target_y = 10, 10
# Obstacles:
# obs_x, obs_y, obs_radius, safety
obstacles = [
    (5, 6, 1, 2),
    (0, 1, 0.5, 5),
    (9, 4, 1, 2),
    (2.5, 0.5, 0.25, 1)
]

dt = 0.02
pid = RobotController(Kp_dist=1.0, Kp_theta=3.0, repulse_gain=12)

x_hist = []
y_hist = []

for _ in range(500):
    x, y, theta = pid.update(x, y, theta, target_x, target_y, dt, obstacles)
    x_hist.append(x)
    y_hist.append(y)

# Plot path
plt.figure()
plt.plot(x_hist, y_hist, label="Path")
plt.plot(target_x, target_y, 'ro', label="Target")
for obstacle in obstacles:
    circle = plt.Circle((obstacle[0], obstacle[1]), obstacle[2], color='red', alpha=0.5)
    plt.gca().add_patch(circle)
plt.xlabel("X")
plt.ylabel("Y")
plt.title("2D PID Movement")
plt.legend()
plt.grid(True)
plt.axis("equal")
plt.show()

