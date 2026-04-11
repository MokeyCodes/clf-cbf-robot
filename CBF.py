import math
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import cvxpy as cp
import numpy as np

class RobotController:
    def __init__(self, x, y, theta, goal_x, goal_y):
        self.x = x
        self.y = y
        self.theta = theta

        self.goal_x = goal_x
        self.goal_y = goal_y
        self.Kp = 3.0
        self.Kw = 3.0

        self.v_prev = 0.0
        self.w_prev = 0.0

        self.dt = 0.05
        self.alpha = 2.0
        self.l = 0.4

        

        self.v_min = 0.0
        self.v_max = 4.0
        self.w_min = -10.5
        self.w_max = 10.5

        # one circular obstacle: (x_obs, y_obs, radius, vxo, vyo)
        self.obstacles = [
            [3, 2, 0.4, -0.25, 0.0],
            [1, 2, 0.4, 0.15, 0.0],
            [3, 1, 0.6, 0.0, 0.4],
            [5, 2, 0.4, -0.3, 0.2],
            [1, 5, 0.5, 0.4, -0.3],
            [4, 1.5, 0.5, 0, 0],
            [3, 3.5, 0.5, 0, 0],
            [3, 0.5, 1.3, 0, 0],
            [4, 5.5, 0.2, 0, 0],
            [4.5, 4.5, 0.2, 0, 0]
        ]

        self.traj_x = [x]
        self.traj_y = [y]


    def calcNom(self):
        dx = (self.goal_x - self.x)
        dy = (self.goal_y - self.y)

        goal_theta = math.atan2(dy, dx)
        e_theta = self.wrap_angle(goal_theta - self.theta)


        distance = math.sqrt((dy**2) + (dx**2))
        if distance < 0.02:
            return 0.0, 0.0
        v_nom = self.Kp * distance * math.cos(e_theta)
        w_nom = self.Kw * e_theta

        return v_nom, w_nom




    def solveCBFQP(self):
        v = cp.Variable()
        w = cp.Variable()

        v_nom, w_nom = self.calcNom()
        objective = cp.Minimize(
            (v - v_nom) ** 2 + (w - w_nom) ** 2
        )

        constraints = [
            v >= self.v_min,
            v <= self.v_max,
            w >= self.w_min,
            w <= self.w_max
        ]

        

        # CBF
        xa, ya = self.lookahead_point()
        for (xo, yo, r, vox, voy) in self.obstacles:
            h = (xa - xo)**2 + (ya - yo)**2 - (r)**2

            A_cbf = (
                (2 * (xa - xo) * math.cos(self.theta)) 
                + (2 * (ya - yo) * math.sin(self.theta))
            )

            B_cbf = (
                (-2 * self.l * (xa-xo) * math.sin(self.theta)) 
                + (2 * self.l * (ya-yo) * math.cos(self.theta))
            )

            C_cbf = (
                (-2 * (xa-xo) * vox) - (2 * (ya - yo) * voy)
            )

            h_dot = (
                (A_cbf * v)
                + (B_cbf * w) 
                + C_cbf
            )
            constraints.append(h_dot + self.alpha * h >= 0)
        
        prob = cp.Problem(objective, constraints)
        prob.solve(solver="Clarabel")


        if v.value is None or w.value is None:
            return 0.0, 0.0
        
        return float(v.value), float(w.value)
        

    def step(self):
        v, w = self.solveCBFQP()

        self.x += self.dt * v * math.cos(self.theta)
        self.y += self.dt * v * math.sin(self.theta)
        self.theta += self.dt * w
        self.theta = self.wrap_angle(self.theta)
        self.v_prev = v
        self.w_prev = w

        self.traj_x.append(self.x)
        self.traj_y.append(self.y)

        return self.x, self.y, self.theta

    def wrap_angle(self, angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle
    
    def lookahead_point(self):
        xa = self.x + self.l * math.cos(self.theta)
        ya = self.y + self.l * math.sin(self.theta)
        return xa, ya


def main():
    robot = RobotController(
        x=0.0,
        y=0.0,
        theta=0.0,
        goal_x=5.0,
        goal_y=5.0,
    )

    fig, ax = plt.subplots()
    ax.set_aspect("equal")
    ax.set_xlim(-0.5, 7.5)
    ax.set_ylim(-0.5, 7.5)
    ax.grid(True)

    # goal
    ax.plot(robot.goal_x, robot.goal_y, "gx", markersize=10, label="goal")

    # obstacles
    obstacle_patches = []

    for (xo, yo, r, vxo, vyo) in robot.obstacles:
        circle = plt.Circle((xo, yo), r, color="r", alpha=0.3)
        ax.add_patch(circle)
        obstacle_patches.append(circle)

    # animated elements
    traj_line, = ax.plot([], [], label="trajectory")
    robot_point, = ax.plot([], [], "bo", label="robot")
    lookahead_point, = ax.plot([], [], "ko", label="look-ahead")
    heading_line, = ax.plot([], [], "b-", linewidth=2)

    ax.legend()

    def init():
        traj_line.set_data([], [])
        robot_point.set_data([], [])
        lookahead_point.set_data([], [])
        heading_line.set_data([], [])
        return traj_line, robot_point, lookahead_point, heading_line, *obstacle_patches

    def update(frame):
        for obs in robot.obstacles:
            obs[0] += robot.dt * obs[3]
            obs[1] += robot.dt * obs[4]

        for patch, obs in zip(obstacle_patches, robot.obstacles):
            xo, yo, r, vxo, vyo = obs
            patch.center = (xo, yo)
        robot.step()

        xa, ya = robot.lookahead_point()

        traj_line.set_data(robot.traj_x, robot.traj_y)
        robot_point.set_data([robot.x], [robot.y])
        lookahead_point.set_data([xa], [ya])

        hx = [robot.x, robot.x + 0.35 * math.cos(robot.theta)]
        hy = [robot.y, robot.y + 0.35 * math.sin(robot.theta)]
        heading_line.set_data(hx, hy)

        return traj_line, robot_point, lookahead_point, heading_line, *obstacle_patches

    ani = animation.FuncAnimation(
        fig,
        update,
        frames=500,
        init_func=init,
        interval=50,
        blit=True
    )

    plt.show()

if __name__ == "__main__":
    main()

