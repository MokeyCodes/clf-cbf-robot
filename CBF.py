import math
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import cvxpy as cp
import numpy as np

class RobotController:
    def __init__(self, x, y, theta, goal_x, goal_y, goal_theta):
        self.x = x
        self.y = y
        self.theta = theta

        self.goal_x = goal_x
        self.goal_y = goal_y
        self.goal_theta = goal_theta

        self.v_prev = 0.0
        self.w_prev = 0.0
        self.q_v = 1
        self.q_w = 1

        self.dt = 0.05
        self.alpha = 2.0
        self.l = 0.3

        self.clf_rate = 1.0   # c
        self.p = 1000.0        # slack penalty

        self.a1 = 1.0
        self.a2 = 1.0
        self.a3 = 0.8
        self.b1 = 0.15
        self.b2 = 0.15

        self.v_min = 0
        self.v_max = 2.0
        self.w_min = -1.5
        self.w_max = 1.5

        # one circular obstacle: (x_obs, y_obs, radius, vxo, vyo)
        self.obstacles = [
            [3, 2, 0.4, -0.2, 0.0],
            [1, 2, 0.4, 0.2, 0.0],
            [3, 1, 0.6, 0.0, 0.5],
            [5, 2, 0.4, -0.3, 0.2],
            [1, 5, 0.5, 0.4, -0.3],
        ]

        self.traj_x = [x]
        self.traj_y = [y]


    def calcCLF(self):
        ex = self.x - self.goal_x
        ey = self.y - self.goal_y
        e_theta = self.wrap_angle(self.theta - self.goal_theta)
        
        V = (
            self.a1*ex**2 
            + self.a2*ey**2 
            + self.a3*e_theta**2 
            + 2*self.b1*ex*e_theta 
            + 2*self.b2*ey*e_theta
        )

        A_clf = 2 * (
            self.a1 * ex * math.cos(self.theta) 
            + self.a2 * ey * math.sin(self.theta) 
            + self.b1 * e_theta * math.cos(self.theta) 
            + self.b2 * e_theta * math.sin(self.theta)
        )
        B_clf = 2 * (
            self.b1 * ex 
            + self.b2 * ey 
            + self.a3 * e_theta
        )
        return V, A_clf, B_clf


    def solveCLFCBFQP(self):
        v = cp.Variable()
        w = cp.Variable()
        delta = cp.Variable(nonneg=True)
        objective = cp.Minimize(
            v**2 + w**2
            + self.q_v * (v - self.v_prev)**2
            + self.q_w * (w - self.w_prev)**2
            + self.p * delta**2
        )

        constraints = [
            v >= self.v_min,
            v <= self.v_max,
            w >= self.w_min,
            w <= self.w_max
        ]

        V, A_clf, B_clf = self.calcCLF()

        constraints.append(A_clf * v + B_clf * w <= -self.clf_rate * V + delta)

        # CBF
        xa, ya = self.lookahead_point()
        for (xo, yo, r, vox, voy) in self.obstacles:
            h = (xa - xo)**2 + (ya - yo)**2 - (r)**2

            A_cbf = (
                2 * (xa - xo) * math.cos(self.theta) 
                + 2 * (ya - yo) * math.sin(self.theta)
            )

            B_cbf = (
                -2 * self.l * (xa-xo) * math.sin(self.theta) 
                + 2 * self.l * (ya-yo) * math.cos(self.theta)
            )

            C_cbf = (
                -2 * (xa-xo) * vox - 2 * (ya - yo) * voy
            )

            h_dot = (
                A_cbf * v
                + B_cbf * w 
                + C_cbf
            )
            constraints.append(h_dot + self.alpha * h >= 0)
        
        prob = cp.Problem(objective, constraints)
        prob.solve(solver="Clarabel")
        
        return float(v.value), float(w.value)
        

    def step(self):
        v, w = self.solveCLFCBFQP()

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
        goal_theta = 0.0,
    )

    fig, ax = plt.subplots()
    ax.set_aspect("equal")
    ax.set_xlim(-0.5, 5.5)
    ax.set_ylim(-0.5, 5.5)
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

