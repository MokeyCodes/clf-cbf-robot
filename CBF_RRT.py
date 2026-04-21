import math
import random
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import cvxpy as cp
import numpy as np
import copy


class Node:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.parent = None


class RRT:
    def __init__(self, start, goal, obstacles,
                 x_bounds=(0, 10), y_bounds=(0, 10),
                 step_size=0.2, goal_sample_rate=0.1,
                 max_iter=3000, robot_radius=0.15):
        self.start = Node(*start)
        self.goal = Node(*goal)
        self.obs = obstacles
        self.x_min, self.x_max = x_bounds
        self.y_min, self.y_max = y_bounds
        self.step_size = step_size
        self.goal_sample_rate = goal_sample_rate
        self.max_iter = max_iter
        self.robot_radius = robot_radius
        self.nodes = [self.start]

    def sample(self):
        if random.random() < self.goal_sample_rate:
            return Node(self.goal.x, self.goal.y)
        return Node(random.uniform(self.x_min, self.x_max),
                    random.uniform(self.y_min, self.y_max))

    def nearest(self, sample):
        return min(self.nodes, key=lambda n: self.dist(n, sample))


    def steer(self, from_node, to_node):
        d = self.dist(from_node, to_node)
        if d < 1e-6:
            return from_node
        ratio = min(self.step_size / d, 1.0)
        new_node = Node(from_node.x + ratio * (to_node.x - from_node.x),
                        from_node.y + ratio * (to_node.y - from_node.y))
        new_node.parent = from_node
        return new_node

    def collision_free(self, from_node, to_node):
        n_checks = max(int(self.dist(from_node, to_node) / 0.05), 2)
        for i in range(n_checks + 1):
            t = i / n_checks
            x = from_node.x + t * (to_node.x - from_node.x)
            y = from_node.y + t * (to_node.y - from_node.y)
            for (ox, oy, r) in self.obs:
                if math.hypot(x - ox, y - oy) < r + self.robot_radius:
                    return False
        return True

    def plan(self):
        for _ in range(self.max_iter):
            rand_node = self.sample()
            near_node = self.nearest(rand_node)
            new_node = self.steer(near_node, rand_node)
            if not self.collision_free(near_node, new_node):
                continue
            self.nodes.append(new_node)

            if self.dist(new_node, self.goal) <= self.step_size:
                if self.collision_free(new_node, self.goal):
                    self.goal.parent = new_node
                    self.nodes.append(self.goal)
                    return self._extract_path()
        print("RRT: no path found")
        return []

    def dist(self, a, b):
        return math.hypot(a.x - b.x, a.y - b.y)

    def _extract_path(self):
        path, node = [], self.goal
        while node is not None:
            path.append((node.x, node.y))
            node = node.parent
        path.reverse()
        return path

class RRTStar(RRT):
    def near(self, node):
        radius = 0.5
        return [n for n in self.nodes
                if self.dist(n, node) < radius]

    def cost(self, node):
        c = 0
        while node.parent is not None:
            c += self.dist(node, node.parent)
            node = node.parent
        return c

    def plan(self):
        for _ in range(self.max_iter):
            rand_node = self.sample()
            near_node = self.nearest(rand_node)
            new_node  = self.steer(near_node, rand_node)
            if not self.collision_free(near_node, new_node):
                continue
            self.nodes.append(new_node)

            # ---------------- RRT* Additions --------------------
            for candidate in self.near(new_node):
                new_cost = self.cost(candidate) + self.dist(candidate, new_node)
                if new_cost < self.cost(new_node) and self.collision_free(candidate, new_node):
                    new_node.parent = candidate

            for candidate in self.near(new_node):
                new_cost = self.cost(new_node) + self.dist(new_node, candidate)
                if new_cost < self.cost(candidate) and self.collision_free(new_node, candidate):
                    candidate.parent = new_node
            # ---------------- RRT* Additions --------------------
            if self.dist(new_node, self.goal) <= self.step_size:
                if self.collision_free(new_node, self.goal):
                    self.goal.parent = new_node
                    self.nodes.append(self.goal)
                    return self._extract_path()

        print("RRT*: no path found")
        return []



class RobotController:
    def __init__(self, x, y, theta, goal_x, goal_y, waypoints, OBSTACLES):
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
        self.alpha = 1.5
        self.l = 0.2

        self.v_min = 0.0
        self.v_max = 2.0
        self.w_min = -4.5
        self.w_max = 4.5

        self.obstacles = copy.deepcopy(OBSTACLES)

        self.traj_x = [x]
        self.traj_y = [y]

        self.collision_count = 0

        # waypoints is a list of (x, y) from RRT.plan().
        # wp_index: index of current waypoint
        # wp_threshold: once the robot is this close, advance to next waypoint.
        self.waypoints = waypoints
        self.wp_index = 0
        self.wp_threshold = 0.35

    def calcNom(self):
        # Keep stepping forward while we're within threshold of the current
        # waypoint AND there's still a next one to go to.
        # The 'while' (not 'if') handles the case where two waypoints are very
        # close together and the robot passes through both in one step.
        while self.wp_index < len(self.waypoints) - 1:
            wx, wy = self.waypoints[self.wp_index]
            if math.hypot(self.x - wx, self.y - wy) < self.wp_threshold:
                self.wp_index += 1
            else:
                break

        # calculates new "Goals" (waypoints) for nominal calculations
        wx, wy = self.waypoints[self.wp_index]
        dx = wx - self.x
        dy = wy - self.y

        goal_theta = math.atan2(dy, dx)
        e_theta    = self.wrap_angle(goal_theta - self.theta)

        distance = math.hypot(dx, dy)
        if distance < 0.02:
            return 0.0, 0.0
        v_nom = self.Kp * distance * math.cos(e_theta)
        w_nom = self.Kw * e_theta
        return v_nom, w_nom

    def solveCBFQP(self):
        v = cp.Variable()
        w = cp.Variable()

        v_nom, w_nom = self.calcNom()
        objective = cp.Minimize((v - v_nom)**2 + (w - w_nom)**2)

        constraints = [
            v >= self.v_min, v <= self.v_max,
            w >= self.w_min, w <= self.w_max,
        ]

        xa, ya = self.lookahead_point()
        for (xo, yo, r, vox, voy) in self.obstacles:
            h = (xa - xo)**2 + (ya - yo)**2 - r**2

            A_cbf = (2*(xa-xo)*math.cos(self.theta)
                     + 2*(ya-yo)*math.sin(self.theta))
            B_cbf = (-2*self.l*(xa-xo)*math.sin(self.theta)
                     + 2*self.l*(ya-yo)*math.cos(self.theta))
            C_cbf = (-2*(xa-xo)*vox - 2*(ya-yo)*voy)

            h_dot = A_cbf*v + B_cbf*w + C_cbf
            constraints.append(h_dot + self.alpha*h >= 0)

        prob = cp.Problem(objective, constraints)
        prob.solve(solver="Clarabel")

        if v.value is None or w.value is None:
            return 0.0, 0.0
        return float(v.value), float(w.value)

    def step(self):
        v, w = self.solveCBFQP()
        self.x += self.dt * v * math.cos(self.theta)
        self.y += self.dt * v * math.sin(self.theta)
        self.theta = self.wrap_angle(self.theta + self.dt * w)
        self.v_prev = v
        self.w_prev = w
        self.traj_x.append(self.x)
        self.traj_y.append(self.y)

        # check collisions
        for obs in self.obstacles:
            if math.hypot(self.x - obs[0], self.y - obs[1]) < obs[2]:
                self.collision_count += 1

        return self.x, self.y, self.theta

    def wrap_angle(self, angle):
        while angle > math.pi: angle -= 2*math.pi
        while angle < -math.pi: angle += 2*math.pi
        return angle

    def lookahead_point(self):
        xa = self.x + self.l * math.cos(self.theta)
        ya = self.y + self.l * math.sin(self.theta)
        return xa, ya


def main():
    START = (0.0, 0.0)
    GOAL  = (5.0, 5.0)

    OBSTACLES = [
        # static wall
        [3.5, 1.5, 0.5, 0, 0],
        [3.5, 2.5, 0.5, 0, 0],
        [3.5, 3.5, 0.5, 0, 0],
        [3.5, 4.5, 0.5, 0, 0],

        [1.5, 3.0, 0.45, 0, 0],
        [5.5, 3.0, 0.45, 0, 0],
        [1.0, 2.0, 0.3, 0.7, 0.0],
        [1.0, 4.0, 0.3, -0.7, 0.0],
        [5.0, 2.0, 0.3, -0.7, 0.0],
        [5.0, 4.0, 0.3, 0.7, 0.0],
        [1.2, 1.0, 0.25, 0.0, 0.6],
        [5.8, 5.0, 0.25, 0.0, -0.6],
        [0.5, 5.0, 0.3, 0.4, -0.4],
        [6.0, 1.0, 0.3, -0.4, 0.4],
        [0.5, 0.5, 0.25, 0.5, 0.5],
        [2.0, 2.5, 0.3, 0.4, 0.0],
        [2.0, 3.5, 0.3, -0.4, 0.0],
        [4.8, 2.5, 0.3, -0.4, 0.0],
        [4.8, 3.5, 0.3, 0.4, 0.0],
        [4.5, 5.0, 0.25, 0.5, 0.2],
        [4.0, 5.5, 0.25, -0.3, 0.4],
        [5.0, 5.5, 0.25, 0.2, -0.5],
        [4.0, 4.8, 0.7, -0.1, 0.1],
        [0.8, 0.8, 0.2, 0.6, 0.4],
        [1.2, 0.4, 0.2, -0.5, 0.6],
        [0.4, 1.5, 0.2, 0.4, -0.5],
    ]



    # CHOOSE STATIC OBS ONLY OR ALL OBS CALCULATED FOR RRT
    # --- STATIC ONLY ---
    static_obs = [(obs[0], obs[1], obs[2]) for obs in OBSTACLES if obs[3] == 0 and obs[4] == 0]
    
    # --- ALL OBS -------
    # static_obs = [(obs[0], obs[1], obs[2]) for obs in OBSTACLES]
    


    # CHOOSE BETWEEN RRT or RRT*
    # ------- RRTStar --------
    planner = RRTStar(start=START, goal=GOAL, obstacles=static_obs)
    
    # ------- RRT ------------
    # planner = RRT(start=START, goal=GOAL, obstacles=static_obs)



    waypoints = planner.plan()
    name = "RRT*" if isinstance(planner, RRTStar) else "RRT"
    if not waypoints:
        print(f"{name} failed — cannot continue.")
        return
    print(f"{name} found {len(waypoints)} waypoints.")

    # ── Step 2: hand the waypoints to the CBF controller ─────────────────────
    robot = RobotController(
        x=START[0], y=START[1], theta=0.0,
        goal_x=GOAL[0], goal_y=GOAL[1],
        waypoints=waypoints, OBSTACLES=OBSTACLES
    )

    fig, ax = plt.subplots()
    ax.set_aspect("equal")
    ax.set_xlim(-0.5, 7.5)
    ax.set_ylim(-0.5, 7.5)
    ax.grid(True)
    ax.plot(GOAL[0], GOAL[1], "gx", markersize=10, label="goal")

    # draw RRT path
    if waypoints:
        wx, wy = zip(*waypoints)
        ax.plot(wx, wy, "b--", linewidth=1, alpha=0.4, label=f"{name} Path")
        for (px, py) in waypoints:
            ax.plot(px, py, "b.", markersize=4, alpha=0.5)

    obstacle_patches = []
    for (xo, yo, r, vxo, vyo) in robot.obstacles:
        circle = plt.Circle((xo, yo), r, color="r", alpha=0.3)
        ax.add_patch(circle)
        obstacle_patches.append(circle)

    traj_line, = ax.plot([], [], label="trajectory")
    robot_point, = ax.plot([], [], "bo", label="robot")
    lookahead_point, = ax.plot([], [], "ko", label="look-ahead")
    heading_line, = ax.plot([], [], "b-", linewidth=2)
    wp_marker, = ax.plot([], [], "r*", markersize=10, label="current waypoint")

    ax.legend(loc="upper right", fontsize=7)

    def init():
        traj_line.set_data([], [])
        robot_point.set_data([], [])
        lookahead_point.set_data([], [])
        heading_line.set_data([], [])
        wp_marker.set_data([], [])
        collision_text.set_text("Collisions: 0")
        return traj_line, robot_point, lookahead_point, heading_line, wp_marker, collision_text, *obstacle_patches


    collision_text = ax.text(0.02, 0.98, "Collisions: 0",
                         transform=ax.transAxes,
                         verticalalignment="top",
                         fontsize=12,
                         color="red")
    def update(frame):
        for obs in robot.obstacles:
            obs[0] += robot.dt * obs[3]
            obs[1] += robot.dt * obs[4]


            # Bounce obstacles when reaching edge
            if obs[0] < 0 or obs[0] > 7:
                obs[3] *= -1
            if obs[1] < 0 or obs[1] > 7:
                obs[4] *= -1

        
        for patch, obs in zip(obstacle_patches, robot.obstacles):
            patch.center = (obs[0], obs[1])

        robot.step()
        xa, ya = robot.lookahead_point()

        traj_line.set_data(robot.traj_x, robot.traj_y)
        robot_point.set_data([robot.x], [robot.y])
        lookahead_point.set_data([xa], [ya])

        collision_text.set_text(f"Collisions: {robot.collision_count}")

        hx = [robot.x, robot.x + robot.l *math.cos(robot.theta)]
        hy = [robot.y, robot.y + robot.l *math.sin(robot.theta)]
        heading_line.set_data(hx, hy)

        # show which waypoint is currently targeted
        cwx, cwy = robot.waypoints[robot.wp_index]
        wp_marker.set_data([cwx], [cwy])

        return traj_line, robot_point, lookahead_point, heading_line, wp_marker, collision_text, *obstacle_patches

    ani = animation.FuncAnimation(
        fig, update, frames=400,
        init_func=init, interval=50, blit=True,
    )
    plt.show()


if __name__ == "__main__":
    main()
