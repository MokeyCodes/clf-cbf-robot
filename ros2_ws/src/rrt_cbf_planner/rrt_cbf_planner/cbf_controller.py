import math
import cvxpy as cp


ROBOT_RADIUS = 0.105  # TurtleBot3 Burger body radius (m)


class CBFController:
    def __init__(self, alpha=1.5, l=0.1, Kp=1.0, Kw=2.0,
                 v_min=0.0, v_max=0.22, w_min=-2.84, w_max=2.84,
                 wp_threshold=0.2):
        self.alpha = alpha
        self.l = l
        self.Kp = Kp
        self.Kw = Kw
        self.v_min = v_min
        self.v_max = v_max
        self.w_min = w_min
        self.w_max = w_max
        self.wp_threshold = wp_threshold

        # state — set by the node each control cycle
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.waypoints = []   # list of (x, y)
        self.wp_index = 0
        self.obstacles = []   # [[x, y, r, vx, vy], ...]

    def lookahead_point(self):
        return (
            self.x + self.l * math.cos(self.theta),
            self.y + self.l * math.sin(self.theta),
        )

    def calcNom(self):
        while self.wp_index < len(self.waypoints) - 1:
            wx, wy = self.waypoints[self.wp_index]
            if math.hypot(self.x - wx, self.y - wy) < self.wp_threshold:
                self.wp_index += 1
            else:
                break

        wx, wy = self.waypoints[self.wp_index]
        dx, dy = wx - self.x, wy - self.y
        dist = math.hypot(dx, dy)
        if dist < 0.02:
            return 0.0, 0.0
        e_theta = self.wrap_angle(math.atan2(dy, dx) - self.theta)
        return self.Kp * dist * math.cos(e_theta), self.Kw * e_theta

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
        for obs in self.obstacles:
            xo, yo, r, vox, voy = obs
            h = (xa - xo)**2 + (ya - yo)**2 - (r + ROBOT_RADIUS)**2
            A = 2*(xa-xo)*math.cos(self.theta) + 2*(ya-yo)*math.sin(self.theta)
            B = (-2*self.l*(xa-xo)*math.sin(self.theta)
                 + 2*self.l*(ya-yo)*math.cos(self.theta))
            C = -2*(xa-xo)*vox - 2*(ya-yo)*voy
            constraints.append(A*v + B*w + C + self.alpha*h >= 0)

        prob = cp.Problem(objective, constraints)
        prob.solve(solver='Clarabel')

        if v.value is None or w.value is None:
            return self._recover()
        return float(v.value), float(w.value)

    def _recover(self):
        """Back away from the nearest obstacle when the QP is infeasible."""
        if not self.obstacles:
            return -0.05, 0.0
        nearest = min(self.obstacles, key=lambda o: math.hypot(self.x - o[0], self.y - o[1]))
        xo, yo = nearest[0], nearest[1]
        away_angle = math.atan2(self.y - yo, self.x - xo)
        angle_diff = self.wrap_angle(away_angle - self.theta)
        # obstacle is behind us → drive forward away; in front → back up
        v = 0.1 if abs(angle_diff) > math.pi / 2 else -0.1
        return v, 0.0

    def wrap_angle(self, a):
        while a > math.pi: a -= 2*math.pi
        while a < -math.pi: a += 2*math.pi
        return a
