import math
import random

class TreeNode:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.parent = None


class RRT:
    def __init__(self, start, goal, obstacles,
                 x_bounds=(0, 10), y_bounds=(0, 10),
                 step_size=0.2, goal_sample_rate=0.1,
                 max_iter=800, robot_radius=0.15):
        self.start = TreeNode(*start)
        self.goal = TreeNode(*goal)
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
            return TreeNode(self.goal.x, self.goal.y)
        return TreeNode(random.uniform(self.x_min, self.x_max),
                    random.uniform(self.y_min, self.y_max))

    def nearest(self, sample):
        return min(self.nodes, key=lambda n: self.dist(n, sample))


    def steer(self, from_node, to_node):
        d = self.dist(from_node, to_node)
        if d < 1e-6:
            return from_node
        ratio = min(self.step_size / d, 1.0)
        new_node = TreeNode(from_node.x + ratio * (to_node.x - from_node.x),
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