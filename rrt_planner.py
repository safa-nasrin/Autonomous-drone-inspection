import numpy as np
import random
import math

class Node:
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z
        self.parent = None
        self.cost = 0.0

class Obstacle:
    def __init__(self, x, y, z, size_x, size_y, size_z):
        self.x = x
        self.y = y
        self.z = z
        self.sx = size_x / 2
        self.sy = size_y / 2
        self.sz = size_z / 2

    def collides(self, x, y, z, margin=1.5):
        return (abs(x - self.x) < self.sx + margin and
                abs(y - self.y) < self.sy + margin and
                abs(z - self.z) < self.sz + margin)

class RRTStar:
    def __init__(self, start, goal, obstacles, bounds, max_iter=2000, step=2.0, radius=5.0):
        self.start = Node(*start)
        self.goal = Node(*goal)
        self.obstacles = obstacles
        self.bounds = bounds
        self.max_iter = max_iter
        self.step = step
        self.radius = radius
        self.nodes = [self.start]

    def distance(self, a, b):
        return math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2 + (a.z-b.z)**2)

    def random_node(self):
        if random.random() < 0.1:
            return Node(self.goal.x, self.goal.y, self.goal.z)
        return Node(
            random.uniform(self.bounds[0], self.bounds[1]),
            random.uniform(self.bounds[2], self.bounds[3]),
            random.uniform(self.bounds[4], self.bounds[5])
        )

    def nearest(self, node):
        return min(self.nodes, key=lambda n: self.distance(n, node))

    def steer(self, from_node, to_node):
        d = self.distance(from_node, to_node)
        if d < self.step:
            return to_node
        ratio = self.step / d
        return Node(
            from_node.x + ratio * (to_node.x - from_node.x),
            from_node.y + ratio * (to_node.y - from_node.y),
            from_node.z + ratio * (to_node.z - from_node.z)
        )

    def collision_free(self, a, b):
        steps = int(self.distance(a, b) / 0.5) + 1
        for i in range(steps + 1):
            t = i / steps
            x = a.x + t * (b.x - a.x)
            y = a.y + t * (b.y - a.y)
            z = a.z + t * (b.z - a.z)
            for obs in self.obstacles:
                if obs.collides(x, y, z):
                    return False
        return True

    def near_nodes(self, node):
        return [n for n in self.nodes if self.distance(n, node) < self.radius]

    def plan(self):
        for _ in range(self.max_iter):
            rand = self.random_node()
            nearest = self.nearest(rand)
            new = self.steer(nearest, rand)
            new.parent = nearest
            new.cost = nearest.cost + self.distance(nearest, new)

            if not self.collision_free(nearest, new):
                continue

            # RRT* rewiring
            neighbors = self.near_nodes(new)
            best_parent = nearest
            best_cost = new.cost

            for nb in neighbors:
                cost = nb.cost + self.distance(nb, new)
                if cost < best_cost and self.collision_free(nb, new):
                    best_parent = nb
                    best_cost = cost

            new.parent = best_parent
            new.cost = best_cost
            self.nodes.append(new)

            # Rewire neighbors
            for nb in neighbors:
                cost = new.cost + self.distance(new, nb)
                if cost < nb.cost and self.collision_free(new, nb):
                    nb.parent = new
                    nb.cost = cost

            if self.distance(new, self.goal) < self.step:
                self.goal.parent = new
                self.goal.cost = new.cost + self.distance(new, self.goal)
                self.nodes.append(self.goal)
                print(f"Path found! Cost: {self.goal.cost:.2f}")
                return self.extract_path()

        print("No path found")
        return None

    def extract_path(self):
        path = []
        node = self.goal
        while node is not None:
            path.append((node.x, node.y, node.z))
            node = node.parent
        return list(reversed(path))


if __name__ == "__main__":
    # Obstacles matching walls world (x, y, z_center, size_x, size_y, size_z)
    obstacles = [
        Obstacle(5,  0,  2.5, 1, 10, 5),
        Obstacle(-5, 0,  2.5, 1, 10, 5),
        Obstacle(0,  5,  2.5, 10, 1, 5),
        Obstacle(0, -5,  2.5, 10, 1, 5),
    ]

    start = (0, 0, 5)
    goal  = (10, 10, 15)
    bounds = (-20, 20, -20, 20, 0, 30)

    planner = RRTStar(start, goal, obstacles, bounds)
    path = planner.plan()

    if path:
        print(f"Path has {len(path)} waypoints:")
        for i, (x, y, z) in enumerate(path):
            print(f"  WP{i+1}: x={x:.1f}, y={y:.1f}, z={z:.1f}")
