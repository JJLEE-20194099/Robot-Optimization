import numpy as np
import random
import matplotlib.pyplot as plt
from scipy.linalg import expm

def skew(w):
    return np.array([[0, -w[2], w[1]],
                     [w[2], 0, -w[0]],
                     [-w[1], w[0], 0]])

def exp_se3(xi):
    """Chuyển twist xi (6x1) sang ma trận SE(3) (4x4)"""
    Xi = np.zeros((4, 4))
    Xi[:3, :3] = skew(xi[3:])
    Xi[:3, 3] = xi[:3]
    return expm(Xi)

class Node:
    def __init__(self, T, velocity):
        self.T = T
        self.vel = velocity
        self.parent = None
        self.cost = 0.0

start_T = np.eye(4)
start_vel = np.zeros(6)

goal_pos = np.array([8, 8, 8])

target_T = np.eye(4)
target_T[:3,:3] = expm(skew(np.array([0, 0, np.pi/2])))

dt = 0.2
max_iters = 5000
search_radius = 2.0

obstacles = [
    {"center": np.array([4, 4, 4]), "r": 6},
    {"center": np.array([2, 6, 2]), "r": 8},
    {"center": np.array([6, 3, 7]), "r": 10}
]

def get_pos(T):
    return T[:3, 3]

def orientation_distance(T1, T2):
    R1 = T1[:3, :3]
    R2 = T2[:3, :3]

    R_err = R1.T @ R2
    trace_val = np.clip((np.trace(R_err) - 1) / 2, -1.0, 1.0)

    return np.arccos(trace_val)

def distance(n1, target_pos):
    pos_dist = np.linalg.norm(get_pos(n1.T) - target_pos)


    ori_dist = orientation_distance(n1.T, target_T)


    return pos_dist + 0.3 * ori_dist

def collision_free(p1, p2):
    steps = 15 
    for i in range(steps + 1):
        t = i / steps
        p = (1 - t) * p1 + t * p2
        for obs in obstacles:
            if np.linalg.norm(p - obs["center"]) < (obs["r"] + 0.2):
                return False
    return True

def simulate_kinodynamics(node, u):
    new_vel = node.vel + u * dt
    new_vel = np.clip(new_vel, -1.5, 1.5)

    T_new = node.T @ exp_se3(new_vel * dt)
    return T_new, new_vel

nodes = [Node(start_T, start_vel)]

for i in range(max_iters):

    z_rand = np.array([random.uniform(0, 10) for _ in range(3)])
    
    n_near = min(nodes, key=lambda n: distance(n, z_rand))
    
    best_control = None
    min_dist = float('inf')
    temp_T, temp_vel = None, None

    for _ in range(30): 
        u = np.random.uniform(-1.0, 1.0, 6)

        T_next, v_next = simulate_kinodynamics(n_near, u)
        
        d = np.linalg.norm(get_pos(T_next) - z_rand)

        if d < min_dist and collision_free(get_pos(n_near.T), get_pos(T_next)):
            min_dist = d
            temp_T, temp_vel = T_next, v_next

    T_next, v_next = temp_T, temp_vel

    if T_next is None:
        best_u = np.random.uniform(-1.0, 1.0, 6)
        T_next, v_next = simulate_kinodynamics(n_near, best_u)

    if not collision_free(get_pos(n_near.T), get_pos(T_next)):
        continue

    new_node = Node(T_next, v_next)
    new_node.parent = n_near
    new_node.cost = n_near.cost + np.linalg.norm(get_pos(T_next) - get_pos(n_near.T))

    near_inds = [j for j, n in enumerate(nodes)
                 if distance(n, get_pos(new_node.T)) < search_radius]

    for idx in near_inds:
        n_neighbor = nodes[idx]

        new_cost = n_neighbor.cost + np.linalg.norm(get_pos(new_node.T) - get_pos(n_neighbor.T))
        
        if new_cost < new_node.cost:
            if collision_free(get_pos(n_neighbor.T), get_pos(new_node.T)):
                new_node.parent = n_neighbor
                new_node.cost = new_cost

    nodes.append(new_node)

    for idx in near_inds:
        n_neighbor = nodes[idx]

        improved_cost = new_node.cost + np.linalg.norm(get_pos(n_neighbor.T) - get_pos(new_node.T))
        
        if improved_cost < n_neighbor.cost:
            if collision_free(get_pos(new_node.T), get_pos(n_neighbor.T)):
                n_neighbor.parent = new_node
                n_neighbor.cost = improved_cost

    if np.linalg.norm(get_pos(new_node.T) - goal_pos) < 0.8:
        print(f"Goal Reached at iteration {i}!")
        break

fig = plt.figure(figsize=(12, 9))
ax = fig.add_subplot(111, projection='3d')

for node in nodes:
    if node.parent:
        p1 = get_pos(node.parent.T)
        p2 = get_pos(node.T)
        ax.plot([p1[0], p2[0]], [p1[1], p2[1]], [p1[2], p2[2]], 
                color='gray', linestyle='-', linewidth=0.5, alpha=0.3)

final_node = min(nodes, key=lambda n: np.linalg.norm(get_pos(n.T) - goal_pos))
path = []
curr = final_node
while curr:
    path.append(get_pos(curr.T))
    curr = curr.parent
path = np.array(path[::-1])

ax.plot(path[:,0], path[:,1], path[:,2], 'b-', linewidth=3, label='Optimal Path')

def draw_pose(ax, T, color, label):
    p = T[:3, 3]

    forward = T[:3, 0] * 0.8 
    ax.quiver(p[0], p[1], p[2], forward[0], forward[1], forward[2], 
              color=color, length=1.0, normalize=True, label=label)

draw_pose(ax, start_T, 'green', 'Start Orientation')
draw_pose(ax, target_T, 'red', 'Goal Orientation')

for obs in obstacles:
    u, v = np.mgrid[0:2*np.pi:20j, 0:np.pi:10j]
    x = obs["center"][0] + obs["r"] * np.cos(u) * np.sin(v)
    y = obs["center"][1] + obs["r"] * np.sin(u) * np.sin(v)
    z = obs["center"][2] + obs["r"] * np.cos(v)
    ax.plot_surface(x, y, z, color='red', alpha=0.1)

ax.scatter(0,0,0, c='g', s=100, label='Start')
ax.scatter(goal_pos[0], goal_pos[1], goal_pos[2], c='r', s=100, label='Goal')

ax.set_title(f"Kinodynamic RRT* Tree (Iters: {max_iters}, Path Cost: {final_node.cost:.2f})")
ax.legend()
plt.show()