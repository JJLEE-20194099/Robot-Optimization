
import numpy as np
from scipy.linalg import expm, logm

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

"""
Input: 3D point of current/target location of drone
Output: Draw the path from current location to target location.
"""

start_location = [0, 0, 0, 0, 0, 0,]
target_location = [2, 2, 2, 0, 0, np.pi / 2]

def get_skew_symmetric_matrix(rotate_vector):
    rotate_x = rotate_vector[0]
    rotate_y = rotate_vector[1]
    rotate_z = rotate_vector[2]

    skew_symmetrc_matrix = [
        [0, -rotate_z, rotate_y],
        [rotate_z, 0, -rotate_x],
        [-rotate_y, rotate_x, 0]
    ]
    return skew_symmetrc_matrix

def exp_se3(location):
    velocity_vector = location[0:3]
    rotate_vector = location[3:6]
    
    skew_symmetric_marix = get_skew_symmetric_matrix(rotate_vector)

    resp_matrix = np.zeros((4, 4))
    resp_matrix[:3, :3] = skew_symmetric_marix
    resp_matrix[:3, 3] = velocity_vector

    return expm(resp_matrix)

def log_se3(matrix):
    location = logm(matrix)
    velocity_vector = location[:3, 3]
    symmetric_matrix = location[:3, :3]
    rotate_vector = np.array([symmetric_matrix[2, 1], symmetric_matrix[0, 2], symmetric_matrix[1, 0]])

    return np.concatenate([velocity_vector, rotate_vector])

def cal_error_vector(T_current_matrix, T_target_matrix):

    error_matrix = np.linalg.inv(T_current_matrix) @ T_target_matrix
    error_vector = log_se3(error_matrix)
    return error_vector


threshold = 1e-6
max_iters = 200
alpha = 0.1

T_current_matrix = exp_se3(start_location)
T_target_matrix = exp_se3(target_location)

path_history = []
for i in range(max_iters):
    path_history.append(T_current_matrix[:3, 3].copy())
    error_vector = cal_error_vector(T_current_matrix, T_target_matrix)
    error_translate_vector = error_vector[:3]
    error_rorate_vector = error_vector[3:]

    # print("error_translate_vector:",error_translate_vector)
    # print("error_rorate_vector:", error_rorate_vector)

    error_norm_val = np.linalg.norm(error_vector)
    print(f"Step {i}: error norm value: {error_norm_val:.6f}")
    if error_norm_val  < threshold:
        print(f"Path optimization process converaged at step {i}")
        break
    
    delta_xi = alpha * error_vector
    
    T_diff = exp_se3(delta_xi)

    T_current_matrix = T_current_matrix @ T_diff

print("Optimized path process finished.")
    
    
path_history = np.array(path_history)

fig = plt.figure(figsize=(10, 8))
ax = fig.add_subplot(111, projection='3d')

ax.plot(path_history[:, 0], path_history[:, 1], path_history[:, 2], color='b', marker='o', markersize=2, label='Drone')

ax.scatter(path_history[0, 0], path_history[0, 1], path_history[0, 2], color='g', s=100, label='Start')
ax.scatter(path_history[-1, 0], path_history[-1, 1], path_history[-1, 2], color='r', s=100, label='Target')

ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.set_title('Path Optimization for Drone')
ax.legend()

plt.show()





