import rclpy
from rclpy.node import Node
import numpy as np
import random
from scipy.linalg import expm, logm
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Point, Vector3
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA, Header
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from scipy.spatial.transform import Rotation
from scipy.spatial.transform import Slerp
import cv2
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from rclpy.qos import QoSProfile

def skew(w):
    return np.array([[0, -w[2], w[1]], [w[2], 0, -w[0]], [-w[1], w[0], 0]])

def exp_se3(xi):
    Xi = np.zeros((4, 4))
    Xi[:3, :3] = skew(xi[3:])
    Xi[:3, 3] = xi[:3]
    return expm(Xi)

def log_se3(T):
    Xi = logm(T)
    xi = np.zeros(6)
    xi[:3] = Xi[:3, 3]
    xi[3:] = np.array([Xi[2, 1], Xi[0, 2], Xi[1, 0]])
    return xi

class RRTNode:
    def __init__(self, T, velocity):
        self.T = T
        self.vel = velocity
        self.parent = None
        self.cost = 0.0

class DroneOptimizer(Node):
    def __init__(self):
        super().__init__('drone_optimizer')
        
        self.rrt_path_pub = self.create_publisher(Path, '/rrt_drone_path', 10)
        self.spline_path_pub = self.create_publisher(Path, '/spline_drone_path', 10)
        self.obstacle_pub = self.create_publisher(MarkerArray, '/obstacles', 10)
        self.tree_pub = self.create_publisher(Marker, '/rrt_tree', 10)
        self.img_pub = self.create_publisher(Image, '/camera/image_raw', 10)
        self.info_pub = self.create_publisher(CameraInfo, '/camera/camera_info', 10)
        self.sg_pub = self.create_publisher(MarkerArray, '/start_goal', 10)
        self.drone_marker_pub = self.create_publisher(Marker, '/drone_marker', 10)
        self.ray_pub = self.create_publisher(Marker, '/normal_ray', 10)

        self.tf_broadcaster = TransformBroadcaster(self)

        self.path = []       
        self.path_index = 0
        self.current_p = None

        # self.publish_static_tf()

        self.bridge = CvBridge()
        self.start_pos = np.array([0.0, 0.0, 0.0])
        
        self.start_T = np.eye(4)
        self.goal_pos = np.array([10.0, 10.0, 8.0])
        self.obstacles = [
            {"center": np.array([4.0, 4.0, 4.0]), "r": 0.8},
            {"center": np.array([2.0, 6.0, 2.0]), "r": 1.4},
            {"center": np.array([6.0, 3.0, 7.0]), "r": 1.0}
        ]

        self.start_vel = np.zeros(6)
        self.target_T = np.eye(4)
        self.target_T[:3,:3] = expm(skew(np.array([0, 0, np.pi/2])))

        self.dt = 0.2
        self.max_iters = 1000
        self.search_radius = 2.0
        self.run_optimization()

        self.follow_path_step()

        self.create_timer(0.05, self.follow_path_step)

        self.get_logger().info("Drone Optimizer Node has started!")

    def generate_se3_bspline(self, path_nodes, degree=3, num_samples=400):
        if len(path_nodes) < degree + 1:
            return [n.T for n in path_nodes]

        control_points = []
        for node in path_nodes:
            control_points.append(log_se3(node.T))
        
        control_points = np.array(control_points)
        n = len(control_points)
        
        knots = np.concatenate(([0] * degree, np.arange(n - degree + 1), [n - degree] * degree))

        def de_boor_6d(t, k, knots, d_points):
            idx = np.searchsorted(knots, t, side='right') - 1
            idx = min(max(idx, k), n - 1)
            
            d = [d_points[j].copy() for j in range(idx - k, idx + 1)]
            
            for r in range(1, k + 1):
                for j in range(k, r - 1, -1):
                    denom = knots[idx + 1 + j - r] - knots[idx - k + j]
                    alpha = (t - knots[idx - k + j]) / denom if denom != 0 else 0
                    d[j] = (1.0 - alpha) * d[j-1] + alpha * d[j]
            return d[k]

        t_values = np.linspace(0, knots[-1], num_samples)
        smooth_nodes = []
        last_node = None
        for t in t_values:
            xi_interp = de_boor_6d(t, degree, knots, control_points)
            T_smooth = exp_se3(xi_interp)
            new_node = RRTNode(T_smooth, np.zeros(6)) 
            new_node.parent = last_node
            
            smooth_nodes.append(new_node)
            last_node = new_node
            
        return smooth_nodes

    def publish_drone_marker(self, position, now_sync):
        marker = Marker()
        marker.header.frame_id = "world"
        marker.header.stamp = now_sync

        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.id = 999

        marker.pose.position.x = float(position[0])
        marker.pose.position.y = float(position[1])
        marker.pose.position.z = float(position[2])

        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2

        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        self.drone_marker_pub.publish(marker)


    def publish_static_tf(self):
        transforms = []

        t2 = TransformStamped()
        t2.header.stamp = self.get_clock().now().to_msg()
        t2.header.frame_id = 'base_link'
        t2.child_frame_id = 'camera_link'

        r = Rotation.from_euler('xyz', [0, 0, 0])
        q = r.as_quat()

        t2.transform.rotation.x = q[0]
        t2.transform.rotation.y = q[1]
        t2.transform.rotation.z = q[2]
        t2.transform.rotation.w = q[3]

        transforms.append(t2)

        self.static_broadcaster.sendTransform(transforms)

    
    def publish_start_goal(self):
        marker_array = MarkerArray()

        start_marker = Marker()
        start_marker.header.frame_id = "world"
        start_marker.header.stamp = self.get_clock().now().to_msg()
        start_marker.id = 0
        start_marker.type = Marker.SPHERE
        start_marker.action = Marker.ADD

        start_marker.pose.position.x = float(self.start_pos[0])
        start_marker.pose.position.y = float(self.start_pos[1])
        start_marker.pose.position.z = float(self.start_pos[2])

        start_marker.scale = Vector3(x=0.3, y=0.3, z=0.3)
        start_marker.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0)

        marker_array.markers.append(start_marker)

        goal_marker = Marker()
        goal_marker.header.frame_id = "world"
        goal_marker.header.stamp = self.get_clock().now().to_msg()
        goal_marker.id = 1
        goal_marker.type = Marker.SPHERE
        goal_marker.action = Marker.ADD

        goal_marker.pose.position.x = float(self.goal_pos[0])
        goal_marker.pose.position.y = float(self.goal_pos[1])
        goal_marker.pose.position.z = float(self.goal_pos[2])

        goal_marker.scale = Vector3(x=0.4, y=0.4, z=0.4)
        goal_marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)
        
        marker_array.markers.append(goal_marker)

        start_text = Marker()
        start_text.header.frame_id = "world"
        start_text.id = 2
        start_text.type = Marker.TEXT_VIEW_FACING
        start_text.text = "START"
        start_text.pose.position.x = float(self.start_pos[0])
        start_text.pose.position.y = float(self.start_pos[1])
        start_text.pose.position.z = float(self.start_pos[2] + 0.5)
        start_text.scale.z = 0.4
        start_text.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0)

        marker_array.markers.append(start_text)

        goal_text = Marker()
        goal_text.header.frame_id = "world"
        goal_text.id = 3
        goal_text.type = Marker.TEXT_VIEW_FACING
        goal_text.text = "GOAL"
        goal_text.pose.position.x = float(self.goal_pos[0])
        goal_text.pose.position.y = float(self.goal_pos[1])
        goal_text.pose.position.z = float(self.goal_pos[2] + 0.5)
        goal_text.scale.z = 0.4
        goal_text.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)

        marker_array.markers.append(goal_text)

        self.sg_pub.publish(marker_array)



    def collision_free(self, p1, p2):
        steps = 15 
        for i in range(steps + 1):
            t = i / steps
            p = (1 - t) * p1 + t * p2
            for obs in self.obstacles:
                if np.linalg.norm(p - obs["center"]) < (obs["r"] + 0.2):
                    return False
        return True
    
    def get_pos(self, T):
        return T[:3, 3]

    def orientation_distance(self, T1, T2):
        R1 = T1[:3, :3]
        R2 = T2[:3, :3]

        R_err = R1.T @ R2
        trace_val = np.clip((np.trace(R_err) - 1) / 2, -1.0, 1.0)

        return np.arccos(trace_val)

    def distance(self, n1, target_pos):
        pos_dist = np.linalg.norm(self.get_pos(n1.T) - target_pos)

        ori_dist = self.orientation_distance(n1.T, self.target_T)

        return pos_dist + 0.3 * ori_dist
    
    def simulate_kinodynamics(self, node, u):
        new_vel = node.vel + u * self.dt
        new_vel = np.clip(new_vel, -1.5, 1.5)

        T_new = node.T @ exp_se3(new_vel * self.dt)
        return T_new, new_vel

    def run_optimization(self):

        nodes = [RRTNode(self.start_T, self.start_vel)]

        print("Starting RRT* optimization...")

        for i in range(self.max_iters):

            z_rand = np.array([random.uniform(0, 10) for _ in range(3)])
            
            n_near = min(nodes, key=lambda n: self.distance(n, z_rand))
            
            best_control = None
            min_dist = float('inf')
            temp_T, temp_vel = None, None

            for _ in range(30): 
                u = np.random.uniform(-1.0, 1.0, 6)

                T_next, v_next = self.simulate_kinodynamics(n_near, u)
                
                d = np.linalg.norm(self.get_pos(T_next) - z_rand)

                if d < min_dist and self.collision_free(self.get_pos(n_near.T), self.get_pos(T_next)):
                    min_dist = d
                    temp_T, temp_vel = T_next, v_next

            T_next, v_next = temp_T, temp_vel

            if T_next is None:
                best_u = np.random.uniform(-1.0, 1.0, 6)
                T_next, v_next = self.simulate_kinodynamics(n_near, best_u)

            if not self.collision_free(self.get_pos(n_near.T), self.get_pos(T_next)):
                continue

            new_node = RRTNode(T_next, v_next)
            new_node.parent = n_near
            new_node.cost = n_near.cost + np.linalg.norm(self.get_pos(T_next) - self.get_pos(n_near.T))

            near_inds = [j for j, n in enumerate(nodes)
                        if self.distance(n, self.get_pos(new_node.T)) < self.search_radius]

            for idx in near_inds:
                n_neighbor = nodes[idx]

                new_cost = n_neighbor.cost + np.linalg.norm(self.get_pos(new_node.T) - self.get_pos(n_neighbor.T))
                
                if new_cost < new_node.cost:
                    if self.collision_free(self.get_pos(n_neighbor.T), self.get_pos(new_node.T)):
                        new_node.parent = n_neighbor
                        new_node.cost = new_cost

            nodes.append(new_node)

            for idx in near_inds:
                n_neighbor = nodes[idx]

                improved_cost = new_node.cost + np.linalg.norm(self.get_pos(n_neighbor.T) - self.get_pos(new_node.T))
                
                if improved_cost < n_neighbor.cost:
                    if self.collision_free(self.get_pos(new_node.T), self.get_pos(n_neighbor.T)):
                        n_neighbor.parent = new_node
                        n_neighbor.cost = improved_cost

            if np.linalg.norm(self.get_pos(new_node.T) - self.goal_pos) < 0.8:
                print(f"Goal Reached at iteration {i}!")
                break

        smooth_path = []
        final_node = min(nodes, key=lambda n: np.linalg.norm(self.get_pos(n.T) - self.goal_pos))
        smooth_path = []
        curr = final_node
        while curr:
            smooth_path.append(curr)
            curr = curr.parent

        smooth_path = smooth_path[::-1]

        dense_nodes = []
        last_node = None
        for i in range(len(smooth_path) - 1):
            n1 = smooth_path[i]
            n2 = smooth_path[i+1]
            if last_node: n1.parent = last_node
            dense_nodes.append(n1)

            r1 = Rotation.from_matrix(n1.T[:3, :3])
            r2 = Rotation.from_matrix(n2.T[:3, :3])

            key_times = [0, 1]
            key_rots = Rotation.from_matrix([n1.T[:3, :3], n2.T[:3, :3]])
            slerp = Slerp(key_times, key_rots)
            
            num_sub_points = 5
            for j in range(1, num_sub_points):
                frac = j / num_sub_points
                interp_T = np.eye(4)
                interp_T[:3, 3] = (1 - frac) * n1.T[:3, 3] + frac * n2.T[:3, 3]
                interp_rot = slerp([frac])[0]
                interp_T[:3, :3] = interp_rot.as_matrix()
                
                mid_node = RRTNode(interp_T, np.zeros(6))
                mid_node.parent = last_node
                dense_nodes.append(mid_node)
                last_node = mid_node
        
        dense_nodes.append(smooth_path[-1])
        
        self.path = self.generate_se3_bspline(dense_nodes)
        self.path_index = 0

        print(f"RRT* optimization completed with {len(nodes)} nodes. Path length: {len(self.path)}")

        self.publish_path(smooth_path, "rrt")
        self.publish_path(self.path, "spline")
        self.publish_obstacles()
        self.publish_tree(nodes)
        self.publish_start_goal()

    def publish_camera_data(self, timestamp):
        # Tạo ảnh giả lập (đen với hình tròn đỏ)
        img = np.zeros((480, 640, 3), dtype=np.uint8) # Khớp với info_msg bên dưới
        cv2.circle(img, (320, 240), 50, (0, 0, 255), -1) 
        
        img_msg = self.bridge.cv2_to_imgmsg(img, encoding="bgr8")
        # QUAN TRỌNG: Dùng chung timestamp với drone_marker và TF
        img_msg.header.stamp = timestamp
        img_msg.header.frame_id = "camera_link" 
        
        info_msg = CameraInfo()
        info_msg.header = img_msg.header 
        info_msg.width = 640
        info_msg.height = 480
        
        # Ma trận camera (Intrinsic matrix)
        info_msg.k = [500.0, 0.0, 320.0, 0.0, 500.0, 240.0, 0.0, 0.0, 1.0]
        info_msg.distortion_model = "plumb_bob"
        info_msg.d = [0.0, 0.0, 0.0, 0.0, 0.0] 
        info_msg.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info_msg.p = [500.0, 0.0, 320.0, 0.0, 0.0, 500.0, 240.0, 0.0, 0.0, 0.0, 1.0, 0.0]

        self.img_pub.publish(img_msg)
        self.info_pub.publish(info_msg)

    def publish_normal_ray(self, drone_pos, next_point):
        marker = Marker()
        marker.header.frame_id = "world"
        marker.header.stamp = self.get_clock().now().to_msg()

        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.id = 1000

        marker.scale.x = 0.2 

        marker.color.r = 0.85
        marker.color.g = 0.5
        marker.color.b = 0.0
        marker.color.a = 1.0

        direction = next_point - drone_pos
        norm = np.linalg.norm(direction)

        if norm < 1e-6:
            return

        direction = direction / norm  

        length = 1.5

        p_start = Point(
            x=float(drone_pos[0]),
            y=float(drone_pos[1]),
            z=float(drone_pos[2])
        )

        p_end = Point(
            x=float(drone_pos[0] + direction[0]*length),
            y=float(drone_pos[1] + direction[1]*length),
            z=float(drone_pos[2] + direction[2]*length)
        )

        marker.points.append(p_start)
        marker.points.append(p_end)

        self.ray_pub.publish(marker)

    def follow_path_step(self):
       
        if not hasattr(self, 'current_drone_pos'):
            self.current_drone_pos = np.array([0.0, 0.0, 0.0])

        now = self.get_clock().now().to_msg()

        p = self.current_p if self.current_p is not None else self.current_drone_pos
        q = [0, 0, 0, 1]
        
        p2 = None
        if hasattr(self, 'path') and len(self.path) >= 2 and self.path_index < len(self.path) - 1:
            n1 = self.path[self.path_index]
            n2 = self.path[self.path_index + 1]

            p1 = self.get_pos(n1.T)
            p2 = self.get_pos(n2.T)

            if self.current_p is None:
                self.current_p = p1

            direction = p2 - self.current_p
            dist = np.linalg.norm(direction)

            step_size = 0.1
            if dist > step_size:
                self.current_p = self.current_p + (direction / dist) * step_size
            else:
                self.current_p = p2
                self.path_index += 1

    

            if dist > 1e-3:
                direction = direction / (np.linalg.norm(direction) + 1e-6)

                x_axis = direction
                z_axis = np.array([0, 0, 1])
                y_axis = np.cross(z_axis, x_axis)
                y_axis /= (np.linalg.norm(y_axis) + 1e-6)

                z_axis = np.cross(x_axis, y_axis)
                R_mat = np.stack([x_axis, y_axis, z_axis], axis=1)

                q = Rotation.from_matrix(R_mat).as_quat()

                rot = Rotation.from_matrix(R_mat)
                roll, pitch, yaw = rot.as_euler('xyz', degrees=True)

                self.get_logger().info(
                    f"Yaw: {yaw:.2f}° | Pitch: {pitch:.2f}° | Roll: {roll:.2f}°"
                )

            else:
                q = [0, 0, 0, 1]

        

        center = np.array([0.0, 0.0, 0.0])
        if p2 is not None:
            self.publish_normal_ray(self.current_p, p2)
        t = TransformStamped()
        t.header.frame_id = "world"
        t.child_frame_id = "base_link"
        t.header.stamp = now

        t.transform.translation.x = float(p[0])
        t.transform.translation.y = float(p[1])
        t.transform.translation.z = float(p[2])

        t.transform.rotation.x = float(q[0])
        t.transform.rotation.y = float(q[1])
        t.transform.rotation.z = float(q[2])
        t.transform.rotation.w = float(q[3])

        self.tf_broadcaster.sendTransform(t)

        self.publish_drone_marker(self.current_p, now)
        self.publish_camera_data(now)

    def publish_path(self, nodes, path_type = "rrt"):
        final_node = min(nodes, key=lambda n: np.linalg.norm(n.T[:3, 3] - self.goal_pos))
        msg = Path()
        msg.header.frame_id = "world"
        msg.header.stamp = self.get_clock().now().to_msg()
        
        curr = final_node
        while curr:
            pose = PoseStamped()
            pose.header.frame_id = "world"
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = curr.T[0, 3]
            pose.pose.position.y = curr.T[1, 3]
            pose.pose.position.z = curr.T[2, 3]
            msg.poses.append(pose)
            curr = curr.parent
        
        msg.poses.reverse()
        if path_type == "rrt":
            self.rrt_path_pub.publish(msg)
        else:
            self.spline_path_pub.publish(msg)

    def publish_obstacles(self):
        marker_array = MarkerArray()
        for i, obs in enumerate(self.obstacles):
            marker = Marker()
            marker.header.frame_id = "world"
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = float(obs["center"][0])
            marker.pose.position.y = float(obs["center"][1])
            marker.pose.position.z = float(obs["center"][2])
            marker.scale = Vector3(x=obs["r"]*2, y=obs["r"]*2, z=obs["r"]*2)
            marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.3)
            marker_array.markers.append(marker)
        self.obstacle_pub.publish(marker_array)

    def publish_tree(self, nodes):
        marker = Marker()
        marker.header.frame_id = "world"
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD
        marker.scale.x = 0.02
        marker.color = ColorRGBA(r=0.5, g=0.5, b=0.5, a=0.2)
        
        for n in nodes:
            if n.parent:
                p1 = Point(x=n.parent.T[0,3], y=n.parent.T[1,3], z=n.parent.T[2,3])
                p2 = Point(x=n.T[0,3], y=n.T[1,3], z=n.T[2,3])
                marker.points.append(p1)
                marker.points.append(p2)
        self.tree_pub.publish(marker)

def main():
    rclpy.init()
    node = DroneOptimizer()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

