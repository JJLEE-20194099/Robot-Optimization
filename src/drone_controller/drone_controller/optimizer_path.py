import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Path
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
import numpy as np
from scipy.linalg import expm, logm

class LiePathPublisher(Node):
    def __init__(self):
        super().__init__('lie_path_publisher')
        self.publisher_ = self.create_publisher(Path, 'drone_path', 10)

        self.tf_static_broadcaster = StaticTransformBroadcaster(self)
        self.publish_static_tf()

        self.timer = self.create_timer(0.1, self.calculate_and_pub)
        
        self.start_location = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.target_location = [5.0, 5.0, 6.0, 0.1, 0.1, 0.5] 
        def get_se3(loc):
            Xi_hat = np.zeros((4, 4))
            v = loc[0:3]
            w = loc[3:6]
           
            Xi_hat[:3, :3] = np.array([[0, -w[2], w[1]], 
                                      [w[2], 0, -w[0]], 
                                      [-w[1], w[0], 0]])
            Xi_hat[:3, 3] = v
            return expm(Xi_hat)

        self.T_curr = get_se3(self.start_location)
        self.T_target = get_se3(self.target_location)
        
        self.path_msg = Path()
        self.path_msg.header.frame_id = "map"

    def publish_static_tf(self):
      
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'map' 
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation.w = 1.0
        self.tf_static_broadcaster.sendTransform(t)

    def calculate_and_pub(self):
       
        error_matrix = np.linalg.inv(self.T_curr) @ self.T_target
        xi_error = logm(error_matrix)
        
        if np.linalg.norm(xi_error) > 1e-6:
            self.T_curr = self.T_curr @ expm(0.05 * xi_error) 
            
            now = self.get_clock().now().to_msg()
            self.path_msg.header.stamp = now
            
            pose = PoseStamped()
            pose.header.stamp = now
            pose.header.frame_id = "map"
            
            pose.pose.position.x = float(self.T_curr[0, 3])
            pose.pose.position.y = float(self.T_curr[1, 3])
            pose.pose.position.z = float(self.T_curr[2, 3])
            pose.pose.orientation.w = 1.0
            
            self.path_msg.poses.append(pose)
            
            if len(self.path_msg.poses) > 100:
                self.path_msg.poses.pop(0)
                
            self.publisher_.publish(self.path_msg)

def main(args=None):
    rclpy.init(args=args)
    node = LiePathPublisher()
    print("--- NODE STARTED SUCCESSFULLY ---")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

