import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import numpy as np
from scipy import interpolate

class PathSmootherNode(Node):
    def __init__(self):
        super().__init__('path_smoother_node')
        self.publisher = self.create_publisher(Marker, 'robot_paths', 10)
        self.timer = self.create_timer(1.0, self.publish_paths)

        n_points = 100
        x = np.linspace(0, 50, n_points) 
        y = 5 * np.sin(x * 0.5) + np.random.uniform(-1.5, 1.5, n_points)
        self.rrt_nodes = np.vstack((x, y)).T
    
    def generate_bspline(self, points, degree = 3, num_samples = 300):
        n = len(points)
        if n <= degree:
            degree = n - 1
        
        tck, u = interpolate.splprep([points[:, 0], points[:, 1]], s = 0, k = degree)

        u_fine = np.linspace(0, 1, num_samples)
        new_points = interpolate.splev(u_fine, tck)

        return np.array(new_points).T
    
    def create_marker(self, points, color, m_id, label):
        marker = Marker()
        marker.header.frame_id = "world"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = label
        marker.id = m_id
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD

        marker.scale.x = 0.1 if label == "RRT" else 0.15
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = color

        for p in points:
            point = Point()
            point.x = float(p[0])
            point.y = float(p[1])
            point.z = 0.0
            marker.points.append(point)
        
        return marker
    
    def de_boor(self, k, knots, t, points):
      
        n = len(points)
        
        idx = 0
        for i in range(len(knots) - 1):
            if knots[i] <= t < knots[i+1]:
                idx = i
                break
        if t == knots[-1]: idx = n - 1

       
        active_points = []
        for j in range(idx - k, idx + 1):
            active_points.append(np.array(points[j]))

        d = active_points
        for r in range(1, k + 1):
            for j in range(k, r - 1, -1):
                alpha = (t - knots[idx - k + j]) / (knots[idx + 1 + j - r] - knots[idx - k + j])
                d[j] = (1.0 - alpha) * d[j-1] + alpha * d[j]
        
        return d[k]


    
    def generate_custome_bspline(self, points, degree = 4, num_samples = 300):
        n = len(points)
        k = degree
        knots = np.concatenate([
            np.zeros(k),
            np.linspace(0, 1, n - k + 1),
            np.ones(k)
        ])

        paths = []

        for t in np.linspace(0, 1, num_samples):
            if t == 1.0: t = 0.999999
            point = self.de_boor(k, knots, t, points)
            paths.append(point)
        
        
        return np.array(paths)



    

    def publish_paths(self):
        rrt_marker = self.create_marker(self.rrt_nodes, (1.0, 0.0, 0.0, 1.0), 0, "RRT")
        self.publisher.publish(rrt_marker)

        # bspline_points = self.generate_bspline(self.rrt_nodes)
        # bspline_marker =  self.create_marker(bspline_points, (0.0, 1.0, 0.0, 1.0), 1, "BSpline")
        # self.publisher.publish(bspline_marker)

        bspline_custom_points = self.generate_custome_bspline(self.rrt_nodes)
        bspline_custom_marker =  self.create_marker(bspline_custom_points, (1.0, 1.0, 0.0, 1.0), 1, "Custom BSpline")
        self.publisher.publish(bspline_custom_marker)
        

        self.get_logger().info("Publish paths")


    

def main(args = None):
    rclpy.init(args = args)
    node = PathSmootherNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
