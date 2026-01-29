import numpy as np
import rclpy
import rclpy.node
import std_msgs.msg

class ThrustAllocator(rclpy.node.Node):
    def __init__(self):
        super().__init__("david")
        self.B = np.empty((3, 5))

        self.sub = self.create_subscription(
            std_msgs.msg.Float32MultiArray,
            'tau_cmd',
            self.tau_cmd_callback,
            10
        )

        self.pub_F = self.create_publisher(
            std_msgs.msg.Float32MultiArray,
            'F',
            10
        )

        self.pub_alpha = self.create_publisher(
            std_msgs.msg.Float32MultiArray,
            'alpha',
            10
        )

        self.setup_allocation_matrix()

    def tau_cmd_callback(self, msg):
        tau_cmd = msg.data
        F, alpha = self.allocate(tau_cmd)

        F_msg = std_msgs.msg.Float32MultiArray()
        F_msg.data = F

        alpha_msg = std_msgs.msg.Float32MultiArray()
        alpha_msg.data = alpha

        self.pub_F.publish(F_msg)
        self.pub_alpha.publish(alpha_msg)

    def setup_allocation_matrix(self):
        self.B = np.random.random((3, 5))

    def allocate(self, tau_cmd):
        f = np.linalg.pinv(self.B) @ tau_cmd
        f_1 = np.sqrt(f[0]**2 + f[1]**2)
        f_2 = np.sqrt(f[2]**2 + f[3]**2)
        f_3 = f[4]
        alpha_1 = np.arctan2(f[1], f[0])
        alpha_2 = np.arctan2(f[3], f[2])
        F = np.array([f_1, f_2, f_3], dtype='float')
        alpha = np.array([alpha_1, alpha_2], dtype='float')
        return F, alpha

def main(args=None):
    rclpy.init(args=args)

    node = ThrustAllocator()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
