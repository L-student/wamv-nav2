import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

class Bringup(Node):
    def __init__(self):
        super().__init__('vrx_nav2_bringup')

        self.pub_left_pos = self.create_publisher(Float64, '/wamv/thrusters/left/pos', 10)
        self.pub_left_thrust = self.create_publisher(Float64, '/wamv/thrusters/left/thrust', 10)
        self.pub_right_pos = self.create_publisher(Float64, '/wamv/thrusters/right/pos', 10)
        self.pub_right_thrust = self.create_publisher(Float64, '/wamv/thrusters/right/thrust', 10)

        self.cmd_vel_subscription = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_listener, 10)
        self.cmd_vel_subscription # Prevent unused variable warning

        self.pos = 0

        #self.set_parameter(rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True))

    def cmd_vel_listener(self, msg):
        vx = msg.linear.x
        wz = msg.angular.z

        left_thrust = Float64()
        right_thrust = Float64()
        left_pos = Float64()
        right_pos = Float64()

        left_thrust.data = vx
        right_thrust.data = vx

        left_pos.data = self.pos + wz
        right_pos.data = self.pos + wz

        self.pos = left_pos.data

        self.pub_left_pos.publish(left_pos)
        self.pub_right_pos.publish(right_pos)
        self.pub_left_thrust.publish(left_thrust)
        self.pub_right_thrust.publish(right_thrust)


def main(args=None):
    rclpy.init(args=args)


    bringup = Bringup()

    rclpy.spin(bringup)

    bringup.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()