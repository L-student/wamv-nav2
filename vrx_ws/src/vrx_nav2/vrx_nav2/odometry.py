import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped, Quaternion, Twist
from tf2_ros import TransformBroadcaster
from math import cos, sin, atan2
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
import numpy as np

class Odometry_Node(Node):
    def __init__(self):
        super().__init__('vrx_nav2_bringup')

        self.pub_odometry = self.create_publisher(Odometry, '/odom', 10)
        self.vz_broadcaster = TransformBroadcaster(self)

        self.imu_subscription = self.create_subscription(Imu, '/wamv/sensors/imu/imu/data', self.odometry_listener, 10)
        self.imu_subscription # Prevent unused variable warning

        
      
        self.last_time = self.get_clock().now()

        self.theta = 0
        self.vx = 0
        self.vy = 0
        self.vz = 0

        # Assumindo que o robô vai começar nas coordenadas do odom
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0

        
    def odometry_listener(self, msg):
        current_time = self.get_clock().now() 
        
        dt = (current_time - self.last_time).nanoseconds / 1e9

        ax = msg.linear_acceleration.x
        ay = msg.linear_acceleration.y
        az = msg.linear_acceleration.z

        wx = msg.angular_velocity.x
        wy = msg.angular_velocity.y
        wz = msg.angular_velocity.z

        # Convertendo a orientação do IMU para ângulo de Euler
        self.theta = atan2(2.0 * (msg.orientation.w * msg.orientation.z + msg.orientation.x * msg.orientation.y),
                           1.0 - 2.0 * (msg.orientation.y * msg.orientation.y + msg.orientation.z * msg.orientation.z))

        self.vx += ax * dt
        self.vy += ay * dt
        self.vz += az * dt

        # Integração das velocidades para obter a posição
        self.x += self.vx * cos(self.theta) * dt
        self.y += self.vx * sin(self.theta) * dt
        self.z += self.vz * dt

        self.last_time = current_time

     

        # Publicar transformações
        transform1 = TransformStamped()
        transform1.header.stamp = current_time.to_msg()
        transform1.header.frame_id = 'odom'
        transform1.child_frame_id = 'base_link'
        transform1.transform.translation.x = self.x
        transform1.transform.translation.y = self.y
        transform1.transform.translation.z = self.z
        transform1.transform.rotation = msg.orientation
        self.vz_broadcaster.sendTransform(transform1)

        transform2 = TransformStamped()
        transform2.header.stamp = current_time.to_msg()
        transform2.header.frame_id = 'base_link'
        transform2.child_frame_id = 'wamv/wamv/base_link'
        transform2.transform.translation.x = 0.0
        transform2.transform.translation.y = 0.0
        transform2.transform.translation.z = 0.0
        transform2.transform.rotation.x = 0.0
        transform2.transform.rotation.y = 0.0
        transform2.transform.rotation.z = 0.0
        transform2.transform.rotation.w = 1.0
        self.vz_broadcaster.sendTransform(transform2)



        transform3 = TransformStamped()
        transform3.header.stamp = current_time.to_msg()
        transform3.header.frame_id = 'map'
        transform3.child_frame_id = 'odom'
        transform3.transform.translation.x = 0.0
        transform3.transform.translation.y = 0.0
        transform3.transform.translation.z = 0.0
        transform3.transform.rotation.x = 0.0
        transform3.transform.rotation.y = 0.0
        transform3.transform.rotation.z = 0.0
        transform3.transform.rotation.w = 1.0
        self.vz_broadcaster.sendTransform(transform3)

        # Publicar odometria
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = 'odom'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = self.z
        odom.pose.pose.orientation = msg.orientation
        odom.child_frame_id = 'base_link'
        odom.twist.twist.linear.x = self.vx
        odom.twist.twist.linear.y = self.vy
        odom.twist.twist.angular.z = wz
        self.pub_odometry.publish(odom)

        

    @staticmethod
    def create_quaternion_from_yaw(yaw):
        return Quaternion(w=cos(yaw / 2), x=0, y=0, z=sin(yaw / 2))

def main(args=None):
    rclpy.init(args=args)

    odometry = Odometry_Node()

    rclpy.spin(odometry)

    odometry.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()