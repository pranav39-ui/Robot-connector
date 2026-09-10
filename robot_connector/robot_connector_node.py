import rclpy
import serial
import time
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import Pose, Twist, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from sensor_msgs.msg import JointState

class RobotConnector(Node):

    def __init__(self):
        super().__init__('robot_connector')
        self.declare_parameter('serial_adress', '/dev/esp_motor')
        self.declare_parameter('robot_length', 0.525)
        self.declare_parameter('min_turning_radius', 0.5)

        self.init_msgs()

        self.lin = 0.0
        self.ang = 0.0

        self.last_joy_time = self.get_clock().now().nanoseconds
        self.last_nav_time = self.get_clock().now().nanoseconds

        self.sub_joy = self.create_subscription(Twist, 'cmd_vel_priority', self.sub_joy_callback, 10)
        self.sub_nav = self.create_subscription(Twist, 'cmd_vel', self.sub_nav_callback, 10)
        self.odom_publisher = self.create_publisher(Odometry, 'odom', 100)
        self.joint_publisher = self.create_publisher(JointState, 'joint_states', 10)

        self.tf_broadcaster = TransformBroadcaster(self)

        self.serial_adress = self.get_parameter('serial_adress').get_parameter_value().string_value
        self.L = self.get_parameter('robot_length').get_parameter_value().double_value
        self.min_turn_rad = self.get_parameter('min_turning_radius').get_parameter_value().double_value

        try:
            self.ser = serial.Serial(self.serial_adress, 115200, 8, serial.PARITY_EVEN, serial.STOPBITS_ONE, dsrdtr=False)
        except serial.SerialException as e:
            self.get_logger().info(f"[ERROR] Could not connect to {self.get_parameter('serial_adress').get_parameter_value().string_value} closing node")
            self.get_logger().debug(f"[ERROR] Could not connect to {self.get_parameter('serial_adress').get_parameter_value().string_value}: {e}")
            super().destroy_node()
        else:
            self.ser.flushInput()
            self.ser.write(bytes(b"RD;RST\n"))
            self.get_logger().info(f"[SUCCESS] Connected to {self.get_parameter('serial_adress').get_parameter_value().string_value}")
            time.sleep(2)
            self.ser.flushInput()
            self.timer = self.create_timer(0.1, self.publisher_callback)

    def sub_joy_callback(self, data):
        self.lin = data.linear.x
        self.ang = data.angular.z
        self.last_joy_time = self.get_clock().now().nanoseconds

    def sub_nav_callback(self, data):
        if self.last_joy_time <= self.get_clock().now().nanoseconds - 500 * 1000000: #0.5s
            self.lin = data.linear.x
            if abs(self.lin) <= 0.01:
                self.ang = 0.0
            else:
                self.ang = np.arctan((self.L * data.angular.z) / self.lin)
        self.last_nav_time = self.get_clock().now().nanoseconds
        

    def publisher_callback(self):
        line = self.ser.readline().decode("utf-8", "ignore")
        #self.get_logger().debug(line)
        if line.startswith("RD;") and len(line) == 52:
            self.msg_odom.header.stamp = self.get_clock().now().to_msg()

            self.msg_odom.twist.twist.linear.x = float(line[3:9]) - self.msg_odom.pose.pose.position.x
            self.msg_odom.twist.twist.linear.y = float(line[10:16]) - self.msg_odom.pose.pose.position.y
            self.msg_odom.twist.twist.angular.z = float(line[17:23]) - self.msg_odom.pose.pose.position.z

            self.msg_odom.pose.pose.position.x = float(line[3:9])
            self.msg_odom.pose.pose.position.y = float(line[10:16])
            self.msg_odom.pose.pose.position.z = float(line[17:23])
            self.msg_odom.pose.pose.orientation.x = float(line[24:30])
            self.msg_odom.pose.pose.orientation.y = float(line[31:37])
            self.msg_odom.pose.pose.orientation.z = float(line[38:44])
            self.msg_odom.pose.pose.orientation.w = float(line[45:51])
            
            self.odom_publisher.publish(self.msg_odom)

            self.msg_joint.position = [0.0, 0.0]
            self.joint_publisher.publish(self.msg_joint)

            self.robot_transform.header.stamp = self.get_clock().now().to_msg()
            self.robot_transform.transform.translation.x = self.msg_odom.pose.pose.position.x
            self.robot_transform.transform.translation.y = self.msg_odom.pose.pose.position.y
            self.robot_transform.transform.translation.z = self.msg_odom.pose.pose.position.z
            self.robot_transform.transform.rotation.x = self.msg_odom.pose.pose.orientation.x
            self.robot_transform.transform.rotation.y = self.msg_odom.pose.pose.orientation.y
            self.robot_transform.transform.rotation.z = self.msg_odom.pose.pose.orientation.z
            self.robot_transform.transform.rotation.w = self.msg_odom.pose.pose.orientation.w

            self.tf_broadcaster.sendTransform(self.robot_transform)

        if self.last_joy_time <= self.get_clock().now().nanoseconds - 500 * 1000000 and self.last_nav_time <= self.get_clock().now().nanoseconds - 500 * 1000000:
            self.ser.write(bytes(b"RD;CTR;%6.3f;%6.2f;0\n" % (0.0, 0.0)))
        else:
            self.ser.write(bytes(b"RD;CTR;%6.3f;%6.2f;0\n" % (self.lin, self.ang)))
        self.ser.flush()

    def init_msgs(self):
        self.msg_odom = Odometry()
        self.msg_odom.header.stamp = self.get_clock().now().to_msg()
        self.msg_odom.header.frame_id = "world"
        self.msg_odom.child_frame_id = "base_link"
        self.msg_odom.pose.pose.position.x = 0.0
        self.msg_odom.pose.pose.position.y = 0.0
        self.msg_odom.pose.pose.position.z = 0.0
        self.msg_odom.pose.pose.orientation.x = 0.0
        self.msg_odom.pose.pose.orientation.y = 0.0
        self.msg_odom.pose.pose.orientation.z = 0.0
        self.msg_odom.pose.pose.orientation.w = 1.0
        self.msg_odom.pose.covariance = np.zeros(36)
        self.msg_odom.twist.twist.linear.x = 0.0
        self.msg_odom.twist.twist.linear.y = 0.0
        self.msg_odom.twist.twist.linear.z = 0.0
        self.msg_odom.twist.twist.angular.x = 0.0
        self.msg_odom.twist.twist.angular.y = 0.0
        self.msg_odom.twist.twist.angular.z = 0.0
        self.msg_odom.twist.covariance = np.zeros(36)

        self.robot_transform = TransformStamped()
        self.robot_transform.header.stamp = self.get_clock().now().to_msg()
        self.robot_transform.header.frame_id = "world"
        self.robot_transform.child_frame_id = "base_link"
        self.robot_transform.transform.translation.x = 0.0
        self.robot_transform.transform.translation.y = 0.0
        self.robot_transform.transform.translation.z = 0.0
        self.robot_transform.transform.rotation.x = 0.0
        self.robot_transform.transform.rotation.y = 0.0
        self.robot_transform.transform.rotation.z = 0.0
        self.robot_transform.transform.rotation.w = 1.0

        self.msg_joint = JointState()
        self.msg_joint.header.stamp = self.get_clock().now().to_msg()
        self.msg_joint.header.frame_id = "world"
        self.msg_joint.name = ["base_link", "front_link"]
        self.msg_joint.position = [0.0, 0.0]
        self.msg_joint.velocity = [0.0, 0.0]
        self.msg_joint.effort = [0.0, 0.0]

    def destroy_node(self):
        self.timer.cancel()
        time.sleep(0.01)
        if self.ser.is_open:
            self.ser.write(bytes(b"RD;CTR; 0.000;  0;00;0\n"))
            self.ser.flush()
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()
            self.ser.rts = False
            self.ser.dtr = False
            self.ser.close()
        time.sleep(0.1)
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    robot_connector = RobotConnector()
    try:
        rclpy.spin(robot_connector)
    except KeyboardInterrupt:
        pass
    finally:
        robot_connector.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()