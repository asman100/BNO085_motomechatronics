import time
from math import atan2, sqrt
from math import pi as PI

from adafruit_extended_bus import ExtendedI2C as I2C
import adafruit_bno08x
from adafruit_bno08x.i2c import BNO08X_I2C
import traceback
import board
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3

orientation_quat = [0,0,0,0] # x, y, z, w (real)
orientation_rad = [0,0,0] # roll, pitch, yaw(heading)  (in rad)
orientation_deg = [0,0,0] # roll, pitch, yaw(heading)  (in deg)

linear_accel = [0,0,0] # x, y, z  (in m/s^2)
gyro = [0,0,0] # x, y, z  (in deg/s)

class BNO085_Publisher(Node):
    def __init__(self):
        super().__init__('BNO085_Publisher')
        self.imu_data_publisher = self.create_publisher(
            Imu,
            'IMU_Data',
            10)
        self.robot_orientation_publisher  = self.create_publisher(
            Vector3,
            'Robot_Euler_Orientation',
            10)

        self.imu = None
        self.target_frequency = 40.0  # Hz  <--- DEFINE TARGET FREQUENCY
        self.init_sensor()

        # create timer for reading and publishing data
        timer_period = 1.0 / self.target_frequency # seconds
        self.get_logger().info(f"Setting ROS2 timer to {timer_period:.4f}s ({self.target_frequency} Hz)")
        self.read_send_timer = self.create_timer(timer_period, self.read_and_send_imu_data)

    def init_sensor(self):
        try:
            # Explicitly set I2C frequency to 400kHz
            # For MCP2221A with Adafruit Blinka, board.I2C() can take a frequency argument
            self.get_logger().info("Attempting to initialize I2C with 400kHz frequency.")
            i2c = board.I2C(frequency=400000)
            # If the above board.I2C(frequency=...) doesn't work directly with your Blinka setup for MCP2221A,
            # you might need to use busio explicitly:
            # i2c = busio.I2C(board.SCL, board.SDA, frequency=400000)
            self.get_logger().info("I2C bus initialized.")
        except Exception as e:
            self.get_logger().error(f"Failed to initialize I2C bus: {e}")
            self.get_logger().error(traceback.format_exc())
            raise

        try:
            self.get_logger().info("Attempting to connect to BNO085 via I2C...")
            self.imu = BNO08X_I2C(i2c)
            self.get_logger().info("Successfully connected to BNO085.")
        except Exception as e:
            self.get_logger().error(f"Failed to connect to BNO085 via I2C: {e}")
            self.get_logger().error(traceback.format_exc())
            raise Exception('Failed to connect to BNO085 via I2C')

        # Calculate report interval in microseconds
        report_interval_us = int((1.0 / self.target_frequency) * 1000000)
        self.get_logger().info(f"Setting BNO08x report interval to {report_interval_us} us.")

        try:
            self.imu.enable_feature(adafruit_bno08x.BNO_REPORT_ROTATION_VECTOR, report_interval_us=report_interval_us)
            self.imu.enable_feature(adafruit_bno08x.BNO_REPORT_LINEAR_ACCELERATION, report_interval_us=report_interval_us)
            self.imu.enable_feature(adafruit_bno08x.BNO_REPORT_GYROSCOPE, report_interval_us=report_interval_us) # Use calibrated if available

            self.get_logger().info("BNO08x features enabled with desired report interval.")
        except Exception as e:
            self.get_logger().error(f"Error enabling BNO08x features: {e}")
            self.get_logger().error(traceback.format_exc())
            # It's possible the chip is in a weird state, try a hardware reset if issues persist after code changes
            # self.imu.hard_reset() # Be cautious with hard_reset, may require re-init
            raise

        time.sleep(0.5) # Allow time for features to stabilize
        self.get_logger().info("IMU Initialized.")

    def read_and_send_imu_data(self):
        # get the Angular Velocity (gryo data) of the robot
        gyro[0], gyro[1], gyro[2] = self.imu.gyro 
        # get the Linear Acceleration of the robot
        linear_accel[0], linear_accel[1], linear_accel[2] = self.imu.linear_acceleration  
        # get the quaternion representation of the robot's orientation
        orientation_quat[0], orientation_quat[1], orientation_quat[2], orientation_quat[3] = self.imu.quaternion
        # calculate the euler representation of the robot's orientation
        self.__calc_euler_angles_deg()

        #create messages to publish
        imu_data_msg = Imu()
        robot_ori_euler_msg = Vector3()

        # TODO: Double check that this is true
        # IMU X right, Y forward, Z up
        # ROS Y left, X forward, Z up
        imu_data_msg.angular_velocity.x = gyro[1]
        imu_data_msg.angular_velocity.y = -gyro[0]
        imu_data_msg.angular_velocity.z = gyro[2]
        imu_data_msg.linear_acceleration.x = linear_accel[1]
        imu_data_msg.linear_acceleration.y = -linear_accel[0]
        imu_data_msg.linear_acceleration.z = linear_accel[2]
        imu_data_msg.orientation.x = orientation_quat[1]
        imu_data_msg.orientation.y = -orientation_quat[0]
        imu_data_msg.orientation.z = orientation_quat[2]
        imu_data_msg.orientation.w = orientation_quat[3]

        robot_ori_euler_msg.x = orientation_deg[0] # roll
        robot_ori_euler_msg.y = orientation_deg[1] # pitch
        robot_ori_euler_msg.z = orientation_deg[2] # yaw
        
        #TODO: Look into getting the IMU's Covariance values
        # Following the recommendation here: https://robotics.stackexchange.com/questions/22756/what-would-be-a-way-to-estimate-imu-noise-covariance-matrix
        # we set the covariance values to a small values (just in case they are needed) on the diagonal
        # imu_data_msg.orientation_covariance[0] = 0.01
        # imu_data_msg.orientation_covariance[4] = 0.01
        # imu_data_msg.orientation_covariance[8] = 0.01
        # imu_data_msg.angular_velocity_covariance[0] = 0.01
        # imu_data_msg.angular_velocity_covariance[4] = 0.01
        # imu_data_msg.angular_velocity_covariance[8] = 0.01
        # imu_data_msg.linear_acceleration_covariance[0] = 0.01
        # imu_data_msg.linear_acceleration_covariance[4] = 0.01
        # imu_data_msg.linear_acceleration_covariance[8] = 0.01

        self.imu_data_publisher.publish(imu_data_msg)
        self.robot_orientation_publisher.publish(robot_ori_euler_msg)

    def __calc_euler_orientation_angles_rad(self):
        # calc the roll (x-axis rotation)
        sinr_cosp = 2 * (orientation_quat[3] * orientation_quat[0] + orientation_quat[1] * orientation_quat[2])
        cosr_cosp = 1 - 2 * (orientation_quat[0] * orientation_quat[0] + orientation_quat[1] * orientation_quat[1])
        orientation_rad[0] = atan2(sinr_cosp, cosr_cosp)
        
        # calc the pitch (y-axis rotation)
        sinp = sqrt(1 + 2 * (orientation_quat[3] * orientation_quat[1] - orientation_quat[0] * orientation_quat[2]))
        cosp = sqrt(1 - 2 * (orientation_quat[3] * orientation_quat[1] - orientation_quat[0] * orientation_quat[2]))
        orientation_rad[1] = 2 * atan2(sinp, cosp) - PI/2
        
        # calc the yaw (z-axis rotation)
        siny_cosp = 2 * (orientation_quat[3] * orientation_quat[2] + orientation_quat[0] * orientation_quat[1])
        cosy_cosp = 1 - 2 * (orientation_quat[1] * orientation_quat[1] + orientation_quat[2] * orientation_quat[2])
        orientation_rad[2] = atan2(siny_cosp, cosy_cosp)
            
    def __calc_euler_angles_deg(self):
        self.__calc_euler_orientation_angles_rad()
        orientation_deg[0] = orientation_rad[0] * (180/PI)
        orientation_deg[1] = orientation_rad[1] * (180/PI)
        orientation_deg[2] = 360 - (orientation_rad[2] * (180/PI))


def main(args=None):
    rclpy.init(args=args)
    try:
        bno_publisher = BNO085_Publisher()
        rclpy.spin(bno_publisher)
        bno_publisher.destroy_node()
    except Exception as e: 
        print(orientation_quat)
        print("------------------")
        print(traceback.format_exc())
        #print(e)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()


if __name__ == '__main__':
    main()
