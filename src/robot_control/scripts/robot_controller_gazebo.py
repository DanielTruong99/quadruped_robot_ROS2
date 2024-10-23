#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy,Imu
from RobotController import RobotController
from InverseKinematics import robot_IK
from std_msgs.msg import Float64
from std_msgs.msg import Float64MultiArray
import threading
import numpy as np

USE_IMU = False

# Robot geometry
body = [0.559, 0.12]
legs = [0.,0.1425, 0.426, 0.345] 

spot_robot = RobotController.Robot(body, legs, USE_IMU)
inverseKinematics = robot_IK.InverseKinematics(body, legs)

del body
del legs
del USE_IMU

class Controller(Node):
    def __init__(self):
        super().__init__('controller')
        self.time_interval = 0.015
        self.publisher = self.create_publisher(Float64MultiArray, '/forward_position_controller/commands', 10)
        self.timer = self.create_timer(self.time_interval, self.timer_callback)

        # for debug
        float_formatter = "{:.2f}".format
        np.set_printoptions(formatter={'float_kind':float_formatter})

    def timer_callback(self):
        leg_positions = spot_robot.run()
        spot_robot.change_controller()

        dx = spot_robot.state.body_local_position[0]
        dy = spot_robot.state.body_local_position[1]
        dz = spot_robot.state.body_local_position[2]
    
        roll = spot_robot.state.body_local_orientation[0]
        pitch = spot_robot.state.body_local_orientation[1]
        yaw = spot_robot.state.body_local_orientation[2]

        try:
            joint_angles = inverseKinematics.inverse_kinematics(leg_positions,
                                                                dx, dy, dz, 
                                                                roll, pitch, yaw)
            pos_array = Float64MultiArray(data=joint_angles)
            self.publisher.publish(pos_array)
        except:
            import traceback
            traceback.print_exc()
            rclpy.logging._root_logger.info(f"Can not solve inverese kinematics")

class Joy_Subscriber(Node):

    def __init__(self):
        super().__init__('get_modelstate')
        self.subscription = self.create_subscription(
            Joy,
            '/spot_joy/joy_ramped',
            spot_robot.joystick_command,
            10)
        self.subscription

class Imu_Subscriber(Node):

    def __init__(self):
        super().__init__('imu_subscriber')
        self.subscription = self.create_subscription(
            Imu,
            '/imu',
            spot_robot.imu_orientation,
            10)
        self.subscription

if __name__ == "__main__":
    rclpy.init(args=None)
    controller = Controller()
    joy_subscriber = Joy_Subscriber()
    imu_subscriber = Imu_Subscriber()

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(controller)
    executor.add_node(joy_subscriber)
    executor.add_node(imu_subscriber)

    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()

    rate = controller.create_rate(2)
    try:
        while rclpy.ok():
            rate.sleep()
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
    executor_thread.join()
