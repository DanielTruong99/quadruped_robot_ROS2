#!/usr/bin/python3

import rclpy
import time
from rclpy.node import Node
from math import fabs
from numpy import array_equal
import threading
from sensor_msgs.msg import Joy

class PS4_controller(Node):
    def __init__(self, rate):
        super().__init__('joystick')

        timer_period = 1/rate
        self.joy_publisher = self.create_publisher(Joy, "spot_joy/joy_ramped", 10)
        self.timer = self.create_timer(timer_period, self.run)

        # target
        self.target_joy = Joy()
        self.target_joy.axes = [0.,0.,1.,0.,0.,1.,0.,0.]
        self.target_joy.buttons = [0,0,0,0,0,0,0,0,0,0,0]

        # last
        self.last_joy = Joy()
        self.last_joy.axes = [0.,0.,1.,0.,0.,1.,0.,0.]
        self.last_joy.buttons = [0,0,0,0,0,0,0,0,0,0,0]
        #self.last_send_time = self.get_clock().now()
        self.last_send_time = time.time()

        self.speed_index = 2
        self.available_speeds = [0.5, 1.0, 3.0, 4.0]

    def run(self):
        global axes
        self.publish_joy()

    def ramped_vel(self,v_prev,v_target,t_prev,t_now):
        #step = (t_now - t_prev).to_sec()
        step = t_now - t_prev
        #self.get_logger().info(f"tnow:{t_now}, t_prev:{t_prev}, step:{step}")

        sign = self.available_speeds[self.speed_index] if \
                (v_target > v_prev) else -self.available_speeds[self.speed_index]
        error = fabs(v_target - v_prev)

        # if we can get there within this timestep -> we're done.
        if error < self.available_speeds[self.speed_index]*step:
            return v_target
        else:
            return v_prev + sign * step # take a step toward the target

    def publish_joy(self):
        #t_now = self.get_clock().now()
        t_now = time.time()

        # determine changes in state
        buttons_change = array_equal(self.last_joy.buttons, self.target_joy.buttons)
        axes_change = array_equal(self.last_joy.axes, self.target_joy.axes)

        # if the desired value is the same as the last value, there's no
        # need to publish the same message again
        if not(buttons_change and axes_change):
            # new message
            joy = Joy()
            if not axes_change:
                # do ramped_vel for every single axis
                for i in range(len(self.target_joy.axes)): 
                    if self.target_joy.axes[i] == self.last_joy.axes[i]:
                        joy.axes.append(self.last_joy.axes[i])
                    else:
                        joy.axes.append(self.ramped_vel(self.last_joy.axes[i],
                                                        self.target_joy.axes[i],
                                                        self.last_send_time,
                                                        t_now))
            else:
                joy.axes = self.last_joy.axes

            joy.buttons = self.target_joy.buttons
            self.last_joy = joy
            self.joy_publisher.publish(self.last_joy)

        self.last_send_time = t_now

class Joy_subscriber(Node):

    def __init__(self):
        super().__init__('joy_subscriber')
        self.use_button = True

        self.subscription = self.create_subscription(
            Joy,
            'joy',
            self.listener_callback,
            10)
        self.subscription

    def listener_callback(self, msg):
        if self.use_button:
            if msg.buttons[4]:
                joystick_pub.speed_index -= 1
                if joystick_pub.speed_index < 0:
                    joystick_pub.speed_index = len(joystick_pub.available_speeds) - 1
                rclpy.logging._root_logger.info(f"Joystick speed:{joystick_pub.available_speeds[joystick_pub.speed_index]}")
                self.use_button = False
            elif msg.buttons[5]:
                joystick_pub.speed_index += 1
                if joystick_pub.speed_index >= len(joystick_pub.available_speeds):
                    joystick_pub.speed_index = 0
                rclpy.logging._root_logger.info(f"Joystick speed:{joystick_pub.available_speeds[joystick_pub.speed_index]}")
                self.use_button = False

        if not self.use_button:
            if not(msg.buttons[4] or msg.buttons[5]):
                self.use_button = True

        joystick_pub.target_joy.axes = msg.axes
        joystick_pub.target_joy.buttons = msg.buttons

if __name__ == "__main__":
    rclpy.init(args=None)

    joystick_pub = PS4_controller(rate = 30)
    joy_subscriber = Joy_subscriber()

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(joystick_pub)
    executor.add_node(joy_subscriber)

    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()
    rate = joystick_pub.create_rate(2)

    try:
        while rclpy.ok():
            rate.sleep()
    except KeyboardInterrupt:
        print("KeyboardInterrupt")

    rclpy.shutdown()
    executor_thread.join()

