import rclpy
from ackermann_msgs.msg import AckermannDrive


class TractorDriver:
    def step(self):
        rclpy.spin_once(self.__node, timeout_sec=0)

    def init(self, webots_node, properties):
        self.__robot = webots_node.robot
        self.FRONT_WHEEL_RADIUS = 0.38
        self.REAR_WHEEL_RADIUS = 0.6

        self.left_front_wheel = self.__robot.getDevice("left_front_wheel")
        self.right_front_wheel = self.__robot.getDevice("right_front_wheel")
        self.left_rear_wheel = self.__robot.getDevice("left_rear_wheel")
        self.right_rear_wheel = self.__robot.getDevice("right_rear_wheel")

        self.steering_angle = 0.0

        self.steering_goal = 0.0
        self.steering_curr = 0.0
        self.change_rate = 0.1

        self.left_steer = self.__robot.getDevice("left_steer")
        self.right_steer = self.__robot.getDevice("right_steer")

        # ROS interface
        rclpy.init(args=None)
        self.__node = rclpy.create_node('tractor_node')
        self.__node.create_subscription(AckermannDrive, 'cmd_ackermann', self.__cmd_ackermann_callback, 1)

    def __cmd_ackermann_callback(self, message):
        self.set_speed(message.speed)
        self.change_manual_steer_angle(message.steering_angle)


    def set_speed(self, kmh):
        if kmh > 30.0:
            kmh = 30.0
        self.speed = kmh
        # print(f"setting speed to {kmh} km/h")
        self.__node.get_logger().info(f'setting speed to {kmh} km/h')
        front_ang_vel = kmh * 1000.0 / 3600.0 / self.FRONT_WHEEL_RADIUS
        rear_ang_vel = kmh * 1000.0 / 3600.0 / self.REAR_WHEEL_RADIUS
        # set motor rotation speed
        self.left_front_wheel.setVelocity(front_ang_vel)
        self.right_front_wheel.setVelocity(front_ang_vel)
        self.left_rear_wheel.setVelocity(rear_ang_vel)
        self.right_rear_wheel.setVelocity(rear_ang_vel)

    def change_manual_steer_angle(self, wheel_angle):
        self.steering_goal = min(0.94, max(-0.94, wheel_angle))

        if self.steering_goal == self.steering_curr:
            return
        
        if self.steering_goal > self.steering_curr:
            self.steering_curr += self.change_rate
        elif self.steering_goal < self.steering_curr:
            self.steering_curr -= self.change_rate
        self.left_steer.setPosition(self.steering_curr)
        self.right_steer.setPosition(self.steering_curr)


    def set_steering_angle(self, wheel_angle):
        self.steering_angle = min(0.94, max(-0.94, wheel_angle))
        self.left_steer.setPosition(self.steering_angle)
        self.right_steer.setPosition(self.steering_angle)