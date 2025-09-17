import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math

class TurtleCircleController(Node):
    def __init__(self):
        super().__init__('turtle_circle_controller')
        self.publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.time = 0
        # Full circle is 2pi radians
        # With angular z=1, it takes 2pi seconds
        # Ticks needed = 2pi / timer_period = (4pi) / 0.5 = 12.56
        # Stop after 13 ticks.
        self.stop_time = 13 

    def create_twist(self, linear_x, angular_z):
        """Creates a Twist message with the given velocities."""
        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z
        return msg

    def get_twist_msg(self):
        """Determines velocity message based on time ticks."""
        # Move in a circle until stop time is reached
        if self.time < self.stop_time:
            # Constant linear and angular velocity = circle
            msg = self.create_twist(2.0, 1.0)
        else:
            # Stop
            msg = self.create_twist(0.0, 0.0)
        return msg
    
    def timer_callback(self):
        """Publishes velocity message and increments time counter."""
        msg = self.get_twist_msg()       
        self.publisher.publish(msg)
        # Only increment time if we are not stopped
        if self.time <= self.stop_time:
            self.time += 1
            print(f"Drawing circle... Time: {self.time}")

def main(args=None):
    rclpy.init(args=args)
    turtle_circle_controller = TurtleCircleController()
    rclpy.spin(turtle_circle_controller)
    turtle_circle_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
