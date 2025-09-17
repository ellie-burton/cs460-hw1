import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class TurtleRectangleController(Node):
    def __init__(self):
        super().__init__('turtle_rectangle_controller')
        self.publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.time = 0

    def create_twist(self, linear_x, angular_z):
        """Creates a Twist message with the given velocities."""
        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z
        return msg

    def get_twist_msg(self):
        """Determines velocity message based on time ticks."""
        # Each tick is 1/2s, so 90-degree turn (pi/2 = 1.57 rad) at 1.6 rad/s takes ~1s, so 2 ticks.
        # Long side: 8 ticks (4s), Short side: 4 ticks (2s)
        
        if self.time < 8:
            # Move forward (long side 1)
            msg = self.create_twist(1.0, 0.0)
        elif self.time < 10:
            # Turn 90 degrees
            msg = self.create_twist(0.0, 1.6)
        elif self.time < 14:
            # Move forward (short side 1)
            msg = self.create_twist(1.0, 0.0)
        elif self.time < 16:
            # Turn 90 degrees
            msg = self.create_twist(0.0, 1.6)
        elif self.time < 24:
            # Move forward (long side 2)
            msg = self.create_twist(1.0, 0.0)
        elif self.time < 26:
            # Turn 90 degrees
            msg = self.create_twist(0.0, 1.6)
        elif self.time < 30:
            # Move forward (short side 2)
            msg = self.create_twist(1.0, 0.0)
        else:
            # Stop
            msg = self.create_twist(0.0, 0.0)
        return msg
    
    def timer_callback(self):
        """Publishes velocity message and increments time counter."""
        msg = self.get_twist_msg()       
        self.publisher.publish(msg)
        self.time += 1
        print(f"Drawing rectangle... Time: {self.time}")

def main(args=None):
    rclpy.init(args=args)
    turtle_rectangle_controller = TurtleRectangleController()
    rclpy.spin(turtle_rectangle_controller)
    turtle_rectangle_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
