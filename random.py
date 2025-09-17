import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class TurtleHeartController(Node):
    def __init__(self):
        super().__init__('turtle_heart_controller')
        self.publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.time = 0

    def create_twist(self, linear_x, angular_z):
        """Creates a Twist message with the given velocities."""
        msg = Twist()
        msg.linear.x = float(linear_x)
        msg.angular.z = float(angular_z)
        return msg

    def get_twist_msg(self):
        """Determines velocity message based on time ticks."""
        # To draw heart, start from the bottom and move up and left first
       
        if self.time < 3:
            # Initially, we want ~135 deg turn left
            # 3 ticks * 0.5s/tick * 1.6 rad/s = 2.4 rad = 137 deg
            msg = self.create_twist(0.0, 1.6)
        elif self.time < 7:
            # Move up and left to make the first straight edge of the heart
            msg = self.create_twist(1.5, 0.0)
        elif self.time < 16:
            # First arc, left half of heart
            msg = self.create_twist(1.0, -0.9)
        elif self.time <19:
            # turn ~205 degrees left
            # 3 ticks * 0.5s/tick * 2.4 rad/s = 3.6 rad = 206 deg
            msg = self.create_twist(0.0, 2.4)
        elif self.time < 28:
            # Second arc, right half of heart
            # Symmetric to left arc
            msg = self.create_twist(1.0, -0.9)
        elif self.time < 32:
            # Final edge to return to start
            msg = self.create_twist(1.5, 0.0)
        else:
            # Stop
            msg = self.create_twist(0.0, 0.0)
        return msg
   
    def timer_callback(self):
        """Publishes velocity message and increments time counter"""
        msg = self.get_twist_msg()     
        self.publisher.publish(msg)
        self.time += 1
        if self.time < 32:
            print(f"Drawing heart... Time: {self.time}")

def main(args=None):
    rclpy.init(args=args)
    turtle_heart_controller = TurtleHeartController()
    rclpy.spin(turtle_heart_controller)
    turtle_heart_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
