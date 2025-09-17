import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class TurtleDiamondController(Node):
    def __init__(self):
        super().__init__('turtle_diamond_controller')
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
        # A 45-degree turn (pi/4 ~= 0.785 rad) at 1.6 rad/s takes ~0.5s, so 1 tick
        # A 90-degree turn (pi/2 ~= 1.57 rad) at 1.6 rad/s takes ~1s, so 2 ticks
        
        if self.time < 1:
            # Initial 45-degree turn to orient the diamond
            msg = self.create_twist(0.0, 1.6)
        elif self.time < 6:
            # Move forward (side 1)
            msg = self.create_twist(1.0, 0.0)
        elif self.time < 8:
            # Turn 90 degrees
            msg = self.create_twist(0.0, 1.6)
        elif self.time < 13:
            # Move forward (side 2)
            msg = self.create_twist(1.0, 0.0)
        elif self.time < 15:
            # Turn 90 degrees
            msg = self.create_twist(0.0, 1.6)
        elif self.time < 20:
            # Move forward (side 3)
            msg = self.create_twist(1.0, 0.0)
        elif self.time < 22:
            # Turn 90 degrees
            msg = self.create_twist(0.0, 1.6)
        elif self.time < 27:
            # Move forward (side 4)
            msg = self.create_twist(1.0, 0.0)
        else:
            # Stop the turtle
            msg = self.create_twist(0.0, 0.0)
        return msg
    
    def timer_callback(self):
        """Publishes the velocity message and increments the time counter."""
        msg = self.get_twist_msg()       
        self.publisher.publish(msg)
        self.time += 1
        print(f"Drawing diamond... Time: {self.time}")

def main(args=None):
    rclpy.init(args=args)
    turtle_diamond_controller = TurtleDiamondController()
    rclpy.spin(turtle_diamond_controller)
    turtle_diamond_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
