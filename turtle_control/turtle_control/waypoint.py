import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class Waypoint(Node):

    def __init__(self):
        super().__init__('waypointer')
        frequency=100
        timer_period = 1/frequency  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        # self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'Issuing Command!'
        self.get_logger().debug('Publishing: "%s"' % msg.data)
        # self.i += 1


def main(args=None):
    rclpy.init(args=args)

    waypoint_node = Waypoint()

    rclpy.spin(waypoint_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    waypoint_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()