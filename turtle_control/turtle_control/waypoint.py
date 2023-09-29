import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty
from enum import Enum, auto
from std_msgs.msg import String

class State(Enum):

    MOVING = auto(),
    STOPPED = auto()
    
class Waypoint(Node):

    def __init__(self):
        super().__init__('waypointer')
        self.declare_parameter('frequency', 1)
        self.freq = self.get_parameter('frequency').get_parameter_value().double_value
        self.srv = self.create_service(Empty, 'toggle', self.toggle)
        self.timer = self.create_timer(self.freq, self.timer_callback)
        self.state=State.STOPPED   

        
        
    def toggle(self):
        if self.state==State.MOVING:
            self.state=State.STOPPED
            self.get_logger().info('Stopping')
        else:
            self.state=State.MOVING
        
    
    
    def timer_callback(self):
        msg = String()
        if self.state==State.MOVING:
            msg.data = 'Issuing Command!'
            self.get_logger().debug('Publishing: "%s"' % msg.data)
            
        



def main(args=None):
    rclpy.init(args=args)

    waypoint_node = Waypoint()

    rclpy.spin(waypoint_node)
    waypoint_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()