import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty
from turtle_interfaces.srv import Waypoints
from enum import Enum, auto
from std_msgs.msg import String
import numpy as np
from turtlesim.srv import SetPen, TeleportAbsolute
from turtlesim.msg import Pose
import asyncio
from rclpy.callback_groups import  ReentrantCallbackGroup


class State(Enum):

    MOVING = auto(),
    STOPPED = auto()
    
class Waypoint(Node):

    def __init__(self):
        super().__init__('waypoint')
        self.declare_parameter('frequency', 1) 
        self.client_cb_group = ReentrantCallbackGroup()
     
        self.freq       = self.get_parameter('frequency').get_parameter_value().double_value
    #creating services  
        self.toggle     = self.create_service(Empty, 'toggle', self.toggle_callback)
        self.load       = self.create_service(Waypoints, 'load', self.load_callback)
    #creating clients
        self.reset      = self.create_client(Empty, "reset",callback_group=self.client_cb_group)
        self.setpen     = self.create_client(SetPen,'turtle1/set_pen',callback_group=self.client_cb_group)
        self.teleabs    = self.create_client(TeleportAbsolute,'turtle1/teleport_absolute',callback_group=self.client_cb_group)
    #state of the node set as stopped 
        self.state      = State.STOPPED   
    #Timer created        
        self.timer      = self.create_timer(self.freq, self.timer_callback)
    #creating a subscriber
        self.subscription = self.create_subscription(Pose,'/turtle1/pose',self.listener_callback,10)   
        self.pose=Pose
  
        if not self.reset.wait_for_service(timeout_sec=1.0):
            raise RuntimeError('Timeout waiting for "reset" service to become available')
        
        if not self.setpen.wait_for_service(timeout_sec=1.0):
            raise RuntimeError('Timeout waiting for "setpen" service to become available')
        
        if not self.teleabs.wait_for_service(timeout_sec=1.0):
            raise RuntimeError('Timeout waiting for "teleport_absolute" service to become available')
        
    
    
    
    
    
    
    async def draw_x(self,x,y):
        #pen up
        await self.setpen.call_async(SetPen.Request(off=1))
        #take to lower left corner of the cross
        await self.teleabs.call_async(TeleportAbsolute.Request(x=x-0.25,y=y-0.25))
        #pen down
        await self.setpen.call_async(SetPen.Request(off=0))
        
        #take to upper right corner of the cross
        await self.teleabs.call_async(TeleportAbsolute.Request(x=x+0.25,y=y+0.25))
        #pen up
        await self.setpen.call_async(SetPen.Request(off=1))
        
        #take to lower right corner of the cross
        await self.teleabs.call_async(TeleportAbsolute.Request(x=x+0.25,y=y-0.25))
        #pen down
        await self.setpen.call_async(SetPen.Request(off=0))
        
        #take to upper left corner of the cross
        await self.teleabs.call_async(TeleportAbsolute.Request(x=x-0.25,y=y+0.25))
        
        # #pen up
        await self.setpen.call_async(SetPen.Request(off=1))
        
        
    def cal_dist(self,wpoints):
        dist=0
        #traverse through the list to calculate distance    
        for i in range(len(wpoints)-1):
           dist+=np.linalg.norm(wpoints[i+1] - wpoints[i])
        
        # dist+=np.linalg.norm(wpoints[0]-wpoints[-1])
        
        return dist
        
        
        
    #callback for load service     
    async def  load_callback(self,request,response):
        await self.reset.call_async(Empty.Request())
        
        wpoints=[]
        
        #get points in a list and draw cross as each point
        for point in request.wpoint:
            wpoints.append(np.array((point.x,point.y)))
            await self.draw_x(point.x,point.y)     #Drawing X at each waypoint
        
        #take the turtle to first waypoint
        # x=request.wpoint[0].x
        # y=request.wpoint[0].y
        
        x=wpoints[0][0]
        y=wpoints[0][1]
        await self.teleabs.call_async(TeleportAbsolute.Request(x=x,y=y))
        self.state      = State.STOPPED 
        
        
        response.dist = self.cal_dist(wpoints=wpoints)  #calculatinf straight line distance
        # response.dist= 2.0
        return response
    
    
    #callback for toggle service
    def toggle_callback(self, request, response):
        if self.state==State.MOVING:
            self.state=State.STOPPED
            self.get_logger().info('Stopping')
        else:
            self.state=State.MOVING
        return response
    
    #callback for subscriber
    def listener_callback(self, msg):
        self.pose=msg
    
    #callback for timmer
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
    asyncio.run(main())