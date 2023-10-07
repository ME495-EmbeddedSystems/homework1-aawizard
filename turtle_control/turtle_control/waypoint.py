"""This file declares a node waypoint which has two sevices, and three clients
    Task of the node is to take in a list of waypoints, mark them on the screen and go through each one of them in a loop
    Services:
        /toggle: changes the state of the turtle
        /load  : reset the turtle, takes in a list of wayoints and marks them on the screen
        
    Clients:
        /reset                   : used to reset the turtle
        /turtle1/set_pen         : used to pen up and pen down
        /turtle1/teleportAbsolute: used to teleport turtle from one point to another to mark "X"
    Subscriber:
        /turtle1/pose : reading pose data to get the correct heading of the turtle to the next desired waypoint
    
    Publisher:
        /turtle1/cmd_vel : Giving velocity to turtle at a frequency f
        /loop_metrics    : The information about the no. of loops travled, actual distance travled by the turtle and error from the optimal path is published on this topic each time turle completes a loop
        
        
    Paraameters:
    
        frequency: specifies the frequency at with the timer is 
        tolerance: the threshold  distance for the turtle to be in viciny of a waypoint to consider it visited
    """


import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty
from turtle_interfaces.srv import Waypoints
from turtle_interfaces.msg import ErrorMetric
from enum import Enum, auto
from std_msgs.msg import String
import numpy as np
from geometry_msgs.msg import Twist
from turtlesim.srv import SetPen, TeleportAbsolute
from turtlesim.msg import Pose
import asyncio
from rclpy.callback_groups import  ReentrantCallbackGroup
# from util import *

class State(Enum):
    """ Defines the state of the turtle:
        moving or stopped
    """
    MOVING = auto(),
    STOPPED = auto()
    
class Waypoint(Node):

    def __init__(self):
        super().__init__('waypoint')
        
        self.client_cb_group = ReentrantCallbackGroup()
        self.vel=Twist()
        self.metric=ErrorMetric()
        self.wpoints=[]
        self.metric.actual_distance=0.0
        self.cal_dist=0.0
        self.curr_wpoint=0        #index of current waypoint
        self.goal_wpoint=0
        self.no_wpoint=0
        self.goal_theta=0.0
        
    #creating parameters  
        self.declare_parameter('frequency', 0.01) 
        self.declare_parameter('tolerance', 0.05)   
        self.freq       = self.get_parameter('frequency').get_parameter_value().double_value
        self.tolerance  = self.get_parameter('tolerance').get_parameter_value().double_value
        if self.freq==0:
            self.freq=1

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
        self.timer      = self.create_timer(1/self.freq, self.timer_callback)
    #creating a subscriber
        self.subscriber = self.create_subscription(Pose,'/turtle1/pose',self.listener_callback,10)   
        self.pose=Pose
        
        
    #creating a publisher
        self.publisher_vel = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.publisher_error = self.create_publisher(ErrorMetric,'/loop_metrics',10)
        
        
  
        if not self.reset.wait_for_service(timeout_sec=1.0):
            raise RuntimeError('Timeout waiting for "reset" service to become available')
        
        if not self.setpen.wait_for_service(timeout_sec=1.0):
            raise RuntimeError('Timeout waiting for "setpen" service to become available')
        
        if not self.teleabs.wait_for_service(timeout_sec=1.0):
            raise RuntimeError('Timeout waiting for "teleport_absolute" service to become available')
        
    

    def reset_vel(self):
        """function to reset velocity of the turtle to 0
        """
        
        
        self.vel.linear.x=0.0
        self.vel.linear.y=0.0
        self.vel.linear.z=0.0
        self.vel.angular.x=0.0
        self.vel.angular.y=0.0
        self.vel.angular.z=0.0
        
    def reset_metric(self):
        """Resetting the loop metrics to 0
        """
        self.metric.complete_loops =0
        self.metric.actual_distance =0.0
        self.metric.error=0.0
    
    
    
    async def draw_x(self,x,y):
        """A function to mark a "X" at a given coordinate

        Args:
            x (float): x coordinate of the waypoint
            y (float): y coordinate of the waypoint
        """
        
        #pen up
        await self.setpen.call_async(SetPen.Request(r=255,g=255,b=255,width=2, off=1))
        #take to lower left corner of the cross
        await self.teleabs.call_async(TeleportAbsolute.Request(x=x-0.25,y=y-0.25))
        #pen down
        await self.setpen.call_async(SetPen.Request(r=255,g=255,b=255,width=2,off=0))
        
        #take to upper right corner of the cross
        await self.teleabs.call_async(TeleportAbsolute.Request(x=x+0.25,y=y+0.25))
        #pen up
        await self.setpen.call_async(SetPen.Request(r=255,g=255,b=255,width=2,off=1))
        
        #take to lower right corner of the cross
        await self.teleabs.call_async(TeleportAbsolute.Request(x=x+0.25,y=y-0.25))
        #pen down
        await self.setpen.call_async(SetPen.Request(r=255,g=255,b=255,width=2,off=0))
        
        #take to upper left corner of the cross
        await self.teleabs.call_async(TeleportAbsolute.Request(x=x-0.25,y=y+0.25))
        
        # #pen up
        await self.setpen.call_async(SetPen.Request(r=255,g=255,b=255,width=2,off=1))
        
      
    def total_dist(self,wpoints):
        """"Function to calculate straight line loop distance for a list of waypoints

        Args:
            wpoints (list): list of waypoints

        Returns:
            float: straight line distance
        """
        dist=0
        #traverse through the list to calculate distance    
        for i in range(len(wpoints)-1):
           dist+=np.linalg.norm(wpoints[i+1] - wpoints[i])
        
        dist+=np.linalg.norm(wpoints[0]-wpoints[-1])
        
        return dist
        
    def get_theta(self):
        """Function to calculate the heading angle for the turtle from its current location to the next waypoint

        
        """
        
        next_wpoint=self.wpoints[self.goal_wpoint]  
        curr_wpoint=[self.pose.x,self.pose.y]
        return np.arctan2([next_wpoint[1]-curr_wpoint[1]], [next_wpoint[0]-curr_wpoint[0]])[0]
        
        
    def get_next_waypoint(self):
        """When a turtle reaches a waypoint this function is called:
        
        it gets the next waypoint and updates the current waypoint 
        """
        l=self.no_wpoint
        if self.curr_wpoint==self.goal_wpoint and l>1:
            self.goal_wpoint+=1
            
        else:
            
            
            self.curr_wpoint=self.goal_wpoint
            if self.goal_wpoint==l-1:
                self.goal_wpoint=0
            else:
                self.goal_wpoint+=1
                
            if self.curr_wpoint==0:
                self.metric.complete_loops+=1
                self.metric.error = abs((self.metric.complete_loops*self.cal_dist)-self.metric.actual_distance)
                self.publisher_error.publish(self.metric)
        
        next_wpoint=self.wpoints[self.goal_wpoint]  
        curr_wpoint=[self.pose.x,self.pose.y]

        self.goal_theta=self.get_theta()
        
    
    
    def get_dist(self,pose,wpoint):
        """Function to calculaate the distance between the pose of the turtle and given waypoint

        Args:
            pose (turtlesim/msg/Pose): pose of the turtle, x,y, theta
            wpoint (list): the x and y coordinate of a waypoint

        Returns:
            _type_: _description_
        """
        x=pose.x
        y=pose.y
        return np.linalg.norm([x,y] - self.wpoints[wpoint])
        
        
    #callback for load service     
    async def  load_callback(self,request,response):
        """Loads all the waypoints, mark X at each waypoint and return the loop length

        Args:
            request (Waypoint_list): list of x and y of waypoints
            response (float): total distance of a loop

        
        """
        self.reset_vel()
        self.publisher_vel.publish(self.vel)
        await self.reset.call_async(Empty.Request())
        self.reset_metric()
        self.reset_vel()
            
        #get points in a list and draw cross as each point
        for point in request.wpoint:
            self.wpoints.append(np.array((point.x,point.y)))
            await self.draw_x(point.x,point.y)     #Drawing X at each waypoint
        
        #take the turtle to first waypoint
        self.no_wpoint=len(self.wpoints)
        await self.teleabs.call_async(TeleportAbsolute.Request(x=self.wpoints[0][0],y=self.wpoints[0][1]))
        #pen down
        await self.setpen.call_async(SetPen.Request(r=255,g=255,b=255,width=2,off=0))
        self.state      = State.STOPPED 

        self.cal_dist=self.total_dist(wpoints=self.wpoints)
        response.dist = self.cal_dist  #calculatinf straight line distance

        return response
    
    
    #callback for toggle service
    def toggle_callback(self, request, response):
        """Toggles the turtle state and stops the moving turtle

        Args:
            request (Empty):
            response (Empty): 

      
        """
        if self.state==State.MOVING:
            self.state=State.STOPPED
            self.reset_vel()
            
            self.get_logger().info('Stopping')
        else:
            if self.no_wpoint==0:
                self.get_logger().error("No waypoints loaded. Load them with the 'load' service")
            else:
                self.state=State.MOVING
            
        return response
    
    #callback for subscriber
    def listener_callback(self, msg):
        """A callback function for subscriber that gets the value from turtle1/pose topic and saves to self.pose

        Args:
            msg (turtlesin/msg/Pose): x,y,theta of the turtle
        """
        self.pose=msg
    
    #callback for timmer
    def timer_callback(self):
        """_summary_
        This function is run at the frequency of the parameter
        
        It moves the turtle from one node to another
        
        
        """
        
        
        
        msg = String()
        msg.data = 'Issuing Command!'
        
        if self.state==State.MOVING:
            
            self.vel.linear.y=0.0
            self.get_logger().debug('Publishing: "%s"' % msg.data)

            dist=self.get_dist(self.pose,self.goal_wpoint)

            if dist<=self.tolerance:
                theta=self.get_next_waypoint()

            
            else:
                
                if abs(self.pose.theta-self.goal_theta)>0.01:
                    if self.pose.theta-self.goal_theta>0:
                       
                        self.vel.angular.z=-0.5
                    else:
                        self.vel.angular.z=0.5
                    self.vel.linear.x=0.0
                else:
                    self.vel.angular.z=0.0
                    self.vel.linear.x=dist
                    if dist<1:
                        self.vel.linear.x=1.0
                    
                    self.metric.actual_distance+= (dist/self.freq)

        self.publisher_vel.publish(self.vel)





def main(args=None):
    """ Entry point of the file
    """
    rclpy.init(args=args)

    waypoint_node = Waypoint()

    rclpy.spin(waypoint_node)
    waypoint_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    asyncio.run(main())