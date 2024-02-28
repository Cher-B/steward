
''' Algorithm goes like:
pass route and pose to same callback(not sure if allowed in ros)
inside callback
    #sequentially go to each node
    #current node counter- class variable always called----------------? #optional step
    #next targeted node
    #get pose right now
    #pass pose and target node to the calculation function(in here until target is reached)
    #return velocities and node completed
    #append to node counter


'''



import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, TwistStamped
from steward_msgs.msg import Route
import math
import numpy as np

class MotionController(Node):
    def __init__(self):
        super().__init__('MotionController')
        self.route = None
        self.pose = None
        #initialize node counter- class variable

        # Subscribe to the "route" topic
        self.route_subscription = self.create_subscription(
            Route,  # Adjusting to use the custom message type
            'route',
            self.callback(Route,PoseWithCovarianceStamped), # not sure if we can pass 2 arguments like this
            10)
        self.route_subscription  # prevent unused variable warning

        # Subscribe to the PoseWithCovarianceStamped topic
        self.pose_subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            'pose',
            self.callback(Route,PoseWithCovarianceStamped), # not sure if we can pass 2 arguments like this
            10)
        self.pose_subscription  # prevent unused variable warning

        # Publish the TwistStamped message
        self.twist_publisher = self.create_publisher(
            TwistStamped,
            'cmd_vel',
            10)
        
    def callback(self,route,pose):
        #route- points array in order of traversal
        self.current_node = []
        for i in (self.route.len()): # sequentially go to each node
            # header = msg.header
            node_x = route.points[i].x # x pos of planting location in ground frame
            node_y = route.points[i].y # y pos of planting location in ground frame
            node_z = route.points[i].z # z pos of planting location in ground frame
            self.current_node = self.current_node.append(i)#records the nodes traversed to, the last index is the most recent one
            process_data(route.points[i],pose)
        #current node counter- class variable always called----------------? #optional step
        #next node to target
        #pose right now
        #pass pose and target node to the calculation function
        #return velocity and node completed
        #append to node counter
        a=0
        


#STOP trying to understand the code beyond this point. Its a hodgepodge mishmash monstrosity. Use it only as helper code. Continue from the point below mentioned.
    def process_vel(self,x,y,z,pose_msg):
        if route_msg is not None and pose_msg is not None:
            self.get_logger().info('Processing velocity. Route: %s, Pose: %s' % (route_msg, pose_msg))
            # Compute a TwistStamped message in robot frame
            header = pose_msg.header #seq, stamp, frame_id
            pose_with_covariance = pose_msg.pose
            
            # Extract data from the PoseWithCovariance message
            covariance_matrix = pose_with_covariance.covariance
            pose_val = pose_with_covariance.pose
            
            # Extract data from the Pose message
            position = pose_val.position
            orientation = pose_val.orientation
            x = orientation.x
            y = orientation.y
            z = orientation.z
            w = orientation.w
            # # Log the received pose data
            # self.get_logger().info('Received pose message inside Process Velocity:')
            # self.get_logger().info('Header: %s' % header)
            # self.get_logger().info('Position: x=%f, y=%f, z=%f' % (position.x, position.y, position.z)) # in ground frame
            # self.get_logger().info('Orientation: x=%f, y=%f, z=%f, w=%f' % (x, y, z, w)) # quaternion in ground frame
            # roll, pitch, yaw = self.euler_from_quaternion(x,y,z,w) # euler angles in ground frame
            # self.get_logger().info('Covariance Matrix: %s' % covariance_matrix) # in ground frame, we don't use "badhu saru thai jashe" this value now
            # Publish a TwistStamped message in robot frame
            twist_msg = TwistStamped()
            twist_msg.twist.linear.x = 5 # m/s
            twist_msg.twist.linear.y = position.y # dont'care  
            twist_msg.twist.linear.z = position.z # dont'care  
            twist_msg.twist.angular.x = orientation.x # dont'care   
            twist_msg.twist.angular.y = orientation.y # dont'care
            twist_msg.twist.angular.z = 10 # fill this thats it ------------------------------------------------------ 
            self.twist_publisher.publish(twist_msg)


#could be helpful from here onwards
    def route_callback(self, msg):#extracting route data
        self.route = msg #in an array form
        self.get_logger().info('Received route message inside Route Callback: %s' % self.route) 
        for i in (self.route.len()): # sequentially go to each node
            # header = msg.header
            node_x = msg.points[i].x # x pos of planting location in ground frame
            node_y = msg.points[i].y # y pos of planting location in ground frame
            node_z = msg.points[i].z # z pos of planting location in ground frame
            self.process_vel(node_x,node_y, node_z, self.pose)#ignore this function


    def pose_callback(self,msg):#extracting pose data
        self.pose = msg #in posewithcovariance pose
        header = msg.header #seq, stamp, frame_id
        pose_with_covariance = msg.pose        
        # Extract data from the PoseWithCovariance message
        covariance_matrix = pose_with_covariance.covariance
        # Extract data from the Pose message
        position = pose_with_covariance.pose.position
        # orientation = pose.orientation
        x = pose_with_covariance.pose.orientation.x
        y = pose_with_covariance.pose.orientation.y
        z = pose_with_covariance.pose.orientation.z
        w = pose_with_covariance.pose.orientation.w
        # Log the received pose data
        self.get_logger().info('Received pose message inside Pose Callback:')
        self.get_logger().info('Header: %s' % header)
        self.get_logger().info('Position: x=%f, y=%f, z=%f' % (position.x, position.y, position.z)) # in ground frame
        self.get_logger().info('Orientation: x=%f, y=%f, z=%f, w=%f' % (x, y, z, w)) # quaternion in ground frame
        roll, pitch, yaw = self.euler_from_quaternion(x,y,z,w)
        self.get_logger().info('Covariance Matrix: %s' % covariance_matrix) # in ground frame, we don't care right now about this value

        self.process_vel(self.route, self.pose)
        
    def euler_from_quaternion(x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
     
        return roll_x, pitch_y, yaw_z # in radians
    
    def next_node(self,route_completed):
        for i in (route_completed.len()): # sequentially go to each node
            # header = msg.header
            node_x = route_completed.points[i].x # x pos of planting location in ground frame
            node_y = route_completed.points[i].y # y pos of planting location in ground frame
            node_z = route_completed.points[i].z # z pos of planting location in ground frame
            # self.process_vel(node_x,node_y, node_z, self.pose)



    
    



def main(args=None):
    rclpy.init(args=args)
    motion_controller = MotionController()
    rclpy.spin(motion_controller)
    motion_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
