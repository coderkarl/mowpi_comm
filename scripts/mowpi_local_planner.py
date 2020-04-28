#!/usr/bin/env python

import rospy, math
import numpy as np
from geometry_msgs.msg import Twist
from std_msgs.msg import Int16
from nav_msgs.msg import Odometry, Path
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import tf
from sensor_msgs.msg import LaserScan #force robot to at least slow down, stop, backup when close to obstacles

from DiffDriveController import DiffDriveController

class PathController():
    def __init__(self):
        rospy.init_node('path_controller')
        self.cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        #rospy.Subscriber('cmd_vel', Twist, self.drive_callback, queue_size=1)
        rospy.Subscriber('auto_mode', Int16, self.auto_mode_callback, queue_size = 1)
        rospy.Subscriber('odom', Odometry, self.odom_callback, queue_size = 1)
        rospy.Subscriber('/move_base/GlobalPlanner/plan', Path, self.path_callback, queue_size = 1)
        rospy.Subscriber('/scan', LaserScan, self.scan_callback, queue_size = 1)
        #<remap from="topic_a_temp" to="/ns1/topic_a">
        
        self.auto_mode = 1700
        self.vx = 0.0
        self.cum_err = 0
        
        MAX_SPEED = 1.0
        MAX_OMEGA = 1.2
        MIN_OMEGA = 0.5
        self.diff_drive_controller = DiffDriveController(MAX_SPEED, MAX_OMEGA, MIN_OMEGA)
        
        x_i = 0.
        y_i = 0.
        theta_i = -99.
        self.state = np.array([[x_i], [y_i], [theta_i] ])
        self.goal = np.array([[x_i], [y_i] ])
        self.goal_reached = True
        
        self.waypoints = []
        self.v = 0.0
        self.w = 0.0
        
        self.tf_listener = tf.TransformListener()
        self.reverse_flag = False
        self.avoid_obs_dir = -1;
        
        self.scan_avoid_min_index = None
        self.scan_avoid_max_index = None
    
    def auto_mode_callback(self, data):
        self.auto_mode = data
        
    def scan_callback(self, data):
        if(self.scan_avoid_min_index == None):
            fov_clear_range_deg = 30. #FOV in deg to look for near-collision obstacles
            left_avoid_angle = fov_clear_range_deg/2. * math.pi/180.
            right_avoid_angle = fov_clear_range_deg/2. * math.pi/180.
            #min_angle = data.angle_min
            #max_angle = data.angle_max
            # TODO ensure min, max angle are +/- 180 deg (not 0 to 360)
            
            self.scan_avoid_min_index = int(math.floor((right_avoid_angle - data.angle_min)/data.angle_increment))
            self.scan_avoid_max_index = int(math.ceil((left_avoid_angle - data.angle_min)/data.angle_increment))
        
        ranges = data.ranges

        #min_range = min(ranges[self.scan_avoid_min_index:self.scan_avoid_max_index]) #5:11 for LeddarM16
        if( (self.vx >= 0 and not self.reverse_flag) or (self.vx < 0 and self.reverse_flag) ):
            min_range1 = min(ranges[330:359])
            min_range2 = min(ranges[0:30])
        else:
            min_range1 = min(ranges[150:179])
            min_range2 = min(ranges[180:210])
        min_range = min([min_range1, min_range2])
        
        # Oscillates back and forth, not turning much
        #   Consider slowly reversing to give the steering angle more time
        if(min_range < 1.2 and not self.reverse_flag):
            print('CHANGE DIRECTION TO AVOID OBS')
            self.reverse_flag = True
            self.avoid_obs_dir = -np.sign(self.vx)
        elif(self.reverse_flag and min_range > 3.5):
            print('CLEAR CHANGE DIRECTION TO AVOID OBS')
            self.reverse_flag = False
    
    def odom_callback(self,odom):
        # This odom pose is in the odom frame, we want the pose in the map frame
        #   Now done in execute_plan() at a fixed rate using a tf.TransformListener()
        #~ quat = odom.pose.pose.orientation
        #~ quat_list = [quat.x, quat.y, quat.z, quat.w]
        #~ (roll, pitch, yaw) = euler_from_quaternion (quat_list)
        
        #~ self.state[0] = odom.pose.pose.position.x + 5.0
        #~ self.state[1] = odom.pose.pose.position.y
        #~ self.state[2] = yaw
        
        self.vx = odom.twist.twist.linear.x
    
    def path_callback(self,data):
        poses = data.poses
        nPose = len(poses)
        self.waypoints = []
        if(nPose > 0):
            wp = np.zeros([2,1])
            if(nPose == 1):
                init = 0
                wp_step = 1
            else:
                # approximate actual path length by breaking into 4 sub-linesegments
                path_dist = 0
                nSegments = 4
                stride = max(int(nPose/nSegments), 1)
                ind1 = 0
                ind2 = 0
                done_flag = False
                k = 0
                while(k <= nSegments):
                    ind2 += stride
                    if(ind2 >= nPose):
                        ind2 = nPose -1
                        done_flag = True
                    dx = poses[ind2].pose.position.x - poses[ind1].pose.position.x
                    dy = poses[ind2].pose.position.y - poses[ind1].pose.position.y
                    path_dist += np.sqrt(dx**2 + dy**2)
                    if(done_flag):
                        print "reached final point for path_dist"
                        break
                    ind1 = ind2
                    k += 1
                
                step_size_meters = 2.0    
                wp_step = int(step_size_meters/path_dist * nPose)
                if(wp_step >= nPose):
                    wp_step = 1
                init = wp_step
                print "path dist: ", path_dist
                print "wp step: ", wp_step
            for k in xrange(init,nPose,wp_step):
                wp[0] = poses[k].pose.position.x
                wp[1] = poses[k].pose.position.y
                self.waypoints.append(wp.copy())
            if ( not k==nPose-1):
                wp[0] = poses[-1].pose.position.x
                wp[1] = poses[-1].pose.position.y
                self.waypoints.append(wp.copy())
            
            #print "waypoints: ", self.waypoints
            self.goal = self.waypoints.pop(0)
            print "initial goal: ", self.goal
            
            self.goal_reached = False
        else:
            self.goal_reached = True
            self.waypoints = []
            
        
    def execute_plan(self):
        # Update robot pose state from tf listener
        try:
            (trans,quat) = self.tf_listener.lookupTransform('/map', '/base_link', rospy.Time(0))
            self.state[0] = trans[0]
            self.state[1] = trans[1]
            #quat_list = [quat.x, quat.y, quat.z, quat.w]
            (roll, pitch, yaw) = euler_from_quaternion (quat)
            self.state[2] = yaw
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass
        
        # Local planner (no dynamic obstacle avoidance, hopefully to be done by global planner called frequently)
        speed_cmd_updated = False
        if(not self.goal_reached):
            raw_v,self.w,self.goal_reached, alpha, pos_beta = self.diff_drive_controller.compute_vel(self.state,self.goal)
            self.v = self.v*0.9+raw_v*0.1
            speed_cmd_updated = True
            #if(self.goal_reached):
                #print "wp goal reached"
                #print "v: ", v
                #print "w: ", w
                #print "state: ", self.state
                #print "goal: ", self.goal
        elif( len(self.waypoints) > 0 and (self.goal_reached) ): # or abs(alpha) > 3.14/2) ):
            self.goal = self.waypoints.pop(0)
            #print "wp goal: ", self.goal
            self.goal_reached = False
        else:
            self.v = 0.
            self.w = 0.
            
        # Don't simply reverse if an obstacle triggers reverse_flag
        #    Need to change directions
        if(self.reverse_flag and speed_cmd_updated):
            self.v = self.avoid_obs_dir*0.9
            #self.w = 0.0
        
        #PI control for desired linear speed v
        # controller will output an offset command to add to v
        Ks = 2.5
        Ki = 1.5
        Kp = 1.5
        err = self.v - self.vx
        self.cum_err += err
        self.cum_err = min(1.0, self.cum_err)
        self.cum_err = max(-1.0, self.cum_err)
        out = Kp*err + Ki*self.cum_err
        
        twist = Twist()
        twist.linear.x = self.v #Ks*v+out
        twist.angular.z = self.w
        self.cmd_pub.publish(twist)
        
        #if(self.auto_mode > 1600):
        #self.cmd_pub.publish(spdCrv)

if __name__ == '__main__':
    try:
        path_control = PathController()
        rospy.loginfo("Starting Wheele Path Controller")
        #rospy.spin()
        r = rospy.Rate(20.0)
        while not rospy.is_shutdown():
            path_control.execute_plan()
            r.sleep()
            
    except rospy.ROSInterruptException:
        pass
