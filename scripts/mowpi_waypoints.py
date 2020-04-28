#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import tf2_ros
import time
import numpy as np

class MowPath():
    def __init__(self):
        rospy.init_node('mow_path')
        time.sleep(0.5)
        #rospy.Subscriber('odom', Odometry, self.odom_callback, queue_size=2)
        rospy.Subscriber('/move_base/GlobalPlanner/plan', Path, self.path_callback, queue_size = 1)
        
        self.nav_goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)
        #/move_base_simple/goal geometry_msgs/PoseStamped
        #'{ header: {stamp: now, frame_id: "map"}, pose: { position: {x: 10.0, y: 30.0, z: 0.0}, orientation: {w: 1.0}}}'

        
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        time.sleep(1.0)
        rospy.loginfo("Mow Path Initialized")
        
        wp_list = []
        go_behind_garage = False
        go_front_west = False
        go_front_east = False
        
        if(go_behind_garage):
            wp_list.append([2.0, 0.0])
            wp_list.append([4.0, 12.5])
            wp_list.append([-8.5, 12.5])
            wp_list.append([-11.5, 6.0])
            #wp_list.append([-10.5, 10.5]) #pause at the corner to update slam map
            #wp_list.append([-10.5, 10.5]) #if same waypoint, pause before updating to next
            
            # go to east of house
            wp_list.append([-12.0, -9.0])
            self.return_ind = 4 #4
        elif(go_front_west):
            wp_list.append([2.0, 0.0])
            wp_list.append([3.0, 4.0])
            self.return_ind = 1
        elif(go_front_east):
            wp_list.append([2.0, 0.0])
            wp_list.append([3.0, 4.0])
            wp_list.append([28,6])
            self.return_ind = 2
        else:
            wp_list.append([0.0, 0.0])
            self.return_ind = 0
        
        #general rectangle, start from bot pose
        mx1 = 1.0
        my1 = -6.0
        mx2 = 11.0
        my2 = 3.0
        
        #first corner, close behind garage, move along y
        #mx1 = -12.5 #-10.0
        #my1 = 13.0 #14.0
        #mx2 = -20.0 #-18.0
        #my2 = -20.0 #-9.0
        # xstep = -0.25
        
        #first corner, close behind garage, small, move along y
        #mx1 = -10.5 #-10.0
        #my1 = 15.5 #14.0
        #mx2 = -18.0 #-18.0
        #my2 = -10.0 #-9.0
        
        #behind garage, farther south, y away from leaves/tree, move along y
        second_patch = False #change to mx3,4 my3,4
        #mx1 = -17.0 #-10.0
        #my1 = 9.0 #14.0
        #mx2 = -28.0 #-18.0
        #my2 = -8.0
        # xstep = -0.5
        
        # extend east from behind garage, move along y, shift south
        #mx1 = -10.5
        #my1 = -10.5
        #mx2 = -18.0
        #my2 = -30.0
        
        # extend east from behind garage, move along y, shift north
        #mx1 = -10.5
        #my1 = -10.5
        #mx2 = 2.0
        #my2 = -30.0
        
        #mx1 = 1.0
        #my1 = -21.0
        #mx2 = 15.0
        #my2 = -32.0
        # xstep = 0.5, move_along_y = True
        
        # far east of garage, move along x, need to adjust x coordinates or xtrans to avoid goal at tree obs cell
        #mx1 = -12.0
        #my1 = -14.0
        #mx2 = 1.5
        #my2 = -30.0
        #mx1 = 0.0
        #my1 = -24.0
        #mx2 = -30.0
        #my2 = -29.5
        #ystep = -0.5, move_along_x = True, y>=my2, mx1-xtrans
        
        # east of house, move along x
        #mx1 = 3.5
        #my1 = -18.5
        #mx2 = 12.5
        #my2 = -31.0
        #ystep = -0.5, move_along_x = True, y>=my2, mx1-xtrans
        
        
        # front west yard, move along x
        #mx1 = 9.0 # 10
        #my1 = 7.0 # 7.5
        #mx2 = 30 #0, 28
        #my2 = 12.0 #-20, 12.5
        
        # front east yard 1 move along y
        #mx1 = 36
        #my1 = 3
        #mx2 = 26
        #my2 = -17
        
        #second_patch = False
        
        # front east yard 2 move along x
        #mx1 = 34
        #my1 = -5
        #mx2 = 25
        #my2 = -15
        
        xtrans = 1.0
        ytrans = 1.0
        xstep = 0.35
        ystep = 0.35
        x = mx1
        y = my1
        
        use_front_loop = False
        move_along_x = True
        move_along_y = False
        wp_list.append([x, y])
        
        if(move_along_x):
            while(y <= my2):
                wp_list.append([mx2, y])
                y += ystep
                wp_list.append([mx2+xtrans,y])
                wp_list.append([mx1,y])
                y += ystep
                wp_list.append([mx1-xtrans,y])
        elif(move_along_y):
            while(x >= mx2): # switch between x <= mx2, x >= mx2 based on sign of xstep
                wp_list.append([x, my2])
                x += xstep
                wp_list.append([x,my2-ytrans])
                wp_list.append([x,my1])
                x += xstep
                wp_list.append([x,my1+ytrans])
            if(second_patch):
                x = mx3
                y = my3
                while(x >= mx4):
                    wp_list.append([x, my4])
                    x += xstep
                    wp_list.append([x,my4-ytrans])
                    wp_list.append([x,my3])
                    x += xstep
                    wp_list.append([x,my3+ytrans])
        else:
            cx = 30
            cy = -12
            obsx = [32.6, 25.9, 25.9, 25.8]
            obsy = [-4.7,-5.1, -13.1,-20.9]
            xlist = [33, 37, 37, 37, 35, 24, 23, 23, 20, 19, 25]
            ylist = [4,  4,  -1,-20,-22,-22,-20,-17,-16, -6, -2]
            for x,y in zip(xlist,ylist):
                wp_list.append([x,y])
            for k in range(8): #how many inner decreasing radius laps to mow
                for x,y in zip(xlist,ylist):
                    dx = (cx-x)
                    dy = (cy-y)
                    dmag = np.sqrt(dx**2+dy**2)
                    dx = dx/dmag*(k+1)*0.5
                    dy = dy/dmag*(k+1)*0.5
                    c=0
                    min_dist = 100.0
                    for ox,oy in zip(obsx,obsy):
                        dist = np.sqrt((x+dx-ox)**2+(y+dy-oy)**2)
                        min_dist = min(dist,min_dist)
                        if(dist < 1.2):
                            c += 1
                            #print('obs',x+dx,y+dy, dist)
                            #break
                    if(c==0):
                        print(x+dx, y+dy, min_dist)
                        wp_list.append([x+dx,y+dy])
        
        #wp_list.reverse()
        self.waypoints = wp_list
        self.num_wp = len(wp_list)
        self.wp_ind = 0
        self.wpx = self.waypoints[0][0]
        self.wpy = self.waypoints[0][1]
        
        self.state = 'init'
        self.near_thresh = 0.75
        self.noPath = False
        self.db_count_noPath = 0
        self.offset_dist = 2.0
        self.trying_offset = False
        
        next_wp = PoseStamped()
        next_wp.header.stamp = rospy.Time.now()
        next_wp.header.frame_id = "map"
        next_wp.pose.position.x = self.wpx
        next_wp.pose.position.y = self.wpy
        next_wp.pose.position.z = 0.0
        next_wp.pose.orientation.w = 1.0
        self.next_wp = next_wp
        
        self.nav_goal_pub.publish(next_wp)
        rospy.loginfo("Initial waypoint published")
        
    
    def near_wp(self, x, y, thresh = 0.3):
        dist_sqd = (x-self.wpx)**2 + (y-self.wpy)**2
        return dist_sqd < thresh**2
        
    def near_end(self, x, y, thresh = 1.0):
        dist_sqd = (x-self.mx2)**2 + (y-self.my2)**2
        return dist_sqd < thresh**2 #and self.wpy-self.my2 < 0.0
        
    def x_near1(self,x,thresh = 0.3):
        return abs(x-self.mx1) < thresh
        
    def x_near2(self,x,thresh = 0.3):
        return abs(x-self.mx2) < thresh
        
    def path_callback(self,data):
        nPose = len(data.poses)
        if(nPose == 0):
            self.db_count_noPath += 1
            if(self.db_count_noPath > 3):
                self.noPath = True
                if(self.trying_offset):
                    self.offset_dist += 1.0
                    if(self.offset_dist > 6.0):
                        self.offset_dist = 6.0
        else:
            self.db_count_noPath = 0
            self.noPath = False
            self.trying_offset = False
            self.offset_dist = 2.0
                
    def pub_waypoint(self, offset_x, offset_y):
        self.next_wp.pose.position.x = self.wpx + offset_x
        self.next_wp.pose.position.y = self.wpy + offset_y
        self.next_wp.header.stamp = rospy.Time.now()
        self.nav_goal_pub.publish(self.next_wp)
        rospy.loginfo("Next waypoint published")
        rospy.loginfo("x,y: %0.1f, %0.1f",self.wpx+offset_x, self.wpy+offset_y)
        
    def update_waypoint(self):
        try:
            self.transform = self.tf_buffer.lookup_transform("map","base_link", rospy.Time())
            #self.transform = self.tf_buffer.waitForTransform("map","laser", msg.header.stamp)
            
            tx = self.transform.transform.translation.x
            ty = self.transform.transform.translation.y
            quat = self.transform.transform.rotation
            quat_list = [quat.x, quat.y, quat.z, quat.w]
            (roll, pitch, yaw) = euler_from_quaternion(quat_list)
            #print "bot x,y,theta:", tx, ty, yaw
            
            if self.noPath:
                # Publish a waypoint offset, in the direction of the bot w.r.t. current waypoint
                dx = tx - self.wpx
                dy = ty - self.wpy
                dist = np.sqrt(dx**2 + dy**2)
                self.wpx += self.offset_dist * dx/dist
                self.wpy += self.offset_dist * dy/dist
                self.noPath = False
                self.trying_offset = True
                self.pub_waypoint(0, 0)
                print("TRYING OFFSET DIST", self.offset_dist)
                
            if self.near_wp(tx,ty, self.near_thresh) and (not self.state == 'done'):
                prev_x = self.wpx
                prev_y = self.wpy
                if(self.state == 'init'):
                    self.wp_ind += 1
                    if(self.wp_ind >= self.num_wp):
                        self.wp_ind = self.return_ind
                        # try doing full waypoints in reverse now
                        #self.wp_ind = self.num_wp-1
                        self.state = 'return'
                    self.wpx = self.waypoints[self.wp_ind][0]
                    self.wpy = self.waypoints[self.wp_ind][1]
                elif(self.state == 'return'):
                    self.wp_ind -= 1
                    if(self.wp_ind >= 0):
                        self.wpx = self.waypoints[self.wp_ind][0]
                        self.wpy = self.waypoints[self.wp_ind][1]  
                
                if(self.wpx == prev_x and self.wpy == prev_y):
                    time.sleep(2.0)
                    rospy.loginfo("Holding waypoint")
                
                self.pub_waypoint(0,0)
                rospy.loginfo("Mow State: %s",self.state)
        
        except Exception as e:
            print "tf issue"
            print repr(e)
            pass

if __name__ == '__main__':
    try:
        mow_path = MowPath()
        rospy.loginfo("Starting Mow Path")
        #rospy.spin()
        r = rospy.Rate(2.0)
        while not rospy.is_shutdown():
            mow_path.update_waypoint()
            r.sleep()
            
    except rospy.ROSInterruptException:
        pass
