#!/usr/bin/env python

import rospy, math
from geometry_msgs.msg import Twist, Quaternion, Point, Pose, Vector3, Vector3Stamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Int16
from sensor_msgs.msg import Imu
import tf

import serial
import sys
import numpy as np

class Arduino():

    def __init__(self):
        #self.lock = lock
        self.serial = serial.Serial('/dev/mowpi_arduino', baudrate=115200, timeout=2)
        
    def safe_write(self, val_str):
        #self.lock.acquire()
        self.serial.write(val_str)
        #self.lock.release()

    def safe_read(self):
        #self.lock.acquire()
        val_read_str = self.serial.readline()
        #self.lock.release()
        return val_read_str

class Controller():
    
    def __init__(self, ard):
        self.ard = ard
        self.speed = 0.0
        self.curv = 0.0

    def write_speed_curv(self, speed_in, curv_in):
        #  A1/1/<speed_byte> <curv_byte>
        self.speed = speed_in # m/s
        self.curv = curv_in # rad/sec
        #print('speed, curv', speed_in, curv_in)
        raw_speed = int(speed_in*100.0)+120
        raw_omega = int(curv_in*180.0/3.14159)+120
        speed_byte = bytes([ raw_speed & 0xff]) #-120 to 120 cm/sec
        curv_byte = bytes([ (raw_omega) & 0xff]) # -120 to 120 deg/sec
        #print('speed_byte, curv_byte', speed_byte, curv_byte)
        
        self.ard.safe_write('A1/1/' + str(raw_speed) + '/' + str(raw_omega) + '/')
        return

class MicroBridge():
    
    def __init__(self):
        
        rospy.init_node('mowpi_ros_comm')
        
        rospy.Subscriber('cmd_vel', Twist, self.drive_callback, queue_size=1)
        rospy.Subscriber('blade_cmd', Int16, self.blade_callback, queue_size=2)
        rospy.Subscriber('imu', Imu, self.imu_callback, queue_size=2)
        
        self.odom_pub = rospy.Publisher('odom', Odometry, queue_size=5)
        self.odom_broadcaster = tf.TransformBroadcaster()
        
        self.ard = Arduino()
        self.controller = Controller(self.ard)
        self.speed = 0.0
        self.curv = 0.0
        self.prev_time = rospy.Time.now()

        self.gyroz_rad = 0.0

        self.enc_total = 0
        self.roll_rad = 0
        self.pitch_rad = 0
        self.blade_status = -1
        
        self.dist_sum = 0
        self.time_sum = 0
        self.vx = 0

        self.bot_deg = 0
        self.botx = 0
        self.boty = 0
        
        self.gyro_sum = 0.0
        self.gyro_count = 0
        self.gyro_bias_rad = 0.0
        
    def drive_callback(self, data):
        v = data.linear.x
        w = data.angular.z
        max_speed = 1.2
        max_omega = 120*3.14159/180.0
        
        if v > max_speed:
            v = max_speed
        elif v < -max_speed:
            v = -max_speed
        
        if w > max_omega:
            w = max_omega
        elif w < -max_omega:
            w = -max_omega
        
        self.speed = v
        self.curv = w
        
        if True or self.speed <> self.controller.speed or self.curv <> self.controller.curv:
            self.controller.write_speed_curv(self.speed, self.curv)
            
        #print("enc_total: ")
        #print(self.enc_total)
        #print('roll rad: ',self.roll_rad, ', pitch rad: ',self.pitch_rad)
    
    def blade_callback(self,bld):
        bld_cmd = bld.data
        bld_str = 'A2/8/' + str(bld_cmd) + '/'
        print("pre cmd: ", bld_str)
        if(bld_cmd == 1 and self.blade_status == 0):
            self.ard.safe_write(bld_str)
            self.blade_status = 1
            print("blade cmd: ", bld_str)
        elif(bld_cmd == 0 and self.blade_status != 0):
            self.ard.safe_write(bld_str)
            self.blade_status = 0
            print("blade cmd: ", bld_str)
        
    def imu_callback(self, data):
        accx = data.linear_acceleration.x
        accy = data.linear_acceleration.y
        self.gyroz_rad = data.angular_velocity.z
        if(self.gyro_count < 100):
            self.gyro_count += 1
            self.gyro_sum += self.gyroz_rad
            if(self.gyro_count == 100):
                self.gyro_bias_rad = self.gyro_sum / self.gyro_count
                print('gyro bias deg: ', self.gyro_bias_rad*180/3.14)
    
    def update_odom(self):
        t2 = rospy.Time.now()
        t1 = self.prev_time
        dt = (t2-t1).to_sec()
        
        BOT_WIDTH = (28.0 * 2.54 / 100.0) #meters
        COUNTS_PER_METER = 157.0
        
        # Process gyro z
        gyro_thresh_dps = 0.04*180/3.14159
        g_bias_dps = self.gyro_bias_rad*180/3.14159
        MAX_DTHETA_GYRO_deg = 100.0
        
        gyroz_raw_dps = float(self.gyroz_rad) * 180.0 / 3.14159
        
        if(abs(gyroz_raw_dps-g_bias_dps) < gyro_thresh_dps):
            gz_dps = 0
            dtheta_gyro_deg = 0
        else:
            gz_dps = gyroz_raw_dps-g_bias_dps
            dtheta_gyro_deg = gz_dps*dt*1.002 #*375.0/360.0 #HACK, WHY!!??
            
        # Read encoder delta   
        try: 
            self.ard.safe_write('A3/4/')
            s = self.ard.safe_read()
            delta_enc_left = int(s)
            s = self.ard.safe_read()
            delta_enc_right = int(s)
        except:
            delta_enc_left = 0
            delta_enc_right = 0
            print 'enc error'
            print "Unexpected Error:", sys.exc_info()[0]
        finally:
            a=0
        
        # Update odom
        
        delta_enc_counts = (delta_enc_left + delta_enc_right)/2
        self.enc_total = self.enc_total + delta_enc_counts
        
        dmeters = float(delta_enc_left + delta_enc_right)/2.0 / COUNTS_PER_METER
        
        dtheta_enc_deg = float(delta_enc_right - delta_enc_left) / COUNTS_PER_METER / BOT_WIDTH * 180.0 / 3.1416

        if(abs(dtheta_gyro_deg) > MAX_DTHETA_GYRO_deg):
            print 'no gyro'
            dtheta_deg = dtheta_enc_deg
        else:
            #print 'use gyro'
            dtheta_deg = dtheta_gyro_deg

        #update bot position
        self.bot_deg = self.bot_deg + dtheta_deg
        dx = dmeters*np.cos(self.bot_deg*3.1416/180)
        dy = dmeters*np.sin(self.bot_deg*3.1416/180)
        self.botx = self.botx + dx
        self.boty = self.boty + dy
        #print 'bot x,y,deg: ', self.bot.botx, self.bot.boty, self.bot.bot_deg
        
        # update bot linear x velocity every 150 msec
        # need to use an np array, then push and pop, moving average
        self.dist_sum = self.dist_sum + dmeters
        self.time_sum = self.time_sum + dt
        if(self.time_sum > 0.15):
            self.vx = self.dist_sum / self.time_sum
            self.dist_sum = 0
            self.time_sum = 0
        
        #bot.botx*100,bot.boty*100,bot.bot_deg
        odom_quat = tf.transformations.quaternion_from_euler(0, 0, self.bot_deg*3.14/180.0)
        self.odom_broadcaster.sendTransform(
        (self.botx, self.boty, 0.),
        odom_quat,
        t2,
        "base_link",
        "odom"
        )
        
        odom = Odometry()
        odom.header.stamp = t2
        odom.header.frame_id = "odom"

        # set the position
        odom.pose.pose = Pose(Point(self.botx, self.boty, 0.), Quaternion(*odom_quat))

        # set the velocity
        odom.child_frame_id = "base_link"
        gz_dps = dtheta_deg / dt
        odom.twist.twist = Twist(Vector3(self.vx, 0, 0), Vector3(0, 0, gz_dps*3.14/180.0))

        # publish the message
        self.odom_pub.publish(odom)
        
        ##### USE IMU TO PUBLISH TRANSFORM BETWEEN LASER AND BASE
        accx = 0
        accy = 0
        br = tf.TransformBroadcaster()
        if(abs(accx) < 3 and abs(accy) < 3):
            try:
                roll_rad = math.asin(accx/9.81) + 0.004 #confirmed with turn-around cal on concrete using rqt_plot
                pitch_rad = math.asin(accy/9.81) -0.061 - 0.01 #-0.01rad pitch back lidar, #-0.075 for body-ground confirmed with turn-around cal on conrete using rqt_plot
            except:
                roll_rad = self.roll_rad
                pitch_rad = self.pitch_rad
                print('asin error for roll or pitch')
        else:
            roll_rad = self.roll_rad
            pitch_rad = self.pitch_rad
            print('accx,y above 3 m/s^2')
                
        self.roll_rad = 0.99*self.roll_rad + 0.01*roll_rad
        self.pitch_rad = 0.99*self.pitch_rad + 0.01*pitch_rad
        laser_quat = tf.transformations.quaternion_from_euler(-self.roll_rad, -self.pitch_rad, 3.14159) #- roll, -pitch b/c of 180 deg yaw
        br.sendTransform((0,0,0),laser_quat,t2,"laser","base_link")
        #####
        
        self.prev_time = t2
        #loop_time = (rospy.Time.now()-t2).to_sec()
        #print("Loop Time: ");
        #print(loop_time)

if __name__ == '__main__': 
  try:
    mowpi_bridge = MicroBridge()
    print("starting mowpi ros comm")
    
    #rospy.spin()
    
    r = rospy.Rate(20.0)
    while not rospy.is_shutdown():
        mowpi_bridge.update_odom()
        r.sleep()
    
  except rospy.ROSInterruptException:
    pass
