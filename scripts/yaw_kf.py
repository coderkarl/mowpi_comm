#!/usr/bin/env python

import rospy, math
from geometry_msgs.msg import Twist, Quaternion, Point, Pose, Vector3, Vector3Stamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Int16, Float32
from sensor_msgs.msg import Imu, MagneticField
import tf

import sys
import numpy as np

from collections import deque #a = deque(maxlen=4); deque.append(x) will pop the old element, can use appendleft, count(a)

pi = 3.14159
gyro_buf_size = 5
mag_buf_size = 5

class YawKF():
    
    def __init__(self):
        
        rospy.init_node('yaw_kf')
        
        rospy.Subscriber('imu', Imu, self.imu_callback, queue_size=2)
        rospy.Subscriber('mag', MagneticField, self.mag_callback, queue_size=2)
        
        self.yaw_pub = rospy.Publisher('yawkf_deg', Float32, queue_size=5)
        self.mag_yaw_pub = rospy.Publisher('yaw_mag_deg', Float32, queue_size=5)
        self.odom_broadcaster = tf.TransformBroadcaster()
        
        self.prev_imu_time = None

        self.gyroz_rad = 0.0

        self.roll_rad = 0
        self.pitch_rad = 0
        
        self.gyro_bias_rad = 0.06*pi/180
        
        self.gyro_rad_buf = deque(maxlen=gyro_buf_size)
        
        self.yaw_rad = 0.0
        self.mag_yaw_meas = 0.0
        
        # initial yaw sigma = 2 deg
        # initial gyro bias sig = 0.1 deg/sec
        init_yaw_var = (2*pi/180)**2
        init_bias_var = (0.1*pi/180)**2
        self.P = np.matrix([[init_yaw_var, 0.], [0., init_bias_var]])
        self.state = np.matrix([[self.yaw_rad], [self.gyro_bias_rad]])
        
        self.A = np.matrix([[1., -0.01], [0., 1.]])
        gyro_Q_error = 0.05*pi/180 # error per sec growth
        bias_Q_error = 0.05*pi/180 # error per sec growth
        self.Q = np.matrix([[gyro_Q_error**2, 0.], [0., bias_Q_error**2]])
        
        mag_yaw_error = 15*pi/180
        self.R = mag_yaw_error**2
        
        self.K = np.matrix([[0.],[0.]])
        self.H = np.matrix([1.,0])
        self.I = np.matrix([[1.,0.],[0.,1.]])
        
        self.mag_update = False
        self.yaw_initialized = False
        self.initial_mag_yaw = 0.0
        self.mag_x_buf = deque(maxlen=mag_buf_size)
        self.mag_y_buf = deque(maxlen=mag_buf_size)
        
        magx_min = 18.0
        magx_max = 53.0
        self.cx = (magx_min + magx_max)/2
        self.rx = (magx_max - magx_min)/2
        magy_min = -46.0
        magy_max = -11.0
        self.cy = (magy_min + magy_max)/2
        self.ry = (magy_max - magy_min)/2
        
        self.low_gyro_thresh_rad = 0.2
        self.low_gyro_std_thresh = 0.1
        
        print 'time,yaw_deg,bias,sigY,sigG,mag_deg,e'
        
    def imu_callback(self, data):
        tNow = data.header.stamp
        self.accx = data.linear_acceleration.x
        self.accy = data.linear_acceleration.y
        self.gyroz_rad = data.angular_velocity.z
        if abs(self.gyroz_rad) < self.low_gyro_thresh_rad:
            self.gyro_rad_buf.append(self.gyroz_rad)
        else:
            self.gyro_rad_buf.clear()
        
        if abs(self.gyroz_rad) > 0.03:
            if not (self.prev_imu_time is None):
                dt = (tNow - self.prev_imu_time).to_sec()
                #print dt
                self.A[0,1] = -dt
                self.state[0] = self.state[0] + (self.gyroz_rad - self.state[1]) * dt
                if(self.state[0] > pi):
                    self.state[0] = self.state[0] - 2*pi
                elif(self.state[0] <= -pi):
                    self.state[0] = self.state[0] + 2*pi
                self.yaw_rad = self.state[0]
                self.gyro_bias_rad = self.state[1]
                self.P = self.A * self.P * self.A.T + self.Q*dt*dt
            
        if self.mag_update:
            mag_output = self.mag_yaw_meas - self.initial_mag_yaw
            mag_output = math.atan2(np.sin(mag_output), np.cos(mag_output))
            e = self.mag_yaw_meas - self.initial_mag_yaw - self.yaw_rad
            e = math.atan2(np.sin(e), np.cos(e))
            
            if abs(e) < 4*np.sqrt(self.P[0,0]):
                S = self.P[0,0] + self.R
                self.K[0] = self.P[0,0] / S
                self.K[1] = self.P[1,0] / S
                self.state = self.state + self.K*e
                if(self.state[0] > pi):
                    self.state[0] = self.state[0] - 2*pi
                elif(self.state[0] <= -pi):
                    self.state[0] = self.state[0] + 2*pi
                self.yaw_rad = self.state[0]
                
                self.gyro_bias_rad = self.state[1]
                self.P = (self.I - self.K*self.H)*self.P
            self.mag_update = False
            print '%f,%f,%f,%f,%f,%f,%f' % (tNow.to_sec(),self.yaw_rad*180/pi, self.gyro_bias_rad*180/pi, np.sqrt(self.P[0,0])*180/pi, np.sqrt(self.P[1,1])*180/pi, mag_output*180/pi, e*180/pi)
        else:
            aa = 0
            print '%f,%f,%f,%f,%f,%f,%f' % (tNow.to_sec(),self.yaw_rad*180/pi, self.gyro_bias_rad*180/pi, np.sqrt(self.P[0,0])*180/pi, np.sqrt(self.P[1,1])*180/pi, 999, 999)
        
        self.prev_imu_time = tNow
        
        yaw_msg = Float32()
        yaw_msg.data = self.yaw_rad*180/pi
        self.yaw_pub.publish(yaw_msg)
        
    def mag_callback(self, data):
        if (not self.mag_update) and len(self.gyro_rad_buf) >= gyro_buf_size:
            magx = data.magnetic_field.x
            magy = data.magnetic_field.y
            mpx = (magx - self.cx)/self.rx
            mpy = (magy - self.cy)/self.ry
            self.mag_x_buf.append(mpx)
            self.mag_y_buf.append(mpy)
            if len(self.mag_x_buf) == mag_buf_size:
                if self.yaw_initialized:
                    self.mag_update = True
                    mpx = np.mean(self.mag_x_buf)
                    mpy = np.mean(self.mag_y_buf)
                    self.mag_yaw_meas = -math.atan2(mpy,mpx)
                    
                    mag_yaw_msg = Float32()
                    mag_yaw_value = self.mag_yaw_meas - self.initial_mag_yaw
                    mag_yaw_value = math.atan2(np.sin(mag_yaw_value), np.cos(mag_yaw_value))
                    mag_yaw_msg.data = mag_yaw_value*180/pi
                    self.mag_yaw_pub.publish(mag_yaw_msg)
                else:
                    self.yaw_initialized = True
                    mpx = np.mean(self.mag_x_buf)
                    mpy = np.mean(self.mag_y_buf)
                    self.initial_mag_yaw = -math.atan2(mpy,mpx)
                
                self.mag_x_buf.clear()
                self.mag_y_buf.clear()
        else:
            self.mag_x_buf.clear()
            self.mag_y_buf.clear()

if __name__ == '__main__': 
  try:
    yaw_kf = YawKF()
    print("starting yaw kf")
    
    #rospy.spin()
    
    r = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        r.sleep()
    
  except rospy.ROSInterruptException:
    pass
