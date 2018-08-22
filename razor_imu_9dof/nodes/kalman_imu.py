#!/usr/bin/env python
import rospy
import numpy as np
#from geometry_msgs.msg import Vector3
from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion  
from tf.transformations import quaternion_from_euler

class Kalman(object):
    """docstring for Kalman"""
    def __init__(self, n_states, n_sensors):
        super(Kalman, self).__init__()
        self.n_states = n_states
        self.n_sensors = n_sensors

        self.x = np.matrix(np.zeros(shape=(n_states,1)))
        self.P = np.matrix(np.identity(n_states)) 
        self.F = np.matrix(np.identity(n_states))
        self.u = np.matrix(np.zeros(shape=(n_states,1)))
        self.H = np.matrix(np.zeros(shape=(n_sensors, n_states)))
        self.R = np.matrix(np.identity(n_sensors))
        self.I = np.matrix(np.identity(n_states))

        self.first = True

    def update(self, Z):
        '''Z: new sensor values as numpy matrix'''

        w = Z - self.H * self.x
        S = self.H * self.P * self.H.getT() + self.R
        K = self.P * self.H.getT() * S.getI()
        self.x = self.x + K * w
        self.P = (self.I - K * self.H) * self.P

    def predict(self):
        self.x = self.F * self.x + self.u
        self.P = self.F * self.P * self.F.getT()

class Subscriber(object):
    """docstring for Subscriber"""
    def __init__(self):
        super(Subscriber, self).__init__()
        rospy.init_node('imu_conv', anonymous=True)

        
        self.kalman = Kalman(n_states = 3, n_sensors = 3)
        self.kalman.H = np.matrix(np.identity(self.kalman.n_states))
        #self.kalman.P *= 10
        #self.kalman.R *= 0.01
        
        #self.pub_accel = rospy.Publisher('kalman/accelerometer', Vector3)
        self.pub_all = rospy.Publisher('kalman/all', Imu)
 
        rospy.Subscriber('imu_data', Imu, self.callback_accel)
        rospy.spin()

    def callback_accel(self, data):
        # print "received data: ", data
        
        euler_angle = euler_from_quaternion((data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w))
        


        Z = np.matrix([data.angular_velocity.x, data.angular_velocity.y, data.angular_velocity.z]).getT()
                       
                      
                       

        if self.kalman.first:
            self.kalman.x = Z
            self.kalman.first = False

        self.kalman.update(Z)
        self.kalman.predict()

        data_updated = Imu()
        
        data_updated.angular_velocity.x = self.kalman.x[0]
        data_updated.angular_velocity.y = self.kalman.x[1]
        data_updated.angular_velocity.z = self.kalman.x[2]
        print("=====================================================")
        print("angular v:",data.angular_velocity.x, data.angular_velocity.y, data.angular_velocity.z)
        print("after filter:",data_updated.angular_velocity.x, data_updated.angular_velocity.y, data_updated.angular_velocity.z) 

        '''
        data_updated.angular_velocity.x = data.angular_velocity.x
        data_updated.angular_velocity.y = data.angular_velocity.y
        data_updated.angular_velocity.z = data.angular_velocity.z
        '''
        '''
        data_updated.linear_acceleration.x = self.kalman.x[3]
        data_updated.linear_acceleration.y = self.kalman.x[4]
        data_updated.linear_acceleration.z = self.kalman.x[5]
        '''

        
        data_updated.linear_acceleration.x = data.linear_acceleration.x
        data_updated.linear_acceleration.y = data.linear_acceleration.y
        data_updated.linear_acceleration.z = data.linear_acceleration.z
        


        #q_updated = quaternion_from_euler(self.kalman.x[6], self.kalman.x[7], self.kalman.x[8])
        q_updated = quaternion_from_euler(euler_angle[0], euler_angle[1], euler_angle[2])
        data_updated.orientation.x = q_updated[0]
        data_updated.orientation.y = q_updated[1]
        data_updated.orientation.z = q_updated[2]
        data_updated.orientation.w = q_updated[3]

        data_updated.orientation.x = data.orientation.x
        data_updated.orientation.y = data.orientation.y
        data_updated.orientation.z = data.orientation.z
        data_updated.orientation.w = data.orientation.w

        
        data_updated.header.stamp= rospy.Time.now()
        data_updated.header.frame_id = 'base_imu_link'
        data_updated.header.seq = data.header.seq
        
        
        self.pub_all.publish(data_updated)



if __name__ == '__main__':
    subscriber = Subscriber()


