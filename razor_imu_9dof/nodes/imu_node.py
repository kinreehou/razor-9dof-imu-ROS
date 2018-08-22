#!/usr/bin/env python

import rospy
import serial
import string
import math
from sensor_msgs.msg import Imu
# from tf.transformations import euler_from_quaternion
# from tf.transformations import quaternion_from_euler

degrees2rad = math.pi/180.0
g2ms = 9.81
imuMsg = Imu()

#no Magnetometer readings
class IMU():
    def __init__(self,imu_params):
        self.time = imu_params[0]
        self.linear_acceleration_x = imu_params[1]
        self.linear_acceleration_y = imu_params[2]
        self.linear_acceleration_z = imu_params[3]
        self.angular_velocity_x = imu_params[4]
        self.angular_velocity_y = imu_params[5]
        self.angular_velocity_z = imu_params[6]
        self.orientation_w = imu_params[7]
        self.orientation_x = imu_params[8]
        self.orientation_y = imu_params[9]
        self.orientation_z = imu_params[10]
        self.angle_x = imu_params[11]
        self.angle_y = imu_params[12]
        self.angle_z = imu_params[13]
      

def calibrate_linear_accel(N):
    calibrate_trial = N
    sum_lax = 0
    sum_lay = 0
    sum_laz = 0
        
    with serial.Serial('/dev/ttyACM0', 57600) as ser:
        line = ser.readline()
        for i in range(calibrate_trial):
            line= ser.readline()
            calib_data = [float(x) for x in str(line.decode('utf-8')).split(',')]
            imu = IMU(calib_data)
            sum_lax += imu.linear_acceleration_x
            sum_lay += imu.linear_acceleration_y
            sum_laz += imu.linear_acceleration_z
                
        accel_offset = [sum_lax/(N*1.0), sum_lay/(N*1.0), sum_laz/(N*1.0)]
        return accel_offset

def calibrate_angular_velocity(N):
    calibrate_trial = N
    sum_avx = 0
    sum_avy = 0
    sum_avz = 0
        
    with serial.Serial('/dev/ttyACM0', 57600) as ser:
        line = ser.readline()
        for i in range(calibrate_trial):
            line= ser.readline()
            calib_data = [float(x) for x in str(line.decode('utf-8')).split(',')]
            imu = IMU(calib_data)
            sum_avx += imu.angular_velocity_x
            sum_avy += imu.angular_velocity_y
            sum_avz += imu.angular_velocity_z               
        angular_vel_offset = [sum_avx/(N*1.0), sum_avy/(N*1.0), sum_avz/(N*1.0)]
        return angular_vel_offset

def calibrate_orientation(N):
    calibrate_trial = N
    sum_ow = 0
    sum_ox = 0
    sum_oy = 0
    sum_oz = 0
        
    with serial.Serial('/dev/ttyACM0', 57600) as ser:
        line = ser.readline()
        for i in range(calibrate_trial):
            line= ser.readline()
            calib_data = [float(x) for x in str(line.decode('utf-8')).split(',')]
            imu = IMU(calib_data)
            sum_ow += imu.orientation_w
            sum_ox += imu.orientation_x
            sum_oy += imu.orientation_y
            sum_oz += imu.orientation_z             
        orientation_offset = [sum_ow/(N*1.0), sum_ox/(N*1.0), sum_oy/(N*1.0), sum_oz/(N*1.0)]
        return orientation_offset

def calibrate_euler(N):
    calibrate_trial = N
    sum_angx = 0
    sum_angy = 0
    sum_angz = 0                          
    with serial.Serial('/dev/ttyACM0', 57600) as ser:
        line = ser.readline()
        for i in range(calibrate_trial):
            line= ser.readline()
            calib_data = [float(x) for x in str(line.decode('utf-8')).split(',')]
            imu = IMU(calib_data)
            quaternion = (
                imu.orientation_x,
                imu.orientation_y,
                imu.orientation_z,
                imu.orientation_w)
            euler_angle = euler_from_quaternion(quaternion)
            sum_angx += euler_angle[0]
            sum_angy += euler_angle[1]
            sum_angy += euler_angle[2]
        euler_offset = [sum_angx/(N*1.0), sum_angy/(N*1.0), sum_angz/(N*1.0)]
        return euler_offset

    
rospy.init_node('sparkfun_imu', anonymous=True)
pub = rospy.Publisher('imu_data', Imu, queue_size=1)
rate = rospy.Rate(10) # 10hz
seq = 0

with serial.Serial('/dev/ttyACM0', 57600) as ser:
    line = ser.readline()
    #calibration
    accel_offset = calibrate_linear_accel(500)
    angular_vel_offset = calibrate_angular_velocity(500)
    orientation_offset = calibrate_orientation(500)
    #euler_offset =calibrate_euler(500)
    
    print("calibration is done!!!")
    print("acceleration offset:", accel_offset)
    print("angular velocity offset:", angular_vel_offset)
    print("orientation offset:", orientation_offset)
    #print("euler angle offset:", euler_offset)
    
    while not rospy.is_shutdown():
        line = ser.readline()
        data = [float(x) for x in str(line.decode('utf-8')).split(',')]
        imu = IMU(data)
        

        #orientation euler
        # imu_euler_angle = euler_from_quaternion((imu.orientation_x, imu.orientation_y, imu.orientation_z, imu.orientation_w))
        # orientation_offseted = quaternion_from_euler(imu_euler_angle[0]-euler_offset[0], imu_euler_angle[1]-euler_offset[1], imu_euler_angle[2]-euler_offset[2])
        # imuMsg.orientation.x = orientation_offseted[0]
        # imuMsg.orientation.y = orientation_offseted[1]
        # imuMsg.orientation.z = orientation_offseted[2]
        # imuMsg.orientation.w = orientation_offseted[3]
   

        model = [1,0,0,0]
        #orientatation quaternion   
        imuMsg.orientation.w = imu.orientation_w - orientation_offset[0] + model[0]
        imuMsg.orientation.x = imu.orientation_x - orientation_offset[1] + model[1]
        imuMsg.orientation.y = imu.orientation_y - orientation_offset[2] + model[2]
        imuMsg.orientation.z = imu.orientation_z - orientation_offset[3] + model[3]
       
        
        #angular velocity
        imuMsg.angular_velocity.x = (imu.angular_velocity_x - angular_vel_offset[0])*degrees2rad 
        imuMsg.angular_velocity.y = (imu.angular_velocity_x - angular_vel_offset[1])*degrees2rad
        imuMsg.angular_velocity.z = (imu.angular_velocity_x - angular_vel_offset[2])*degrees2rad
        
        
        #linear_acceleration
        imuMsg.linear_acceleration.x = (imu.linear_acceleration_x - accel_offset[0])*g2ms
        imuMsg.linear_acceleration.y = (imu.linear_acceleration_x - accel_offset[1])*g2ms        
        imuMsg.linear_acceleration.z = (imu.linear_acceleration_x - accel_offset[2])*g2ms

        
        imuMsg.header.stamp= rospy.Time.now()
        imuMsg.header.frame_id = 'base_imu_link'
        imuMsg.header.seq = seq
        seq = seq + 1
        
        pub.publish(imuMsg)
        #print("========================================================================================")
        #print("seq:",seq)
        #print("angular v:",imuMsg.angular_velocity.x,imuMsg.angular_velocity.y,imuMsg.angular_velocity.z)
        #print("linear_accel",imuMsg.linear_acceleration.x, imuMsg.linear_acceleration.y, imuMsg.linear_acceleration.z)
        print("orientation in quaternion:", imuMsg.orientation.w, imuMsg.orientation.x, imuMsg.orientation.y, imuMsg.orientation.z)
        
            
    
    
    

