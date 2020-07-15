
import rospy
from pid import PID
from lowpass import LowPassFilter

from yaw_controller import YawController

class Controller(object):

#Define a class for the calculation of throttle, brake, steering.


    def __init__(self, EgoParam):

        '''
        Claimed yaw controller for future use.
        Input for yaw controller:
            Vehicle's information of wheel base, steer ratio, maximum lateral acceleration, and maximum steer angle.
            Manually given minimum speed for angle calculation
        '''
        self.yaw_controller = YawController(
        wheel_base= EgoParam.wheel_base,
        steer_ratio = EgoParam.steer_ratio,
        min_speed=0.1,
        max_lat_accel= EgoParam.max_lat_accel,
        max_steer_angle= EgoParam.max_steer_angle)

        self.EgoParam=EgoParam

        '''
        Throttle controller call for function in pid.py
        return:
           throttle calculation functions
        '''
        self.throttle_controller = PID(kp=8,ki=0.5,kd=0.0,mn = EgoParam.decel_limit,mx=EgoParam.accel_limit)

        tau=0.5
        ts = 0.02

        self.vel_lpf = LowPassFilter(tau,ts)
        self.last_time = rospy.get_time()
        pass

    def control(self,current_vel,dbw_enabled,linear_vel):
        '''
        Args:
            current_vel (Current Velocity): data subscribed
            dbw_enabled (Whether enable dbw node): only matters when real test
            linear_vel (Linear velocity): velocity move ahead
            angular_vel (Angular velocity): yaw velocity

        Returns:
            throttle, brake, and steering values.
        '''

        if not dbw_enabled:
            self.throttle_controller.reset()
            return 0.0,0.0
        current_vel= self.vel_lpf.filt(current_vel)

        vel_error = linear_vel-current_vel
        self.last_vel = current_vel

        current_time = rospy.get_time()
        sample_time = current_time - self.last_time
        self.last_time= current_time

        throttle = self.throttle_controller.step(vel_error, sample_time)
        brake=0

        if linear_vel ==0.0 and current_vel<0.1:
            throttle=0
            brake=700
        elif throttle<0.1 and vel_error<0:
            throttle=0
            decel=max(vel_error, self.EgoParam.decel_limit)
            brake=abs(decel)*self.EgoParam.total_vehicle_mass *self.EgoParam.wheel_radius

        return throttle, brake
