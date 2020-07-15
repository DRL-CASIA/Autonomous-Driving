#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool
from coolhigh_dbw_msgs.msg import PedalCmd, SteeringCmd, BrakeCmd, SteeringReport
from geometry_msgs.msg import TwistStamped
import math

from twist_controller import Controller

'''
This node will subscribe:
'dbw_enabled' --------> Boolean value represent whether dbw is enabled.
'throttle_cmd' -------> Proposed linear and angular velocity.
'current_velocity' ---> Current vehicle velocity.

This node will publish:
'steering_cmd' -------> Steer angle.
'ThrottleCmd' --------> Accelerate value.
'BrakeCmd' -----------> Brake torque.
'''

GAS_DENSITY = 2.858
ONE_MPH = 0.44704

class EgoParams(object):


#Define a class containing all vehicle parameters.


    def __init__(self):
        self.vehicle_mass = None
        self.fuel_capacity= None
        self.brake_deadband= None
        self.decel_limit= None
        self.accel_limit= None
        self.wheel_radius = None
        self.wheel_base = None
        self.steer_ratio = None
        self.max_lat_accel = None
        self.max_steer_angle = None

class DBWNode(object):

#Define a class running dbw node.


    def __init__(self):
        rospy.init_node('adbw_node')
        EgoParam= EgoParams()

        EgoParam.vehicle_mass = rospy.get_param('~vehicle_mass', 1736.35)
        EgoParam.fuel_capacity = rospy.get_param('~fuel_capacity', 13.5)
        EgoParam.brake_deadband = rospy.get_param('~brake_deadband', .1)
        EgoParam.decel_limit = rospy.get_param('~decel_limit', -5)
        EgoParam.accel_limit = rospy.get_param('~accel_limit', 1.)
        EgoParam.wheel_radius = rospy.get_param('~wheel_radius', 0.2413)
        EgoParam.wheel_base = rospy.get_param('~wheel_base', 2.8498)
        EgoParam.steer_ratio = rospy.get_param('~steer_ratio', 1)
        EgoParam.max_lat_accel = rospy.get_param('~max_lat_accel', 3.)
        EgoParam.max_steer_angle = rospy.get_param('~max_steer_angle', 8.)
        EgoParam.total_vehicle_mass = 200

        self.steer_pub = rospy.Publisher('/vehicle/steering_cmd',
                                         SteeringCmd, queue_size=1)
        self.throttle_pub = rospy.Publisher('/vehicle/pedal_cmd',
                                            PedalCmd, queue_size=1)
        self.brake_pub = rospy.Publisher('/vehicle/brake_cmd',
                                         BrakeCmd, queue_size=1)

        # Create "Controller" object, will return throttle, brake, steering.
        self.controller= Controller(EgoParam= EgoParam)
        self.current_vel= None
        self.curr_ang_vel = None
        self.dbw_enabled = False
        self.linear_vel = None
        self.throttle=self.steering=self.brake=0

        # Subscribers
        rospy.Subscriber('/vehicle/dbw_enabled',Bool, self.dbw_enabled_cb)
        rospy.Subscriber('/twist_cmd',TwistStamped, self.twist_cb)
        rospy.Subscriber('/current_velocity', TwistStamped, self.velocity_cb)

        self.loop()

    def loop(self):

        '''
        Calculte the upcoming throttle, brake, and steering information.
        Publish the calculated information.
        Calculation and publish information all based on the set rate (50Hz).
        '''

        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            if not None in(self.current_vel, self.linear_vel):
                self.throttle, self.brake = self.controller.control(self.current_vel,
                                                                 self.dbw_enabled,
                                                                 self.linear_vel)
                print(self.brake)
            if self.dbw_enabled:
               self.publish(self.throttle, self.brake, self.steering/2)
            rate.sleep()


    def dbw_enabled_cb(self,msg):
        self.dbw_enabled =msg

    def twist_cb(self,msg):
        self.linear_vel = msg.twist.linear.x
        self.steering = msg.twist.angular.z

    def velocity_cb(self,msg):
        self.current_vel = msg.twist.linear.x
        

    def publish(self, throttle, brake, steer):
        '''
        Given throttle, brake, steer values,
        publish these values through 'throttle_cmd', 'brake_cmd' ,'steering_cmd' seperately.
        '''

        tcmd = PedalCmd()
        tcmd.enable = True
        tcmd.pedal_cmd = throttle
        self.throttle_pub.publish(tcmd)

        scmd = SteeringCmd()
        scmd.enable = True
        scmd.angle_cmd = steer
        self.steer_pub.publish(scmd)

        bcmd = BrakeCmd()
        bcmd.enable = True
        bcmd.brake_cmd = brake
        self.brake_pub.publish(bcmd)


if __name__ == '__main__':
    DBWNode()
