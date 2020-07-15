#ifndef _REMOTE_CONTROL_H_
#define _REMOTE_CONTROL_H_

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>

#include <coolhigh_dbw_msgs/BrakeCmd.h>
#include <coolhigh_dbw_msgs/GearCmd.h>
#include <coolhigh_dbw_msgs/PedalCmd.h>
#include <coolhigh_dbw_msgs/SteeringCmd.h>
#include <std_msgs/Empty.h>


namespace remote_control
{
    class RemoteControl
    {
        public:
            RemoteControl(ros::NodeHandle &node, ros::NodeHandle &priv_nh);
            ~RemoteControl();
            void run();
        private:

            void recvJoy(const sensor_msgs::Joy::ConstPtr& msg);


            bool button_event_;
            bool enabled_;
            float throttle_;
            float brake_;
            float steer_;

        private:
            ros::Subscriber sub_joy_;
            ros::Publisher pub_steer_cmd_;
            ros::Publisher pub_brake_cmd_;
            ros::Publisher pub_throttle_cmd_;
            ros::Publisher pub_enable_cmd_;
            ros::Publisher pub_disable_cmd_;


    };

}



#endif //_REMOTE_CONTROL_H_