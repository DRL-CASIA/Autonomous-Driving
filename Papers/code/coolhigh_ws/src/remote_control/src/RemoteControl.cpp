#include "RemoteControl.h"

namespace remote_control
{
    RemoteControl::RemoteControl(ros::NodeHandle &node, ros::NodeHandle &priv_nh)
    {
        ROS_INFO("Axis  is now at position ");
        enabled_ = false;
        brake_ = 0.0;
        throttle_ = 0.0;
        steer_ = 0.0;
        button_event_ = false;

        pub_steer_cmd_ = node.advertise<coolhigh_dbw_msgs::SteeringCmd>("/vehicle/steering_cmd", 10);
        pub_brake_cmd_ = node.advertise<coolhigh_dbw_msgs::BrakeCmd>("/vehicle/brake_cmd", 10);
        pub_throttle_cmd_ = node.advertise<coolhigh_dbw_msgs::PedalCmd>("/vehicle/pedal_cmd", 10);
        pub_enable_cmd_ = node.advertise<std_msgs::Empty>("/vehicle/enable", 10);
        pub_disable_cmd_ = node.advertise<std_msgs::Empty>("/vehicle/disable", 10);
        sub_joy_ = node.subscribe("/game_control/joy", 10, &RemoteControl::recvJoy, this, ros::TransportHints().tcpNoDelay(true));
        run();
    }
    

    void RemoteControl::recvJoy(const sensor_msgs::Joy::ConstPtr& msg)
    {
        brake_ = msg->axes[3];
        throttle_ = msg->axes[2];
        steer_ = msg->axes[0];
        button_event_ = msg->buttons[23];

        for (unsigned i = 0; i < msg->axes.size(); ++i) {
            ROS_INFO("Axis %d is now at position %f", i, msg->axes[i]);
        }

        ROS_INFO("Button is now at position %d", button_event_);

    }

    RemoteControl::~RemoteControl()
    {
        
    }

    void RemoteControl::run()
    {
        ros::Rate loop_rate(10);
        while (ros::ok())
        {
            coolhigh_dbw_msgs::BrakeCmd brake;
            brake.brake_cmd = (brake_+1) * 30;
            brake.enable = 1;

            coolhigh_dbw_msgs::PedalCmd pedal;
            pedal.pedal_cmd = (throttle_+1) * 20;
            pedal.enable = 1;

            coolhigh_dbw_msgs::SteeringCmd steer;
            steer.angle_cmd = steer_;
            steer.enable = 1;
            
            if(button_event_)
            {
                if(enabled_)
                {
                    enabled_ = false;
                    std_msgs::Empty empty_msg;
                    pub_disable_cmd_.publish(empty_msg);                 
                }
                else
                {
                    enabled_ = true;
                    std_msgs::Empty empty_msg;
                    pub_enable_cmd_.publish(empty_msg);
                }
            }
            if(enabled_)
            {
                pub_steer_cmd_.publish(steer);
                pub_brake_cmd_.publish(brake);
                pub_throttle_cmd_.publish(pedal);
            }
           ros::spinOnce();
           loop_rate.sleep();
        }

    }





}