/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2018-2019 New Eagle 
 *  Copyright (c) 2015-2018, Dataspeed Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Dataspeed Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#include "DbwNode.h"
#include <dbw_coolhigh_can/dispatch.h>

namespace dbw_coolhigh_can
{

DbwNode::DbwNode(ros::NodeHandle &node, ros::NodeHandle &priv_nh)
{
  priv_nh.getParam("dbw_dbc_file", dbcFile_);
  

  // Initialize enable state machine
  prev_enable_ = true;
  enable_ = false;
  override_brake_ = false;
  override_accelerator_pedal_ = false;
  override_steering_ = false;
  override_gear_ = false;
  fault_brakes_ = false;
  fault_accelerator_pedal_ = false;
  fault_steering_ = false;
  fault_steering_cal_ = false;
  fault_watchdog_ = false;
  fault_watchdog_using_brakes_ = false;
  fault_watchdog_warned_ = false;
  timeout_brakes_ = false;
  timeout_accelerator_pedal_ = false;
  timeout_steering_ = false;
  enabled_brakes_ = false;
  enabled_accelerator_pedal_ = false;
  enabled_steering_ = false;
  gear_warned_ = false;

  // Frame ID
  frame_id_ = "base_footprint";
  priv_nh.getParam("frame_id", frame_id_);

  // Buttons (enable/disable)
  buttons_ = true;
  priv_nh.getParam("buttons", buttons_);


  // Ackermann steering parameters
  acker_wheelbase_ = 2.8498; // 112.2 inches
  acker_track_ = 1.5824; // 62.3 inches
  steering_ratio_ = 14.8;
  priv_nh.getParam("ackermann_wheelbase", acker_wheelbase_);
  priv_nh.getParam("ackermann_track", acker_track_);
  priv_nh.getParam("steering_ratio", steering_ratio_);

  // Initialize joint states
  joint_state_.position.resize(JOINT_COUNT);
  joint_state_.velocity.resize(JOINT_COUNT);
  joint_state_.effort.resize(JOINT_COUNT);
  joint_state_.name.resize(JOINT_COUNT);
  joint_state_.name[JOINT_FL] = "wheel_fl"; // Front Left
  joint_state_.name[JOINT_FR] = "wheel_fr"; // Front Right
  joint_state_.name[JOINT_RL] = "wheel_rl"; // Rear Left
  joint_state_.name[JOINT_RR] = "wheel_rr"; // Rear Right
  joint_state_.name[JOINT_SL] = "steer_fl";
  joint_state_.name[JOINT_SR] = "steer_fr";

  // Set up Publishers
  pub_can_ = node.advertise<can_msgs::Frame>("can_tx", 10);
  pub_brake_ = node.advertise<coolhigh_dbw_msgs::BrakeReport>("brake_report", 2);
  pub_accel_pedal_ = node.advertise<coolhigh_dbw_msgs::PedalReport>("accelerator_pedal_report", 2);
  pub_steering_ = node.advertise<coolhigh_dbw_msgs::SteeringReport>("steering_report", 2);
  pub_gear_ = node.advertise<coolhigh_dbw_msgs::GearReport>("gear_report", 2);
  pub_twist_ = node.advertise<geometry_msgs::TwistStamped>("/current_velocity", 10);
  pub_acc_ = node.advertise<std_msgs::Float64>("/current_acc", 10);
  pub_control_enable_ = node.advertise<std_msgs::Bool>("/vehicle/dbw_enabled", 10);

  publishDbwEnabled();

  // Set up Subscribers
  sub_enable_ = node.subscribe("enable", 10, &DbwNode::recvEnable, this, ros::TransportHints().tcpNoDelay(true));
  sub_disable_ = node.subscribe("disable", 10, &DbwNode::recvDisable, this, ros::TransportHints().tcpNoDelay(true));
  sub_can_ = node.subscribe("can_rx", 100, &DbwNode::recvCAN, this, ros::TransportHints().tcpNoDelay(true));

  sub_brake_ = node.subscribe("brake_cmd", 1, &DbwNode::recvBrakeCmd, this, ros::TransportHints().tcpNoDelay(true));
  sub_steering_ = node.subscribe("steering_cmd", 1, &DbwNode::recvSteeringCmd, this, ros::TransportHints().tcpNoDelay(true));
  sub_gear_ = node.subscribe("gear_cmd", 1, &DbwNode::recvGearCmd, this, ros::TransportHints().tcpNoDelay(true));
  sub_global_enable_ = node.subscribe("global_enable_cmd", 1, &DbwNode::recvGlobalEnableCmd, this, ros::TransportHints().tcpNoDelay(true));
  sub_pedal_ = node.subscribe("pedal_cmd", 1, &DbwNode::recvPedalCmd, this, ros::TransportHints().tcpNoDelay(true));

  dbwDbc_ = NewEagle::DbcBuilder().NewDbc(dbcFile_);

  // Set up Timer
  timer_ = node.createTimer(ros::Duration(1 / 20.0), &DbwNode::timerCallback, this);
}

DbwNode::~DbwNode()
{
}

void DbwNode::recvEnable(const std_msgs::Empty::ConstPtr& msg)
{
  enableSystem();
}

void DbwNode::recvDisable(const std_msgs::Empty::ConstPtr& msg)
{
  disableSystem();
}

void DbwNode::recvCAN(const can_msgs::Frame::ConstPtr& msg)
{

  if (!msg->is_rtr && !msg->is_error) {
    switch (msg->id) {
      case ID_BRAKE_REPORT:
      {
        NewEagle::DbcMessage* message = dbwDbc_.GetMessageById(ID_BRAKE_REPORT);

        if (msg->dlc >= message->GetDlc()) {
          message->SetFrame(msg);

          coolhigh_dbw_msgs::BrakeReport brakeReport;
          brakeReport.header.stamp = msg->header.stamp;
          brakeReport.pedal_sts = message->GetSignal("BRAKE_PEDAL_STS")->GetResult();
          brakeReport.enabled = message->GetSignal("BRAKE_PEDAL_EN_STS")->GetResult() ? true : false;

          pub_brake_.publish(brakeReport);
          // if (faultCh1 || faultCh2) {
          //   ROS_WARN_THROTTLE(5.0, "Brake fault.    FLT1: %s FLT2: %s",
          //       faultCh1 ? "true, " : "false,",
          //       faultCh2 ? "true, " : "false,");
          // }
        }
      }
      break;

    

      case ID_PEDAL_REPORT:
      {
        NewEagle::DbcMessage* message = dbwDbc_.GetMessageById(ID_PEDAL_REPORT);
        if (msg->dlc >= message->GetDlc()) {

          message->SetFrame(msg);

          coolhigh_dbw_msgs::PedalReport accelPedalReprt;
          accelPedalReprt.header.stamp = msg->header.stamp;
          accelPedalReprt.pedal_sts = message->GetSignal("THROTTLE_PEDAL_STS")->GetResult();
          accelPedalReprt.enabled = message->GetSignal("THROTTLE_PEDAL_EN_STS")->GetResult() ? true : false;

          pub_accel_pedal_.publish(accelPedalReprt);

          // if (faultCh1 || faultCh2) {
          //   ROS_WARN_THROTTLE(5.0, "Accelerator Pedal fault. FLT1: %s FLT2: %s",
          //       faultCh1 ? "true, " : "false,",
          //       faultCh2 ? "true, " : "false,");
          // }
        }
      }
      break;
    

      case ID_STEERING_REPORT:
      {
        NewEagle::DbcMessage* message = dbwDbc_.GetMessageById(ID_STEERING_REPORT);
        if (msg->dlc >= message->GetDlc()) {

          message->SetFrame(msg);

          coolhigh_dbw_msgs::SteeringReport steeringReport;
          steeringReport.header.stamp = msg->header.stamp;
          steeringReport.steering_wheel_angle_sts = message->GetSignal("STEER_ANGLE_STS")->GetResult();
          steeringReport.enabled = message->GetSignal("STEER_ANGLE_EN_STS")->GetResult() ? true : false;

          pub_steering_.publish(steeringReport);

          // publishJointStates(msg->header.stamp, NULL, &steeringReport);

          // if (steeringSystemFault) {
          //   ROS_WARN_THROTTLE(5.0, "Steering fault: %s",
          //       steeringSystemFault ? "true, " : "false,");
          // }
        }
      }
      break;

      case ID_GEAR_REPORT:
      {
        NewEagle::DbcMessage* message = dbwDbc_.GetMessageById(ID_GEAR_REPORT);

        if (msg->dlc >= 1) {

          message->SetFrame(msg);

          coolhigh_dbw_msgs::GearReport out;
          out.header.stamp = msg->header.stamp;
          out.state.gear = message->GetSignal("GEAR_STS")->GetResult();     
          pub_gear_.publish(out);
        }
      }
      break;

      case ID_ECU_REPORT:
      {
        NewEagle::DbcMessage* message = dbwDbc_.GetMessageById(ID_ECU_REPORT);

        if (msg->dlc >= 1) {

          message->SetFrame(msg);
          // TODO
          geometry_msgs::TwistStamped vel_out;
          vel_out.header.stamp = msg->header.stamp;
          vel_out.twist.linear.x = message->GetSignal("SPEED")->GetResult();
          pub_twist_.publish(vel_out);

          std_msgs::Float64 acc_out;
          acc_out.data = message->GetSignal("ACC_SPEED")->GetResult()*0.001;
          pub_acc_.publish(acc_out);

          // coolhigh_dbw_msgs::EcuStatusReport out;
          // out.header.stamp = msg->header.stamp;
          // out.state.gear = message->GetSignal("GEAR_STS")->GetResult();     
          // pub_gear_.publish(out);
        }
      }
      break;



      case ID_BRAKE_CMD:
        //ROS_WARN("DBW system: Another node on the CAN bus is commanding the vehicle!!! Subsystem: Brake. Id: 0x%03X", ID_BRAKE_CMD);
        break;
      case ID_PEDAL_CMD:
        //ROS_WARN("DBW system: Another node on the CAN bus is commanding the vehicle!!! Subsystem: Accelerator Pedal. Id: 0x%03X", ID_ACCELERATOR_PEDAL_CMD);
        break;
      case ID_STEERING_CMD:
        //ROS_WARN("DBW system: Another node on the CAN bus is commanding the vehicle!!! Subsystem: Steering. Id: 0x%03X", ID_STEERING_CMD);
        break;
      case ID_GEAR_CMD:
        //ROS_WARN("DBW system: Another node on the CAN bus is commanding the vehicle!!! Subsystem: Shifting. Id: 0x%03X", ID_GEAR_CMD);
        break;
    }
  }
#if 0
  ROS_INFO("ena: %s, clr: %s, brake: %s, Accelerator Pedal: %s, steering: %s, gear: %s",
           enabled() ? "true " : "false",
           clear() ? "true " : "false",
           override_brake_ ? "true " : "false",
           override_accelerator_pedal_ ? "true " : "false",
           override_steering_ ? "true " : "false",
           override_gear_ ? "true " : "false"
       );
#endif


}

void DbwNode::recvBrakeCmd(const coolhigh_dbw_msgs::BrakeCmd::ConstPtr& msg)
{
  NewEagle::DbcMessage* message = dbwDbc_.GetMessage("Brake_Command");
  
  message->GetSignal("BRAKE_PEDAL_EN_CTRL")->SetResult(0);
  message->GetSignal("BRAKE_PEDAL_CMD")->SetResult(0);

  if (enabled()) {
    if(msg->enable) {
      message->GetSignal("BRAKE_PEDAL_CMD")->SetResult(msg->brake_cmd);  
      message->GetSignal("BRAKE_PEDAL_EN_CTRL")->SetResult(1);
      can_msgs::Frame frame = message->GetFrame();
      pub_can_.publish(frame);
    }
  }


}

void DbwNode::recvPedalCmd(const coolhigh_dbw_msgs::PedalCmd::ConstPtr& msg)
{
  NewEagle::DbcMessage* message = dbwDbc_.GetMessage("Throttle_Command");

  message->GetSignal("THROTTLE_PEDAL_EN_CTRL")->SetResult(0);
  message->GetSignal("THROTTLE_PEDAL_CMD")->SetResult(0);

  if (enabled()) {
    if(msg->enable) {
      message->GetSignal("THROTTLE_PEDAL_EN_CTRL")->SetResult(1);
      message->GetSignal("THROTTLE_PEDAL_CMD")->SetResult(msg->pedal_cmd);
    }
  }

  can_msgs::Frame frame = message->GetFrame();
  pub_can_.publish(frame);                                                                                                                                                                                                                                      
}

void DbwNode::recvSteeringCmd(const coolhigh_dbw_msgs::SteeringCmd::ConstPtr& msg)
{
  NewEagle::DbcMessage* message = dbwDbc_.GetMessage("Steer_Command");

  message->GetSignal("STEER_ANGLE_EN_CTRL")->SetResult(0);
  message->GetSignal("STEER_ANGLE_CMD")->SetResult(0);

  if (enabled()) {
      message->GetSignal("STEER_ANGLE_EN_CTRL")->SetResult(1);      
      // double scmd = std::max((float)-470.0, std::min((float)470.0, (float)(msg->angle_cmd)));
      message->GetSignal("STEER_ANGLE_CMD")->SetResult(msg->angle_cmd);

  }

  can_msgs::Frame frame = message->GetFrame();
  pub_can_.publish(frame);
}

void DbwNode::recvGearCmd(const coolhigh_dbw_msgs::GearCmd::ConstPtr& msg)
{
  NewEagle::DbcMessage* message = dbwDbc_.GetMessage("Gear_Command");

  message->GetSignal("GEAR_CMD")->SetResult(0);

  if (enabled()) {
    message->GetSignal("GEAR_CMD")->SetResult(msg->cmd.gear);
  }  

  can_msgs::Frame frame = message->GetFrame();

  pub_can_.publish(frame);
}

void DbwNode::recvGlobalEnableCmd(const coolhigh_dbw_msgs::GlobalEnableCmd::ConstPtr& msg)
{
  NewEagle::DbcMessage* message = dbwDbc_.GetMessage("Control_Command");

  message->GetSignal("CTRL_CMD")->SetResult(0);

  if (enabled()) {
    if(msg->global_enable) {
      message->GetSignal("CTRL_CMD")->SetResult(1);
    }
  }  
   
  can_msgs::Frame frame = message->GetFrame();
  pub_can_.publish(frame);      
}



bool DbwNode::publishDbwEnabled()
{
  bool change = false;
  bool en = enabled();
  if (prev_enable_ != en) {
    std_msgs::Bool msg;
    msg.data = en;
    // pub_sys_enable_.publish(msg);
    change = true;
  }
  prev_enable_ = en;
  return change;
}

void DbwNode::timerCallback(const ros::TimerEvent& event)
{/*
  if (clear()) {
    can_msgs::Frame out;
    out.is_extended = false;

    if (override_brake_) {
      // Might have an issue with WatchdogCntr when these are set.
      NewEagle::DbcMessage* message = dbwDbc_.GetMessage("Brake_Command");
      message->GetSignal("BRAKE_PEDAL_EN_CTRL")->SetResult(0);
      message->GetSignal("BRAKE_PEDAL_CMD")->SetResult(0);
      //message->GetSignal("AKit_BrakePedalCtrlMode")->SetResult(0);
      pub_can_.publish(message->GetFrame());
    }

    if (override_accelerator_pedal_)
    {
      // Might have an issue with WatchdogCntr when these are set.
      NewEagle::DbcMessage* message = dbwDbc_.GetMessage("Throttle_Command");
      message->GetSignal("THROTTLE_PEDAL_EN_CTRL")->SetResult(0);
      message->GetSignal("THROTTLE_PEDAL_CMD")->SetResult(0);
      pub_can_.publish(message->GetFrame());
    }

    if (override_steering_) {
      // Might have an issue with WatchdogCntr when these are set.
      NewEagle::DbcMessage* message = dbwDbc_.GetMessage("Steer_Command");
      message->GetSignal("STEER_ANGLE_EN_CTRL")->SetResult(0);
      message->GetSignal("STEER_ANGLE_CMD")->SetResult(0);
      pub_can_.publish(message->GetFrame());
    }

    if (override_gear_) {
      NewEagle::DbcMessage* message = dbwDbc_.GetMessage("Gear_Command");
      message->GetSignal("GEAR_CMD")->SetResult(0);
      pub_can_.publish(message->GetFrame());
    }
  }*/
}

void DbwNode::enableSystem()
{
  if (!enable_) {
    if (fault()) {
      if (fault_steering_cal_) {
        ROS_WARN("DBW system not enabled. Steering calibration fault.");
      }
      if (fault_brakes_) {
        ROS_WARN("DBW system not enabled. Braking fault.");
      }
      if (fault_accelerator_pedal_) {
        ROS_WARN("DBW system not enabled. Accelerator Pedal fault.");
      }
      if (fault_steering_) {
        ROS_WARN("DBW system not enabled. Steering fault.");
      }
      if (fault_watchdog_) {
        ROS_WARN("DBW system not enabled. Watchdog fault.");
      }
    } else {
      enableVehicle();
      enable_ = true;
      if (publishDbwEnabled()) {
        // enable the vehicle
        ROS_INFO("DBW system enabled.");
        ROS_WARN("Control module enabled.");
      } else {
        ROS_INFO("DBW system enable requested. Waiting for ready.");
      }



    }
  }
}

void DbwNode::disableSystem()
{
  if (enable_) {
    enable_ = false;
    publishDbwEnabled();
    ROS_WARN("DBW system disabled.");
    std_msgs::Bool enable_control;
    enable_control.data = false;
    pub_control_enable_.publish(enable_control);
    ROS_WARN("Control module disabled.");
  }
}

void DbwNode::buttonCancel()
{
  if (enable_) {
    enable_ = false;
    publishDbwEnabled();
    ROS_WARN("DBW system disabled. Cancel button pressed.");
  }
}

void DbwNode::overrideBrake(bool override)
{
  bool en = enabled();
  if (override && en) {
    enable_ = false;
  }
  override_brake_ = override;
  if (publishDbwEnabled()) {
    if (en) {
      ROS_WARN("DBW system disabled. Driver override on brake/Accelerator Pedal pedal.");
    } else {
      ROS_INFO("DBW system enabled.");
    }
  }
}

void DbwNode::overrideAcceleratorPedal(bool override)
{
  bool en = enabled();
  if (override && en) {
    enable_ = false;
  }
  override_accelerator_pedal_ = override;
  if (publishDbwEnabled()) {
    if (en) {
      ROS_WARN("DBW system disabled. Driver override on brake/Accelerator Pedal pedal.");
    } else {
      ROS_INFO("DBW system enabled.");
    }
  }
}

void DbwNode::overrideSteering(bool override)
{
  bool en = enabled();
  if (override && en) {
    enable_ = false;
  }
  override_steering_ = override;
  if (publishDbwEnabled()) {
    if (en) {
      ROS_WARN("DBW system disabled. Driver override on steering wheel.");
    } else {
      ROS_INFO("DBW system enabled.");
    }
  }
}

void DbwNode::overrideGear(bool override)
{
  bool en = enabled();
  if (override && en) {
    enable_ = false;
  }
  override_gear_ = override;
  if (publishDbwEnabled()) {
    if (en) {
      ROS_WARN("DBW system disabled. Driver override on shifter.");
    } else {
      ROS_INFO("DBW system enabled.");
    }
  }
}

void DbwNode::timeoutBrake(bool timeout, bool enabled)
{
  if (!timeout_brakes_ && enabled_brakes_ && timeout && !enabled) {
    ROS_WARN("Brake subsystem disabled after 100ms command timeout");
  }
  timeout_brakes_ = timeout;
  enabled_brakes_ = enabled;
}

void DbwNode::timeoutAcceleratorPedal(bool timeout, bool enabled)
{
  if (!timeout_accelerator_pedal_ && enabled_accelerator_pedal_ && timeout && !enabled) {
    ROS_WARN("Accelerator Pedal subsystem disabled after 100ms command timeout");
  }
  timeout_accelerator_pedal_ = timeout;
  enabled_accelerator_pedal_ = enabled;
}

void DbwNode::timeoutSteering(bool timeout, bool enabled)
{
  if (!timeout_steering_ && enabled_steering_ && timeout && !enabled) {
    ROS_WARN("Steering subsystem disabled after 100ms command timeout");
  }
  timeout_steering_ = timeout;
  enabled_steering_ = enabled;
}

void DbwNode::faultBrakes(bool fault)
{
  bool en = enabled();
  if (fault && en) {
    enable_ = false;
  }
  fault_brakes_ = fault;
  if (publishDbwEnabled()) {
    if (en) {
      ROS_ERROR("DBW system disabled. Braking fault.");
    } else {
      ROS_INFO("DBW system enabled.");
    }
  }
}

void DbwNode::faultAcceleratorPedal(bool fault)
{
  bool en = enabled();
  if (fault && en) {
    enable_ = false;
  }
  fault_accelerator_pedal_ = fault;
  if (publishDbwEnabled()) {
    if (en) {
      ROS_ERROR("DBW system disabled. Accelerator Pedal fault.");
    } else {
      ROS_INFO("DBW system enabled.");
    }
  }
}

void DbwNode::faultSteering(bool fault)
{
  bool en = enabled();
  if (fault && en) {
    enable_ = false;
  }
  fault_steering_ = fault;
  if (publishDbwEnabled()) {
    if (en) {
      ROS_ERROR("DBW system disabled. Steering fault.");
    } else {
      ROS_INFO("DBW system enabled.");
    }
  }
}

void DbwNode::faultSteeringCal(bool fault)
{
  bool en = enabled();
  if (fault && en) {
    enable_ = false;
  }
  fault_steering_cal_ = fault;
  if (publishDbwEnabled()) {
    if (en) {
      ROS_ERROR("DBW system disabled. Steering calibration fault.");
    } else {
      ROS_INFO("DBW system enabled.");
    }
  }
}

void DbwNode::faultWatchdog(bool fault, uint8_t src, bool braking)
{
  bool en = enabled();
  if (fault && en) {
    enable_ = false;
  }
  fault_watchdog_ = fault;
  if (publishDbwEnabled()) {
    if (en) {
      ROS_ERROR("DBW system disabled. Watchdog fault.");
    } else {
      ROS_INFO("DBW system enabled.");
    }
  }
  if (braking && !fault_watchdog_using_brakes_) {
    ROS_WARN("Watchdog event: Alerting driver and applying brakes.");
  } else if (!braking && fault_watchdog_using_brakes_) {
    ROS_INFO("Watchdog event: Driver has successfully taken control.");
  }
  if (fault && src && !fault_watchdog_warned_) {
    ROS_WARN("Watchdog event: Unknown Fault!");
      // switch (src) {
      //   case dbw_pacifica_msgs::WatchdogStatus::OTHER_BRAKE:
      //     ROS_WARN("Watchdog event: Fault determined by brake controller");
      //     break;
      //   case dbw_pacifica_msgs::WatchdogStatus::OTHER_ACCELERATOR_PEDAL:
      //     ROS_WARN("Watchdog event: Fault determined by Accelerator Pedal controller");
      //     break;
      //   case dbw_pacifica_msgs::WatchdogStatus::OTHER_STEERING:
      //     ROS_WARN("Watchdog event: Fault determined by steering controller");
      //     break;
      //   case dbw_pacifica_msgs::WatchdogStatus::BRAKE_COUNTER:
      //     ROS_WARN("Watchdog event: Brake command counter failed to increment");
      //     break;
      //   case dbw_pacifica_msgs::WatchdogStatus::BRAKE_DISABLED:
      //     ROS_WARN("Watchdog event: Brake transition to disabled while in gear or moving");
      //     break;
      //   case dbw_pacifica_msgs::WatchdogStatus::BRAKE_COMMAND:
      //     ROS_WARN("Watchdog event: Brake command timeout after 100ms");
      //     break;
      //   case dbw_pacifica_msgs::WatchdogStatus::BRAKE_REPORT:
      //     ROS_WARN("Watchdog event: Brake report timeout after 100ms");
      //     break;
      //   case dbw_pacifica_msgs::WatchdogStatus::ACCELERATOR_PEDAL_COUNTER:
      //     ROS_WARN("Watchdog event: Accelerator Pedal command counter failed to increment");
      //     break;
      //   case dbw_pacifica_msgs::WatchdogStatus::ACCELERATOR_PEDAL_DISABLED:
      //     ROS_WARN("Watchdog event: Accelerator Pedal transition to disabled while in gear or moving");
      //     break;
      //   case dbw_pacifica_msgs::WatchdogStatus::ACCELERATOR_PEDAL_COMMAND:
      //     ROS_WARN("Watchdog event: Accelerator Pedal command timeout after 100ms");
      //     break;
      //   case dbw_pacifica_msgs::WatchdogStatus::ACCELERATOR_PEDAL_REPORT:
      //     ROS_WARN("Watchdog event: Accelerator Pedal report timeout after 100ms");
      //     break;
      //   case dbw_pacifica_msgs::WatchdogStatus::STEERING_COUNTER:
      //     ROS_WARN("Watchdog event: Steering command counter failed to increment");
      //     break;
      //   case dbw_pacifica_msgs::WatchdogStatus::STEERING_DISABLED:
      //     ROS_WARN("Watchdog event: Steering transition to disabled while in gear or moving");
      //     break;
      //   case dbw_pacifica_msgs::WatchdogStatus::STEERING_COMMAND:
      //     ROS_WARN("Watchdog event: Steering command timeout after 100ms");
      //     break;
      //   case dbw_pacifica_msgs::WatchdogStatus::STEERING_REPORT:
      //     ROS_WARN("Watchdog event: Steering report timeout after 100ms");
      //     break;
      // }
      fault_watchdog_warned_ = true;
  } else if (!fault) {
    fault_watchdog_warned_ = false;
  }
  fault_watchdog_using_brakes_ = braking;
  if (fault && !fault_watchdog_using_brakes_ && fault_watchdog_warned_) {
    ROS_WARN_THROTTLE(2.0, "Watchdog event: Press left OK button on the steering wheel or cycle power to clear event.");
  }
}

void DbwNode::faultWatchdog(bool fault, uint8_t src) {
  faultWatchdog(fault, src, fault_watchdog_using_brakes_); // No change to 'using brakes' status
}


void DbwNode::enableVehicle()
{
  NewEagle::DbcMessage* message_1 = dbwDbc_.GetMessage("Throttle_Command");
  message_1->GetSignal("THROTTLE_PEDAL_EN_CTRL")->SetResult(0);
  message_1->GetSignal("THROTTLE_PEDAL_CMD")->SetResult(0);
  pub_can_.publish(message_1->GetFrame());

  NewEagle::DbcMessage* message_2 = dbwDbc_.GetMessage("Brake_Command"); 
  message_2->GetSignal("BRAKE_PEDAL_EN_CTRL")->SetResult(0);
  message_2->GetSignal("BRAKE_PEDAL_CMD")->SetResult(70);
  pub_can_.publish(message_2->GetFrame());

  NewEagle::DbcMessage* message_3 = dbwDbc_.GetMessage("Steer_Command");
  message_3->GetSignal("STEER_ANGLE_EN_CTRL")->SetResult(0);
  message_3->GetSignal("STEER_ANGLE_CMD")->SetResult(0);
  pub_can_.publish(message_3->GetFrame());

  NewEagle::DbcMessage* message_4 = dbwDbc_.GetMessage("Throttle_Command");
  message_4->GetSignal("THROTTLE_PEDAL_EN_CTRL")->SetResult(1);
  message_4->GetSignal("THROTTLE_PEDAL_CMD")->SetResult(0);
  pub_can_.publish(message_4->GetFrame());

  NewEagle::DbcMessage* message_5 = dbwDbc_.GetMessage("Brake_Command"); 
  message_5->GetSignal("BRAKE_PEDAL_EN_CTRL")->SetResult(1);
  message_5->GetSignal("BRAKE_PEDAL_CMD")->SetResult(70);
  pub_can_.publish(message_5->GetFrame());

  NewEagle::DbcMessage* message_6 = dbwDbc_.GetMessage("Steer_Command");
  message_6->GetSignal("STEER_ANGLE_EN_CTRL")->SetResult(1);
  message_6->GetSignal("STEER_ANGLE_CMD")->SetResult(0);
  pub_can_.publish(message_6->GetFrame());

  NewEagle::DbcMessage* message_7 = dbwDbc_.GetMessage("Steer_Command");
  message_7->GetSignal("STEER_ANGLE_EN_CTRL")->SetResult(1);
  message_7->GetSignal("STEER_ANGLE_CMD")->SetResult(0);
  pub_can_.publish(message_7->GetFrame());

  NewEagle::DbcMessage* message_8 = dbwDbc_.GetMessage("Gear_Command");
  message_8->GetSignal("GEAR_CMD")->SetResult(4);
  pub_can_.publish(message_8->GetFrame());

  NewEagle::DbcMessage* message_9 = dbwDbc_.GetMessage("Brake_Command"); 
  message_9->GetSignal("BRAKE_PEDAL_EN_CTRL")->SetResult(1);
  message_9->GetSignal("BRAKE_PEDAL_CMD")->SetResult(0);
  pub_can_.publish(message_9->GetFrame());

  std_msgs::Bool enable_control;
  enable_control.data = true;
  pub_control_enable_.publish(enable_control);

}


/*
void DbwNode::publishJointStates(const ros::Time &stamp, const dbw_pacifica_msgs::WheelSpeedReport *wheels, const dbw_pacifica_msgs::SteeringReport *steering)
{
  double dt = (stamp - joint_state_.header.stamp).toSec();
  if (wheels) {
    joint_state_.velocity[JOINT_FL] = wheels->front_left;
    joint_state_.velocity[JOINT_FR] = wheels->front_right;
    joint_state_.velocity[JOINT_RL] = wheels->rear_left;
    joint_state_.velocity[JOINT_RR] = wheels->rear_right;
  }
  if (steering) {
    const double L = acker_wheelbase_;
    const double W = acker_track_;
    const double r = L / tan(steering->steering_wheel_angle / steering_ratio_);
    joint_state_.position[JOINT_SL] = atan(L / (r - W/2));
    joint_state_.position[JOINT_SR] = atan(L / (r + W/2));
  }
  if (dt < 0.5) {
    for (unsigned int i = JOINT_FL; i <= JOINT_RR; i++) {
      joint_state_.position[i] = fmod(joint_state_.position[i] + dt * joint_state_.velocity[i], 2*M_PI);
    }
  }
  joint_state_.header.stamp = stamp;
  pub_joint_states_.publish(joint_state_);
}
*/

} // dbw_pacifica_can
