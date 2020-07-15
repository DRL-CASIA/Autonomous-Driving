/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2018-2019 New Eagle 
 *  Copyright (c) 2015-2018, Dataspeed Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
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

#ifndef _DBW_PACIFICA_CAN_DISPATCH_H
#define _DBW_PACIFICA_CAN_DISPATCH_H
#include <stdint.h>

namespace dbw_pacifica_can
{
typedef enum {
  VIN_MUX_VIN0  = 0x00,
  VIN_MUX_VIN1  = 0x01,
  VIN_MUX_VIN2  = 0x02,
} VinMux;

typedef enum {
  WHEEL_SPEED_MUX0  = 0x00,
  WHEEL_SPEED_MUX1  = 0x01,
  WHEEL_SPEED_MUX2  = 0x02,
} WheelSpeedMux;



enum {
  ID_BRAKE_CMD                  = 0x2F04,
  ID_BRAKE_REPORT               = 0x1F04,
  ID_PEDAL_CMD                  = 0x2F01,
  ID_PEDAL_REPORT               = 0x1F02,
  ID_STEERING_CMD               = 0x2F03,
  ID_STEERING_REPORT            = 0x1F03,
  ID_GEAR_CMD                   = 0x2F05,
  ID_GEAR_REPORT                = 0x1F05,
  ID_REPORT_WHEEL_SPEED         = 0x1F0B,
  ID_REPORT_IMU                 = 0x1F0A,
  ID_REPORT_TIRE_PRESSURE       = 0x1f07,
  ID_REPORT_SURROUND            = 0x1f10,
  ID_VIN                        = 0x1F08,
  ID_REPORT_DRIVER_INPUT        = 0x1F0F,
  ID_REPORT_WHEEL_POSITION      = 0x1F06,
  ID_MISC_REPORT                = 0x1F01,
  ID_LOW_VOLTAGE_SYSTEM_REPORT  = 0x1F11,
  ID_BRAKE_2_REPORT             = 0x1F12,
  ID_STEERING_2_REPORT          = 0x1F13,
  ID_OTHER_ACTUATORS_REPORT     = 0x1F14,
  ID_FAULT_ACTION_REPORT        = 0x1F15,
  ID_HMI_GLOBAL_ENABLE_REPORT   = 0x3f01,
};

} //dbw_pacifica_can

#endif // _DBW_PACIFICA_CAN_DISPATCH_H
