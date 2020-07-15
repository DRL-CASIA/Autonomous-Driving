/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2018 New Eagle
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
 *   * Neither the name of New Eagle nor the names of its
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
 
#ifndef _NEW_EAGLE_DBC_MESSAGE_H
#define _NEW_EAGLE_DBC_MESSAGE_H

#include <string>
#include <can_msgs/Frame.h>

#include <dbc/DbcSignal.h>

namespace NewEagle
{
  struct DbcMessageComment
  {
    uint32_t Id;
    std::string Comment;
  };

  enum IdType
  {
    STD = 0,
    EXT = 1
  };

  typedef struct {
    uint8_t :8;
    uint8_t :8;
    uint8_t :8;
    uint8_t :8;
    uint8_t :8;
    uint8_t :8;
    uint8_t :8;
    uint8_t :8;
  } EmptyData;

  class DbcMessage
  {
    public:
      DbcMessage();
      DbcMessage(
        uint8_t dlc,
        uint32_t id,
        IdType idType,
        std::string name,
        uint32_t rawId
     );

     ~DbcMessage();

     uint8_t GetDlc();
     uint32_t GetId();
     IdType GetIdType();
     std::string GetName();
     can_msgs::Frame GetFrame();
     uint32_t GetSignalCount();
     void SetFrame(const can_msgs::Frame::ConstPtr& msg);
     void AddSignal(std::string signalName, NewEagle::DbcSignal signal);
     NewEagle::DbcSignal* GetSignal(std::string signalName);
     void SetRawText(std::string rawText);
     uint32_t GetRawId();
     void SetComment(NewEagle::DbcMessageComment comment);
     std::map<std::string, NewEagle::DbcSignal>* GetSignals();
     bool AnyMultiplexedSignals();

   private:
     std::map<std::string, NewEagle::DbcSignal> _signals;
     uint8_t _data[8];
     uint8_t _dlc;
     uint32_t _id;
     IdType _idType;
     std::string _name;
     uint32_t _rawId;
     NewEagle::DbcMessageComment _comment;
  };
}
#endif // _NEW_EAGLE_DBC_UTILITIES_H
