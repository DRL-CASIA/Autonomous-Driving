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
 
#include <dbc/DbcMessage.h>

#include "DbcUtilities.h"

namespace NewEagle
{
  DbcMessage::DbcMessage()
  {
  };

  DbcMessage::DbcMessage(
    uint8_t dlc,
    uint32_t id,
    IdType idType,
    std::string name,
    uint32_t rawId)
  {
    _dlc = dlc;
    _id = id;
    _idType = idType;
    _name = name;
    _rawId = rawId;
  }

  DbcMessage::~DbcMessage()
  {
  }

  uint8_t DbcMessage::GetDlc()
  {
    return _dlc;
  }

  uint32_t DbcMessage::GetId()
  {
    return _id;
  }

  uint32_t DbcMessage::GetRawId()
  {
    return _rawId;
  }

  IdType DbcMessage::GetIdType()
  {
    return _idType;
  }

  std::string DbcMessage::GetName()
  {
    return _name;
  }

  can_msgs::Frame DbcMessage::GetFrame()
  {
    can_msgs::Frame frame;

    frame.id = _id;
    frame.dlc = _dlc;
    frame.is_extended = _idType == EXT;

    uint8_t *ptr = (uint8_t*)frame.data.elems;
    memset(ptr, 0x00, 8);

    if(!AnyMultiplexedSignals()) {
      for(std::map<std::string, NewEagle::DbcSignal>::iterator it = _signals.begin(); it != _signals.end(); it++) {
        Pack(ptr, it->second);
      }    
    }
    else{
      // Start by looping through an only setting signals that are not multiplexed
      // While we're at it, we can pick out the mutliplexer switch.  
      // Perform a second loop to find the mulitplexed signal based on the multiplexer switch.
            
      NewEagle::DbcSignal* muxSwitch; // only one multiplexer switch per message is allowed

      for(std::map<std::string, NewEagle::DbcSignal>::iterator it = _signals.begin(); it != _signals.end(); it++) {
        if (NewEagle::NONE == it->second.GetMultiplexerMode()) {
          Pack(ptr, it->second);
        }                
        if (NewEagle::MUX_SWITCH == it->second.GetMultiplexerMode()) {
          muxSwitch = &it->second;
          Pack(ptr, it->second);
        }
      }  

      for(std::map<std::string, NewEagle::DbcSignal>::iterator it = _signals.begin(); it != _signals.end(); it++) {
        if (NewEagle::MUX_SIGNAL == it->second.GetMultiplexerMode()) {
          if (muxSwitch->GetResult() == it->second.GetMultiplexerSwitch()) {
            Pack(ptr, it->second);
          }

        }                  
      }
    }

    return frame;
  }

  void DbcMessage::SetFrame(const can_msgs::Frame::ConstPtr& msg)
  {
    uint8_t *ptr = (uint8_t*)msg->data.elems;

    if(!AnyMultiplexedSignals()) {
      for(std::map<std::string, NewEagle::DbcSignal>::iterator it = _signals.begin(); it != _signals.end(); it++) {
        double res = Unpack(ptr, it->second);
        it->second.SetResult(res);
      }
    }
    else{
      // Start by looping through an only setting signals that are not multiplexed
      // While we're at it, we can pick out the mutliplexer switch.  
      // Perform a second loop to find the mulitplexed signal based on the multiplexer switch.
            
      NewEagle::DbcSignal* muxSwitch; // only one multiplexer switch per message is allowed

      for(std::map<std::string, NewEagle::DbcSignal>::iterator it = _signals.begin(); it != _signals.end(); it++) {
        if (NewEagle::NONE == it->second.GetMultiplexerMode()) {
          double res = Unpack(ptr, it->second);
          it->second.SetResult(res);
        }                
        if (NewEagle::MUX_SWITCH == it->second.GetMultiplexerMode()) {
          muxSwitch = &it->second;
          double res = Unpack(ptr, it->second);
          it->second.SetResult(res);
        }
      }  

      for(std::map<std::string, NewEagle::DbcSignal>::iterator it = _signals.begin(); it != _signals.end(); it++) {
        if (NewEagle::MUX_SIGNAL == it->second.GetMultiplexerMode()) {
          if (muxSwitch->GetResult() == it->second.GetMultiplexerSwitch()) {
            double res = Unpack(ptr, it->second);
            it->second.SetResult(res);
          }

        }                  
      }
    }    

  }

  void DbcMessage::AddSignal(std::string signalName, NewEagle::DbcSignal signal)
  {
    _signals.insert(std::pair<std::string, NewEagle::DbcSignal>(signalName, signal));
  }

  NewEagle::DbcSignal* DbcMessage::GetSignal(std::string signalName)
  {
    std::map<std::string, NewEagle::DbcSignal>::iterator it;

    it = _signals.find(signalName);

    if (_signals.end() == it)
    {
      return NULL;
    }

    NewEagle::DbcSignal* signal = &it->second;

    return signal;
  }

  void DbcMessage::SetRawText(std::string rawText)
  {

  }

  uint32_t DbcMessage::GetSignalCount()
  {
    return _signals.size();
  }

  void DbcMessage::SetComment(NewEagle::DbcMessageComment comment)
  {
    _comment = comment;
  }

  std::map<std::string, NewEagle::DbcSignal>* DbcMessage::GetSignals()
  {
    return &_signals;
  }

  bool DbcMessage::AnyMultiplexedSignals()
  {
    for(std::map<std::string, NewEagle::DbcSignal>::iterator it = _signals.begin(); it != _signals.end(); it++)
    {
      if (NewEagle::MUX_SWITCH == it->second.GetMultiplexerMode())
      {
        return true;
      }
    }

    return false;
  }
}
