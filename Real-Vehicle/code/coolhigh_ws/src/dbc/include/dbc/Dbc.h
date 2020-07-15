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

#ifndef _NEW_EAGLE_DBC_H
#define _NEW_EAGLE_DBC_H

#include <ros/ros.h>

#include <string>
#include <ctype.h>

#include <dbc/DbcMessage.h>

namespace NewEagle
{
  class Dbc
  {
    public:
      Dbc();
      ~Dbc();

      void AddMessage(std::string messageName, NewEagle::DbcMessage message);
      NewEagle::DbcMessage* GetMessage(std::string messageName);
      NewEagle::DbcMessage* GetMessageById(uint32_t id);
      uint16_t GetMessageCount();
      std::map<std::string, NewEagle::DbcMessage>* GetMessages();

    private:
      std::map<std::string, NewEagle::DbcMessage> _messages;

  };
}

#endif // _NEW_EAGLE_DBC_H
