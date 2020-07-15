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

#include <dbc/Dbc.h>

namespace NewEagle
{
  ////
  Dbc::Dbc()
  {
  }

  Dbc::~Dbc()
  {
  }

  std::map<std::string, NewEagle::DbcMessage>* Dbc::GetMessages()
  {
    return &_messages;

  }

  void Dbc::AddMessage(std::string messageName, NewEagle::DbcMessage message)
  {
    _messages.insert(std::pair<std::string, NewEagle::DbcMessage>(message.GetName(), message));
  }

  NewEagle::DbcMessage* Dbc::GetMessage(std::string messageName)
  {
    std::map<std::string, NewEagle::DbcMessage>::iterator it;

    it = _messages.find(messageName);

    if (_messages.end() == it)
    {
      return NULL;
    }

    NewEagle::DbcMessage* message = &it->second;

    return message;
  }

  NewEagle::DbcMessage* Dbc::GetMessageById(uint32_t id)
  {
    for(std::map<std::string, NewEagle::DbcMessage>::iterator it = _messages.begin(); it != _messages.end(); it++)
    {
      if (it->second.GetId() == id)
      {
        NewEagle::DbcMessage* message = &it->second;

        return message;
      }

    }

    return NULL;
  }

  uint16_t Dbc::GetMessageCount()
  {
    return _messages.size();
  }
}
