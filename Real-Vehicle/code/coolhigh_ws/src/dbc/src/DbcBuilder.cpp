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

#include <dbc/DbcBuilder.h>
#include <dbc/DbcMessage.h>
#include <dbc/DbcSignal.h>

namespace NewEagle
{
  DbcBuilder::DbcBuilder()
  {
    MessageToken = std::string("BO_");
    SignalToken = std::string("SG_");
    CommentToken = std::string("CM_");
    EnumValueToken = std::string("VAL_");
    AttributeToken = std::string("BA_");
    SignalValueTypeToken = std::string("SIG_VALTYPE_");
  }

  DbcBuilder::~DbcBuilder()
  {
  }

  NewEagle::Dbc DbcBuilder::NewDbc(const std::string &dbcFile)
  {

    NewEagle::Dbc dbc;

    std::istringstream f(dbcFile);
    std::string line;
    uint32_t lineNumber = 0;

    NewEagle::DbcMessage currentMessage;

    int32_t cnt = 0;

    while (std::getline(f, line, '\n'))
    {
      lineNumber++;

      NewEagle::LineParser parser(line);


      std::string identifier;
      try
      {
        identifier = parser.ReadCIdentifier();
      }
      catch(std::exception& ex)
      {
        identifier = std::string();
      }

      if (!MessageToken.compare(identifier))
      {
        try
        {
          currentMessage =  ReadMessage(parser);
          currentMessage.SetRawText(line);
          dbc.AddMessage(currentMessage.GetName(), currentMessage);
        }
        catch(LineParserExceptionBase& exlp)
        {
          ROS_WARN("LineParser Exception: [%s]", exlp.what());
        }
        catch(std::exception& ex)
        {
          ROS_ERROR("DBC Message Parser Exception: [%s]", ex.what());
        }
      }
      else if (!SignalToken.compare(identifier))
      {
        try
        {
          NewEagle::DbcSignal signal = ReadSignal(parser);

          NewEagle::DbcMessage* msg = dbc.GetMessage(currentMessage.GetName());
          msg->AddSignal(signal.GetName(), signal);
        }
        catch(LineParserExceptionBase& exlp)
        {
          ROS_WARN("LineParser Exception: [%s]", exlp.what());
        }
        catch(std::exception& ex)
        {
          ROS_ERROR("DBC Signal Parser Exception: [%s]", ex.what());
        }
      }
      else if (!CommentToken.compare(identifier))
      {
        try
        {
          std::string token = parser.ReadCIdentifier();

          if (!MessageToken.compare(token))
          {
            NewEagle::DbcMessageComment dbcMessageComment = ReadMessageComment(parser);

            std::map<std::string, NewEagle::DbcMessage>::iterator it;
            int32_t ttt = 0;

            for (it = dbc.GetMessages()->begin(); it != dbc.GetMessages()->end(); ++it)
            {
              ttt++;
              if (it->second.GetRawId() == dbcMessageComment.Id)
              {
                it->second.SetComment(dbcMessageComment);
                break;
              }
            }
          }
          else if (!SignalToken.compare(token))
          {
            NewEagle::DbcSignalComment dbcSignalComment = ReadSignalComment(parser);

            std::map<std::string, NewEagle::DbcMessage>::iterator it;
            for (it = dbc.GetMessages()->begin(); it != dbc.GetMessages()->end(); ++it)
            {
              if (it->second.GetRawId() == dbcSignalComment.Id)
              {
                DbcMessage msg = it->second;
                std::map<std::string, NewEagle::DbcSignal>::iterator its;

                for (its = msg.GetSignals()->begin(); its != msg.GetSignals()->end(); ++its)
                {
                  if (its->second.GetName() == dbcSignalComment.SignalName)
                  {
                    its->second.SetComment(dbcSignalComment);
                    break;
                  }
                }
              }
            }
          }
        }
        catch(LineParserExceptionBase& exlp)
        {
          ROS_WARN("LineParser Exception: [%s]", exlp.what());
        }
        catch(std::exception& ex)
        {
          ROS_ERROR("DBC Comment Parser Exception: [%s]", ex.what());
        }
      }
      else if (!AttributeToken.compare(identifier))
      {
        try
        {
          NewEagle::DbcAttribute dbcAttribute = ReadAttribute(parser);

          if (dbc.GetMessageCount() > 0)
          {
            std::map<std::string, NewEagle::DbcMessage>::iterator it;
            for (it = dbc.GetMessages()->begin(); it != dbc.GetMessages()->end(); ++it)
            {
              if (it->second.GetRawId() == dbcAttribute.Id)
              {
                std::map<std::string, NewEagle::DbcSignal>::iterator its;
                DbcMessage msg = it->second;
                for (its = msg.GetSignals()->begin(); its != msg.GetSignals()->end(); ++its)
                {
                  if (its->second.GetName() == dbcAttribute.SignalName)
                  {
                    NewEagle::DbcSignal sig = its->second;

                    double gain = sig.GetGain();
                    double offset = sig.GetOffset();

                    double f = 0.0;

                    std::stringstream ss;
                    ss << dbcAttribute.Value;
                    ss >> f;

                    double val = gain * f + offset;
                    sig.SetInitialValue(val);
                    break;
                  }
                }
              }
            }
          }
        }
        catch(LineParserExceptionBase& exlp)
        {
          ROS_WARN("LineParser Exception: [%s]", exlp.what());
        }
        catch(std::exception& ex)
        {
          ROS_ERROR("DBC Signal Value Type Parser Exception: [%s]", ex.what());
        }
      }
      else if (!EnumValueToken.compare(identifier))
      {
        // Empty for now.
      }
      else if (!SignalValueTypeToken.compare(identifier))
      {
        try
        {
          NewEagle::DbcSignalValueType dbcSignalValueType = ReadSignalValueType(parser);

          if (dbc.GetMessageCount() > 0)
          {
            std::map<std::string, NewEagle::DbcMessage>::iterator it;
            for (it = dbc.GetMessages()->begin(); it != dbc.GetMessages()->end(); ++it)
            {
              if (it->second.GetRawId() == dbcSignalValueType.Id)
              {
                std::map<std::string, NewEagle::DbcSignal>::iterator its;
                DbcMessage msg = it->second;
                for (its = msg.GetSignals()->begin(); its != msg.GetSignals()->end(); ++its)
                {
                  if (its->second.GetName() == dbcSignalValueType.SignalName)
                  {
                    NewEagle::DbcSignal sig = its->second;
                    sig.SetDataType(dbcSignalValueType.Type);
                    break;
                  }
                }
              }
            }
          }
        }
        catch(LineParserExceptionBase& exlp)
        {
          ROS_WARN("LineParser Exception: [%s]", exlp.what());
        }
        catch(std::exception& ex)
        {
          ROS_ERROR("DBC Signal Value Type Parser Exception: [%s]", ex.what());
        }

      }
    }

    ROS_INFO("dbc.size() = %d", dbc.GetMessageCount());

    return dbc;
  }
}
