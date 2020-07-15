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
 
#ifndef _NEW_EAGLE_LINEPARSER_H
#define _NEW_EAGLE_LINEPARSER_H

#include <ros/ros.h>

#include <string>
#include <exception>
#include <ctype.h>

namespace NewEagle
{
  class LineParserExceptionBase: public std::exception
  {
  };

  class LineParserAtEOLException: public LineParserExceptionBase
  {
    virtual const char* what() const throw()
    {
      return "Unexpected end of line.";
    }
  };

  class LineParserLenZeroException: public LineParserExceptionBase
  {
    virtual const char* what() const throw()
    {
      return "Nothing found in search space.";
    }
  };

  class LineParserInvalidCharException: public LineParserExceptionBase
  {
    virtual const char* what() const throw()
    {
      return "Invalid character(s) search space.";
    }
  };

  enum ReadDoubleState
  {
    READING_WHOLE_NUMBER = 0,
    READING_FRACTION = 1,
    READ_E = 2,
    READ_SIGN = 3,
    READING_EXP = 4
  };

  class LineParser
  {
    public:
      LineParser(const std::string &line);
      ~LineParser();

      int32_t GetPosition();
      std::string ReadCIdentifier();
      std::string ReadCIdentifier(std::string fieldName);
      uint32_t ReadUInt();
      uint32_t ReadUInt(std::string fieldName);
      void SeekSeparator(char separator);
      char ReadNextChar(std::string fieldName);
      int32_t ReadInt();
      double ReadDouble();
      double ReadDouble(std::string fieldName);
      std::string ReadQuotedString();
      uint32_t PeekUInt();

    private:
      int32_t _position;
      std::string _line;

      void SkipWhitespace();
      bool AtEOL();
      char ReadNextChar();
  };
}

#endif // _NEW_EAGLE_LINEPARSER_H
