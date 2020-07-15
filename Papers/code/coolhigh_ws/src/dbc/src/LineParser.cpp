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
 
#include <dbc/LineParser.h>

namespace NewEagle
{
  LineParser::LineParser(const std::string &line)
  {
    _line = line;
    _position = 0;
  }

  LineParser::~LineParser()
  {
  }

  int32_t LineParser::GetPosition()
  {
    return _position;
  }

  std::string LineParser::ReadCIdentifier()
  {
    SkipWhitespace();

    if (AtEOL())
    {
      throw LineParserAtEOLException();
    }

    if (!isalpha(_line[_position]) && _line[_position] != '_')
    {
      throw std::runtime_error("ReadCIdentifier: Unexpected character");
    }

    int32_t startIdx = _position;

    for(_position++; !AtEOL(); _position++)
    {
      if (!(isalpha(_line[_position]) || isdigit(_line[_position])) && _line[_position] != '_')
      {
        int32_t len = _position - startIdx;
        return _line.substr(startIdx, len);
      }
    }

    return _line.substr(startIdx, std::string::npos);
  }

  std::string LineParser::ReadCIdentifier(std::string fieldName)
  {
    std::string val = ReadCIdentifier();

    if (val == std::string())
    {
      throw std::runtime_error("Synxax Error: Expected : " + fieldName);
    }

    return val;

  }

  void LineParser::SkipWhitespace()
  {
    while(!AtEOL() && isspace(_line[_position]))
    {
      _position++;
    }
  }

  bool LineParser::AtEOL()
  {
    return _position >= _line.length();
  }

  char LineParser::ReadNextChar()
  {
    SkipWhitespace();

    if (AtEOL())
    {
      throw LineParserAtEOLException(); 
    }

    int32_t idx = _position;
    _position++;

    return _line[idx];

  }

  char LineParser::ReadNextChar(std::string fieldName)
  {
    try
    {
      char val = ReadNextChar();
      return val;
    }
    catch(LineParserExceptionBase& exlp)
    {
      throw;
    }

  }

  uint32_t LineParser::PeekUInt()
  {
    SkipWhitespace();

    if (AtEOL())
    {
      throw LineParserAtEOLException(); 
    }

    int32_t position = _position;

    int32_t startIdx = position;
    int32_t len = -1;

    for(; !AtEOL(); position++)
    {
      if (!isdigit(_line[position]))
      {
        len = position - startIdx;
        break;
      }
    }

    if (-1 == len)
    {
      len = position - startIdx;
    }

    if (0 == len)
    {
      throw LineParserLenZeroException(); 
    }

    std::istringstream reader(_line.substr(startIdx, len));
    uint32_t val;
    reader >> val;

    return val;
  }

  uint32_t LineParser::ReadUInt()
  {
    SkipWhitespace();

    if (AtEOL())
    {
      throw LineParserAtEOLException(); 
    }

    int32_t startIdx = _position;
    int32_t len = -1;

    for(; !AtEOL(); _position++)
    {
      if (!isdigit(_line[_position]))
      {
        len = _position - startIdx;
        break;
      }
    }

    if (-1 == len)
    {
      len = _position - startIdx;
    }

    if (0 == len)
    {
      throw LineParserLenZeroException(); 
    }

    std::istringstream reader(_line.substr(startIdx, len));
    uint32_t val;
    reader >> val;

    return val;
  }

  uint32_t LineParser::ReadUInt(std::string fieldName)
  {
    try
    {
      uint32_t val = ReadUInt();
      return val;
    }
    catch(LineParserExceptionBase& exlp)
    {
      throw;
    }
  }

  int32_t LineParser::ReadInt()
  {
    SkipWhitespace();

    if (AtEOL())
    {
      throw LineParserAtEOLException(); 
    }

    if (!isdigit(_line[_position]) && _line[_position] != '-' && _line[_position] != '+')
    {
      throw LineParserInvalidCharException(); 
    }

    int32_t startIdx = _position;
    int32_t len = -1;

    for(_position++; !AtEOL(); _position++)
    {
      if (!isdigit(_line[_position]))
      {
        len = _position - startIdx;
        break;
      }
    }

    if (-1 == len)
    {
      len = _position - startIdx;
    }

    if (0 == len)
    {
      throw LineParserLenZeroException(); 
    }

    std::istringstream reader(_line.substr(startIdx, len));
    int32_t val;
    reader >> val;

    return val;

  }

  double LineParser::ReadDouble()
  {
    SkipWhitespace();

    if (AtEOL())
    {
      throw LineParserAtEOLException(); 
    }

    if (!isdigit(_line[_position]) && _line[_position] != '-' && _line[_position] != '+')
    {
      throw LineParserInvalidCharException(); 
    }

    int32_t startIdx = _position;
    int32_t len = -1;

    NewEagle::ReadDoubleState state = NewEagle::READING_WHOLE_NUMBER;

    for(_position++; !AtEOL(); _position++)
    {
      char c = _line[_position];

      switch (state) {
        case NewEagle::READING_WHOLE_NUMBER:
          if ('E' == c || 'e' == c)
          {
            state = NewEagle::READ_E;
          }
          else if('.' == c)
          {
            state = NewEagle::READING_FRACTION;
          }
          else if (!isdigit(c))
          {
            goto DoneReading;
          }
          break;
        case NewEagle::READING_FRACTION:
          if ('E' == c || 'e' == c)
          {
            state = NewEagle::READ_E;
          }
          else if(!isdigit(c))
          {
            goto DoneReading;
          }
          break;
        case NewEagle::READ_E:
          if ('+' == c || '-' == c)
          {
            state = NewEagle::READ_SIGN;
          }
          else if (isdigit(c))
          {
            state = NewEagle::READING_EXP;
          }
          break;
        case NewEagle::READ_SIGN:
          if (!isdigit(c))
          {
            throw LineParserInvalidCharException();
          }
          else
          {
            state = NewEagle::READING_EXP;
          }

          break;
        case NewEagle::READING_EXP:
          if (!isdigit(c))
          {
            goto DoneReading;
          }
          break;
        default:
          break;

      }

    }

DoneReading:
    len = _position - startIdx;

    if (0 == len)
    {
      throw LineParserLenZeroException(); 
    }

    std::istringstream reader(_line.substr(startIdx, len));
    double val;
    reader >> val;

    return val;

  }

  double LineParser::ReadDouble(std::string fieldName)
  {
    try
    {
      double val = ReadDouble();
     return val;
    }
    catch(LineParserExceptionBase& exlp)
    {
      throw;
    }

  }

  void LineParser::SeekSeparator(char separator)
  {
    char nextChar = ReadNextChar();

    if (nextChar == 0x00 || nextChar != separator)
    {
      throw std::runtime_error("Synxax Error: Expected : " + separator);
    }
  }

  std::string LineParser::ReadQuotedString()
  {
    SkipWhitespace();

    if (AtEOL())
    {
      throw LineParserAtEOLException(); 
    }

    if (_line[_position] != '"')
    {
      throw std::runtime_error("ReadQuotedString: Missing Quote");
    }

    int32_t startIdx = ++_position;
    int32_t len  = -1;

    for (; _position < _line.size(); _position++)
    {
      if (_line[_position] == '"')
      {
        len = _position - startIdx;
        _position++;
        break;
      }
    }

    if (-1 == len)
    {
      throw LineParserLenZeroException();
    }

    if (0 == len)
    {
      throw LineParserLenZeroException(); 
    }

    return _line.substr(startIdx, len);
  }
}
