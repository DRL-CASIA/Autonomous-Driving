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
 
#include <dbc/DbcSignal.h>

namespace NewEagle
{
  DbcSignal::DbcSignal(
    uint8_t dlc,
    double gain,
    double offset,
    uint8_t startBit,
    ByteOrder endianness,
    uint8_t length,
    SignType sign,
    std::string name,
    MultiplexerMode multiplexerMode)
  {
    _dlc = dlc;
    _gain  = gain;
    _offset = offset;
    _startBit = startBit;
    _endianness = endianness;
    _length = length;
    _sign = sign;
    _name = name;
    _multiplexerMode = multiplexerMode;
  }

  DbcSignal::DbcSignal(
    uint8_t dlc,
    double gain,
    double offset,
    uint8_t startBit,
    ByteOrder endianness,
    uint8_t length,
    SignType sign,
    std::string name,
    MultiplexerMode multiplexerMode,
    int32_t multiplexerSwitch)
  {
    _dlc = dlc;
    _gain  = gain;
    _offset = offset;
    _startBit = startBit;
    _endianness = endianness;
    _length = length;
    _sign = sign;
    _name = name;
    _multiplexerMode = multiplexerMode;
    _multiplexerSwitch = multiplexerSwitch;
  }

  DbcSignal::~DbcSignal()
  {
  }

  uint8_t DbcSignal::GetDlc() const
  {
    return _dlc;
  }

  double DbcSignal::GetResult() const
  {
    return _result;
  }

  double DbcSignal::GetGain() const
  {
    return _gain;
  }

  double DbcSignal::GetOffset() const
  {
    return _offset;
  }

  uint8_t DbcSignal::GetStartBit() const
  {
    return _startBit;
  }

  ByteOrder DbcSignal::GetEndianness() const
  {
    return _endianness;
  }

  uint8_t DbcSignal::GetLength() const
  {
    return _length;
  }

  SignType DbcSignal::GetSign() const
  {
    return _sign;
  }

  std::string DbcSignal::GetName() const
  {
    return _name;
  }

  void DbcSignal::SetResult(double result)
  {
    _result = result;
  }

  void DbcSignal::SetComment(NewEagle::DbcSignalComment comment)
  {
    _comment = comment;
  }

  void DbcSignal::SetInitialValue(double value)
  {
    _initialValue = value;
  }
  double DbcSignal::GetInitialValue()
  {
    return _initialValue;
  }

  void DbcSignal::SetDataType(NewEagle::DataType type)
  {
    _type = type;
  }

  NewEagle::DataType DbcSignal::GetDataType()
  {
    return _type;
  }

  NewEagle::MultiplexerMode DbcSignal::GetMultiplexerMode() const
  {
    return _multiplexerMode;
  }

  int32_t DbcSignal::GetMultiplexerSwitch() const
  {
    return _multiplexerSwitch;
  }
}
