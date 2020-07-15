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
 
#ifndef _NEW_EAGLE_DBC_SIGNAL_H
#define _NEW_EAGLE_DBC_SIGNAL_H

#include <ros/ros.h>

#include <string>

namespace NewEagle
{
  struct DbcSignalComment
  {
    uint32_t Id;
    std::string SignalName;
    std::string Comment;
  };

  enum SignType
  {
    UNSIGNED = 0,
    SIGNED = 1
  };

  enum ByteOrder
  {
    LITTLE_END = 0,
    BIG_END = 1
  };

  enum DataType
  {
    INT = 0,
    FLOAT = 1,
    DOUBLE = 2
  };

  enum MultiplexerMode
  {
    NONE = 0,
    MUX_SWITCH = 1,
    MUX_SIGNAL = 2
  };

  class DbcSignal {
    public:
      DbcSignal(
        uint8_t dlc,
        double gain,
        double offset,
        uint8_t startBit,
        ByteOrder endianness,
        uint8_t length,
        SignType sign,
        std::string name, 
        MultiplexerMode multiplexerMode);

      DbcSignal(
        uint8_t dlc,
        double gain,
        double offset,
        uint8_t startBit,
        ByteOrder endianness,
        uint8_t length,
        SignType sign,
        std::string name, 
        MultiplexerMode multiplexerMode,
        int32_t multiplexerSwitch);

      ~DbcSignal();

      uint8_t GetDlc() const;
      double GetResult() const;
      double GetGain() const;
      double GetOffset() const;
      uint8_t GetStartBit() const;
      ByteOrder GetEndianness() const;
      uint8_t GetLength() const;
      SignType GetSign() const;
      std::string GetName() const;
      void SetResult(double result);
      void SetComment(NewEagle::DbcSignalComment comment);
      void SetInitialValue(double value);
      double GetInitialValue();
      DataType GetDataType();
      void SetDataType(DataType type);
      MultiplexerMode GetMultiplexerMode() const;
      int32_t GetMultiplexerSwitch() const;      

    private:
      uint8_t _dlc;
      double _result;
      double _gain;
      double _offset;
      uint8_t _startBit;
      ByteOrder _endianness;
      uint8_t _length;
      SignType _sign;
      std::string _name;
      NewEagle::DbcSignalComment _comment;
      double _initialValue;
      DataType _type;
      MultiplexerMode _multiplexerMode;
      int32_t _multiplexerSwitch;
  };
}

#endif // _NEW_EAGLE_DBC_SIGNAL_H
