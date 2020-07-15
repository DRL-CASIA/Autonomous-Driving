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
 
#ifndef _NEW_EAGLE_DBC_UTILITIES_H
#define _NEW_EAGLE_DBC_UTILITIES_H

#include <ros/ros.h>
#include <map>
#include <sstream>      // std::istringstream
#include <string>

#include <dbc/DbcSignal.h>
#include <dbc/DbcMessage.h>

namespace NewEagle
{
  static int32_t ConvertToMTBitOrdering(uint32_t bit, uint32_t dlc)
  {
    if (bit < 0 || bit >- dlc * 8)
    {
      return -1;
    }

    int32_t msgBitLength = (int32_t)dlc * 8;

    int32_t row = (int32_t) bit / 8;
    int32_t offset = (int32_t)bit % 8;

    return (msgBitLength - (row + 1) * 8) + offset;
  }

  static int32_t ConvertToMTBitOrdering(uint32_t bit)
  {
    return ConvertToMTBitOrdering(bit, 8);

  }

  static double Unpack(uint8_t* data, const NewEagle::DbcSignal &signal)
  {
    int32_t wordSize = sizeof(data);
    int32_t startBit = (int32_t)signal.GetStartBit();

    if (signal.GetEndianness() == NewEagle::LITTLE_END)
    {
      startBit = ConvertToMTBitOrdering(signal.GetStartBit(), signal.GetDlc());
    }
    else
    {
      startBit = ConvertToMTBitOrdering(signal.GetStartBit(), signal.GetDlc()) - ((int32_t)signal.GetLength() - 1);
    }

    int32_t bit = (int32_t)(startBit % 8);

    bool isExactlyByte = ((bit + signal.GetLength()) % 8 == 0);
    uint32_t numBytes = (isExactlyByte ? 0 : 1) + ((bit + (int32_t)signal.GetLength()) / 8);

    int32_t b = (int32_t)wordSize - ((int)startBit / 8) - 1;
    int32_t w = (int)signal.GetLength();
    int32_t maskShift = bit;
    int32_t rightShift = 0;

    uint32_t unsignedResult = 0;
    for(int32_t i = 0; i < numBytes; i++)
    {
      if ((b < 0) || (b >= sizeof(data)))
      {
        return std::numeric_limits<int>::quiet_NaN();
      }

      int32_t mask = 0xFF;
      if (w < 8)
      {
        mask >>= (8 - w);
      }
      mask <<= maskShift;

      int32_t extractedByte = (data[b] & mask) >> maskShift;
      unsignedResult |= (uint32_t)extractedByte << (8 * i - rightShift);

      if (signal.GetEndianness() == NewEagle::BIG_END)
      {
        if ((b % wordSize) == 0)
        {
          b += 2 * wordSize - 1;
        }
        else
        {
          b--;
        }
      }
      else
      {
        b++;
      }
      
      w -= ( 8 - maskShift);
      rightShift += maskShift;
      maskShift = 0;

    }

    double result = 0;
    if (signal.GetSign() == NewEagle::SIGNED)
    {
      if ((unsignedResult & (1 << (int32_t)signal.GetLength() - 1)) != 0)
      {
        if (signal.GetLength() < 32)
        {
          uint32_t signExtension = (0xFFFFFFFF << (int32_t)signal.GetLength());
          unsignedResult |= signExtension;
        }
      }

      result = (double)((int32_t)unsignedResult);

    }
    else if (signal.GetSign() == NewEagle::UNSIGNED)
    {
      result = (double)(unsignedResult);
    }

    if ((signal.GetGain() != 1) || (signal.GetOffset() != 0))
    {
      result *= signal.GetGain();
      result += signal.GetOffset();
    }

    return result;
  }

  static void Pack(uint8_t * data, const NewEagle::DbcSignal &signal)
  {
    uint32_t result = 0;

    double tmp = signal.GetResult();

    if ((signal.GetGain() != 1) || (signal.GetOffset() != 0))
    {
      tmp -= signal.GetOffset();
      tmp /= signal.GetGain();
    }

    if (signal.GetSign() == NewEagle::SIGNED)
    {
      int32_t i = (int32_t)tmp;
      uint32_t u = (uint)i;

      result = u;
    }
    else
    {
      result = (uint)tmp;
    }

    int8_t wordSize = sizeof(data);
    int8_t startBit = (int)signal.GetStartBit();

    if (signal.GetEndianness() == NewEagle::LITTLE_END)
    {
      startBit = ConvertToMTBitOrdering(signal.GetStartBit(), signal.GetDlc());
    }
    else
    {
      startBit = ConvertToMTBitOrdering(signal.GetStartBit(), signal.GetDlc()) - ((int32_t)signal.GetLength() - 1);
    }

    int32_t bit = (int32_t)(startBit % 8);

    bool isExactlyByte = ((bit + signal.GetLength()) % 8 == 0);
    uint32_t numBytes = (isExactlyByte ? 0 : 1) + ((bit + (int32_t)signal.GetLength()) / 8);

    int32_t b = (int32_t)wordSize - ((int)startBit / 8) - 1;
    int32_t w = (int)signal.GetLength();
    int32_t maskShift = bit;
    int32_t rightShift = 0;

    uint8_t mask = 0xFF;
    uint32_t extractedByte;

    for(int i = 0; i < numBytes; i++)
    {
      if ((b < 0 || (b >= sizeof(data))))
      {
        return;
      }

      mask = 0xFF;

      if (w < 8)
      {
        mask >>= (8 - w);
      }

      mask <<= maskShift;

      extractedByte = (result >> (8 * i - rightShift)) & 0xFF;

      data[b] = (uint32_t)(data[b] & ~mask);
      data[b] |= (uint8_t)((extractedByte << maskShift) & mask);

      if (signal.GetEndianness() == NewEagle::BIG_END)
      {
        if ((b % wordSize) == 0)
        {
          b += 2 * wordSize - 1;
        }
        else
        {
          b--;
        }
      }
      else
      {
        b++;
      }

      w -= ( 8 - maskShift);
      rightShift += maskShift;
      maskShift = 0;
    }

  }

}

#endif // _NEW_EAGLE_DBC_UTILITIES_H
