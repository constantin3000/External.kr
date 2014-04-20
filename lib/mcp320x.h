// Copyright 2013 Constantin Engelmann
//
// Author: Constantin Engelmann
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
//
// communicate with a MCP 320x ADC

#ifndef _MCP320X_H_
#define _MCP320X_H_

#include "spi.h"

#define SINGLE_ENDED 1
#define DIFFERENTIAL 0

static inline int16_t ReadAdc(uint8_t _channel){
  uint8_t command;
  uint8_t msb = 0;
  uint8_t lsb = 0;
  uint8_t input_mode = SINGLE_ENDED; // single ended = 1, differential = 0
  uint16_t result;

  command = 0x04; // start flag
  command |= (input_mode<<1); // shift input_mode
  command |= (_channel>>2) & 0x01; // add msb of channel in our first command byte

  Spi_Begin();
  Spi_Send(command);
  Spi_Send(_channel<<6);
  msb = Spi_ImmediateRead() & 0x0f;
  Spi_Send(0xff);
  lsb = Spi_ImmediateRead();
  Spi_End();

  result = msb<<8 | lsb;
  result = result<<4;
  result = result - (0xffff>>1);
  return result;
}

#endif //_MCP320X_H_