// Copyright 2012 Constantin Engelmann
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
// basic SPI implementation

#ifndef _SPI_H_
#define _SPI_H_

#include <avr/io.h>

#define MSB_FIRST 0
#define LSB_FIRST 1


#define CS_PIN    0
#define SCK_PIN   1
#define MOSI_PIN  2
#define MISO_PIN  3

static inline void Spi_Init(const uint8_t order, const uint8_t speed){

		DDRB |= _BV(SCK_PIN);		// set PB1 SCK as output
		DDRB |= _BV(MOSI_PIN);	// set PB2 MOSI as output
		DDRB |= _BV(CS_PIN);		// set PB0 CS as output
		DDRB &= ~_BV(MISO_PIN); // set PB3 MISO as input

		PORTB |= _BV(CS_PIN);		// set CS High

    // SPI enabled, configured as master.
    uint8_t configuration = _BV(SPE) | _BV(MSTR);
    if (order == LSB_FIRST) {
      configuration |= _BV(DORD);
    }
    // DoubleSpeed::clear();
    SPSR &= ~_BV(SPI2X);
    switch (speed) {
      case 2:
        // DoubleSpeed::set();
    		SPSR |= _BV(SPI2X);
      case 4:
        break;
      case 8:
        // DoubleSpeed::set();
      	SPSR |= _BV(SPI2X);
      case 16:
        configuration |= _BV(SPR0);
        break;
      case 32:
        // DoubleSpeed::set();
      	SPSR |= _BV(SPI2X);
      case 64:
        configuration |= _BV(SPR1);
        break;
      case 128:
        configuration |= _BV(SPR0);
        configuration |= _BV(SPR1);
        break;
    }
    SPCR = configuration;
}

static inline void Spi_Begin(void){
  //bring CS low
  PORTB &= ~_BV(CS_PIN); 
}

static inline void Spi_End(void){
  //bring CS high
  PORTB |= _BV(CS_PIN); 
}

//SPSR Register SPIF
inline void Spi_Wait(void){
  while( !(SPSR & _BV(SPIF)) );
}

static inline void Spi_Overwrite(uint8_t value){
  SPDR = value;
}

static inline void Spi_Send(uint8_t value){
  Spi_Overwrite(value);
  Spi_Wait();
}

static inline uint8_t Spi_ImmediateRead(void){
  return SPDR;
}

#endif // _SPI_H_