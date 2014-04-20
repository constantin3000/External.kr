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
// based on the LUFA AudioInput template by Dean Camera, 
// http://www.fourwalledcubicle.com/LUFA.php

/** \file
 *
 *  Header file for externalkr.c.
 */

#ifndef _AUDIO_INPUT_H_
#define _AUDIO_INPUT_H_

	/* Includes: */
		#include <avr/io.h>
		#include <avr/wdt.h>
		#include <avr/power.h>
		#include <avr/interrupt.h>

		#include <LUFA/Drivers/Peripheral/ADC.h>
		#include <LUFA/Drivers/USB/USB.h>

		#include "descriptors.h"
 		#include "lib/spi.h"
 		#include "lib/mcp320x.h"

	/* Macros: */
 		/*Number of Channels to read from ADC*/
 		#define CHANNEL_NUM								8
 		/*Bit of onboard LED Adafruit atmega32u4*/
 		#define LED_ONBOARD								6

	/* Function Prototypes: */
		void SetupHardware(void);
		int16_t ReadAdc(uint8_t channel);
		void AdcTask(void);

		void EVENT_USB_Device_Connect(void);
		void EVENT_USB_Device_Disconnect(void);
		void EVENT_USB_Device_ConfigurationChanged(void);
		void EVENT_USB_Device_ControlRequest(void);
		
		bool CALLBACK_Audio_Device_GetSetEndpointProperty(USB_ClassInfo_Audio_Device_t* const AudioInterfaceInfo,
		                                                  const uint8_t EndpointProperty,
		                                                  const uint8_t EndpointAddress,
		                                                  const uint8_t EndpointControl,
		                                                  uint16_t* const DataLength,
		                                                  uint8_t* Data) ATTR_NON_NULL_PTR_ARG(1);
		bool CALLBACK_Audio_Device_GetSetInterfaceProperty(USB_ClassInfo_Audio_Device_t* const AudioInterfaceInfo,
		                                                   const uint8_t Property,
		                                                   const uint8_t EntityAddress,
		                                                   const uint16_t Parameter,
		                                                   uint16_t* const DataLength,
		                                                   uint8_t* Data);
#endif

