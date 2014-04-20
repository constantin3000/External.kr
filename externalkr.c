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
// 
// implemented 8 channel 12bit ADC (MCP 3208)
//

/** \file
 *
 *  Main source file for the AudioInput demo. This file contains the main tasks of
 *  the demo and is responsible for the initial application hardware configuration.
 */

#include "externalkr.h"

/** LUFA Audio Class driver interface configuration and state information. This structure is
 *  passed to all Audio Class driver functions, so that multiple instances of the same class
 *  within a device can be differentiated from one another.
 */

 USB_ClassInfo_Audio_Device_t Microphone_Audio_Interface =
  {
    .Config =
      {
        .ControlInterfaceNumber   = 0,
        .StreamingInterfaceNumber = 1,
        .DataINEndpoint           =
          {
            .Address          = AUDIO_STREAM_EPADDR,
            .Size             = AUDIO_STREAM_EPSIZE,
            .Banks            = 2,
          },
      },
  };

 USB_ClassInfo_Audio_Device_t Microphone_Audio_Interface_1 =
  {
    .Config =
      {
        .ControlInterfaceNumber   = 2,
        .StreamingInterfaceNumber = 3,
        .DataINEndpoint           =
          {
            .Address          = AUDIO_STREAM_EPADDR_1,
            .Size             = AUDIO_STREAM_EPSIZE_1,
            .Banks            = 2,
          },
      },
  };
/** Current audio sampling frequency of the streaming audio endpoint. */
static uint32_t CurrentAudioSampleFrequency = 8000;
static int16_t adc_samples[8];
static uint8_t adc_channel = 0;

/** Main program entry point. This routine contains the overall program flow, including initial
 *  setup of all components and the main program loop.
 */
int main(void)
{
  SetupHardware();
  GlobalInterruptEnable();

  for (;;)
  {
    Audio_Device_USBTask(&Microphone_Audio_Interface);
    Audio_Device_USBTask(&Microphone_Audio_Interface_1);
    USB_USBTask();
    AdcTask();
  }
}

void AdcTask(void){
  adc_samples[adc_channel] = ReadAdc(adc_channel);
  adc_channel++;
  if(adc_channel == CHANNEL_NUM){
    adc_channel = 0;
  }
}

/** Configures the board hardware and chip peripherals for the demo's functionality. */
void SetupHardware(void)
{
  /* Disable watchdog if enabled by bootloader/fuses */
  MCUSR &= ~(1 << WDRF);
  wdt_disable();

  /* Disable clock division */
  clock_prescale_set(clock_div_1);

  /* Hardware Initialization */
  DDRE |= _BV(LED_ONBOARD);   // set as output
  PORTE &= ~_BV(LED_ONBOARD); // low

  USB_Init();
  Spi_Init(MSB_FIRST, 4);
}

/** ISR to handle the reloading of the data endpoint with the next sample. */
ISR(TIMER0_COMPA_vect, ISR_BLOCK)
{
  uint8_t PrevEndpoint = Endpoint_GetCurrentEndpoint();

  /* Check that the USB bus is ready for the next sample to write */
  if (Audio_Device_IsReadyForNextSample(&Microphone_Audio_Interface))
  {
    Audio_Device_WriteSample16(&Microphone_Audio_Interface, adc_samples[0]);
    Audio_Device_WriteSample16(&Microphone_Audio_Interface, adc_samples[1]);
    Audio_Device_WriteSample16(&Microphone_Audio_Interface, adc_samples[2]);
    Audio_Device_WriteSample16(&Microphone_Audio_Interface, adc_samples[3]);
    // Audio_Device_WriteSample16(&Microphone_Audio_Interface, 0xffff);
    // Audio_Device_WriteSample16(&Microphone_Audio_Interface, 0x1fff);
  }

  if (Audio_Device_IsReadyForNextSample(&Microphone_Audio_Interface_1))
  {
    Audio_Device_WriteSample16(&Microphone_Audio_Interface_1, adc_samples[4]);
    Audio_Device_WriteSample16(&Microphone_Audio_Interface_1, adc_samples[5]);
    Audio_Device_WriteSample16(&Microphone_Audio_Interface_1, adc_samples[6]);
    Audio_Device_WriteSample16(&Microphone_Audio_Interface_1, adc_samples[7]);
    // Audio_Device_WriteSample16(&Microphone_Audio_Interface, 0xffff);
    // Audio_Device_WriteSample16(&Microphone_Audio_Interface, 0x1fff);
  }

  Endpoint_SelectEndpoint(PrevEndpoint);
}

/** Event handler for the library USB Connection event. */
void EVENT_USB_Device_Connect(void)
{
  // LEDs_SetAllLEDs(LEDMASK_USB_ENUMERATING);

  /* Sample reload timer initialization */
  TIMSK0  = (1 << OCIE0A);
  OCR0A   = ((F_CPU / 8 / CurrentAudioSampleFrequency) - 1);
  TCCR0A  = (1 << WGM01);  // CTC mode
  TCCR0B  = (1 << CS01);   // Fcpu/8 speed
}

/** Event handler for the library USB Disconnection event. */
void EVENT_USB_Device_Disconnect(void)
{
  /* Stop the sample reload timer */
  TCCR0B = 0;
  PORTE &= ~_BV(LED_ONBOARD); // low

  // LEDs_SetAllLEDs(LEDMASK_USB_NOTREADY);
}

/** Event handler for the library USB Configuration Changed event. */
void EVENT_USB_Device_ConfigurationChanged(void)
{
  bool ConfigSuccess = true;

  ConfigSuccess &= Audio_Device_ConfigureEndpoints(&Microphone_Audio_Interface);
  ConfigSuccess &= Audio_Device_ConfigureEndpoints(&Microphone_Audio_Interface_1);

  if(ConfigSuccess){
    PORTE |= _BV(LED_ONBOARD);
  } else {
    PORTE &= _BV(LED_ONBOARD);
  }
  // LEDs_SetAllLEDs(ConfigSuccess ? LEDMASK_USB_READY : LEDMASK_USB_ERROR);
}

/** Event handler for the library USB Control Request reception event. */
void EVENT_USB_Device_ControlRequest(void)
{
  Audio_Device_ProcessControlRequest(&Microphone_Audio_Interface);
  Audio_Device_ProcessControlRequest(&Microphone_Audio_Interface_1);
}

/** Audio class driver callback for the setting and retrieval of streaming endpoint properties. This callback must be implemented
 *  in the user application to handle property manipulations on streaming audio endpoints.
 *
 *  When the DataLength parameter is NULL, this callback should only indicate whether the specified operation is valid for
 *  the given endpoint index, and should return as fast as possible. When non-NULL, this value may be altered for GET operations
 *  to indicate the size of the retrieved data.
 *
 *  \note The length of the retrieved data stored into the Data buffer on GET operations should not exceed the initial value
 *        of the \c DataLength parameter.
 *
 *  \param[in,out] AudioInterfaceInfo  Pointer to a structure containing an Audio Class configuration and state.
 *  \param[in]     EndpointProperty    Property of the endpoint to get or set, a value from Audio_ClassRequests_t.
 *  \param[in]     EndpointAddress     Address of the streaming endpoint whose property is being referenced.
 *  \param[in]     EndpointControl     Parameter of the endpoint to get or set, a value from Audio_EndpointControls_t.
 *  \param[in,out] DataLength          For SET operations, the length of the parameter data to set. For GET operations, the maximum
 *                                     length of the retrieved data. When NULL, the function should return whether the given property
 *                                     and parameter is valid for the requested endpoint without reading or modifying the Data buffer.
 *  \param[in,out] Data                Pointer to a location where the parameter data is stored for SET operations, or where
 *                                     the retrieved data is to be stored for GET operations.
 *
 *  \return Boolean \c true if the property get/set was successful, \c false otherwise
 */
bool CALLBACK_Audio_Device_GetSetEndpointProperty(USB_ClassInfo_Audio_Device_t* const AudioInterfaceInfo,
                                                  const uint8_t EndpointProperty,
                                                  const uint8_t EndpointAddress,
                                                  const uint8_t EndpointControl,
                                                  uint16_t* const DataLength,
                                                  uint8_t* Data)
{
  /* Check the requested endpoint to see if a supported endpoint is being manipulated */
  if (EndpointAddress == Microphone_Audio_Interface.Config.DataINEndpoint.Address)
  {
    /* Check the requested control to see if a supported control is being manipulated */
    if (EndpointControl == AUDIO_EPCONTROL_SamplingFreq)
    {
      switch (EndpointProperty)
      {
        case AUDIO_REQ_SetCurrent:
          /* Check if we are just testing for a valid property, or actually adjusting it */
          if (DataLength != NULL)
          {
            /* Set the new sampling frequency to the value given by the host */
            CurrentAudioSampleFrequency = (((uint32_t)Data[2] << 16) | ((uint32_t)Data[1] << 8) | (uint32_t)Data[0]);

            /* Adjust sample reload timer to the new frequency */
            OCR0A = ((F_CPU / 8 / CurrentAudioSampleFrequency) - 1);
          }

          return true;
        case AUDIO_REQ_GetCurrent:
          /* Check if we are just testing for a valid property, or actually reading it */
          if (DataLength != NULL)
          {
            *DataLength = 3;

            Data[2] = (CurrentAudioSampleFrequency >> 16);
            Data[1] = (CurrentAudioSampleFrequency >> 8);
            Data[0] = (CurrentAudioSampleFrequency &  0xFF);
          }

          return true;
      }
    }
  }

    /* Check the requested endpoint to see if a supported endpoint is being manipulated */
  if (EndpointAddress == Microphone_Audio_Interface_1.Config.DataINEndpoint.Address)
  {
    /* Check the requested control to see if a supported control is being manipulated */
    if (EndpointControl == AUDIO_EPCONTROL_SamplingFreq)
    {
      switch (EndpointProperty)
      {
        case AUDIO_REQ_SetCurrent:
          /* Check if we are just testing for a valid property, or actually adjusting it */
          if (DataLength != NULL)
          {
            /* Set the new sampling frequency to the value given by the host */
            CurrentAudioSampleFrequency = (((uint32_t)Data[2] << 16) | ((uint32_t)Data[1] << 8) | (uint32_t)Data[0]);

            /* Adjust sample reload timer to the new frequency */
            OCR0A = ((F_CPU / 8 / CurrentAudioSampleFrequency) - 1);
          }

          return true;
        case AUDIO_REQ_GetCurrent:
          /* Check if we are just testing for a valid property, or actually reading it */
          if (DataLength != NULL)
          {
            *DataLength = 3;

            Data[2] = (CurrentAudioSampleFrequency >> 16);
            Data[1] = (CurrentAudioSampleFrequency >> 8);
            Data[0] = (CurrentAudioSampleFrequency &  0xFF);
          }

          return true;
      }
    }
  }

  return false;
}

/** Audio class driver callback for the setting and retrieval of streaming interface properties. This callback must be implemented
 *  in the user application to handle property manipulations on streaming audio interfaces.
 *
 *  When the DataLength parameter is NULL, this callback should only indicate whether the specified operation is valid for
 *  the given entity and should return as fast as possible. When non-NULL, this value may be altered for GET operations
 *  to indicate the size of the retrieved data.
 *
 *  \note The length of the retrieved data stored into the Data buffer on GET operations should not exceed the initial value
 *        of the \c DataLength parameter.
 *
 *  \param[in,out] AudioInterfaceInfo  Pointer to a structure containing an Audio Class configuration and state.
 *  \param[in]     Property            Property of the interface to get or set, a value from Audio_ClassRequests_t.
 *  \param[in]     EntityAddress       Address of the audio entity whose property is being referenced.
 *  \param[in]     Parameter           Parameter of the entity to get or set, specific to each type of entity (see USB Audio specification).
 *  \param[in,out] DataLength          For SET operations, the length of the parameter data to set. For GET operations, the maximum
 *                                     length of the retrieved data. When NULL, the function should return whether the given property
 *                                     and parameter is valid for the requested endpoint without reading or modifying the Data buffer.
 *  \param[in,out] Data                Pointer to a location where the parameter data is stored for SET operations, or where
 *                                     the retrieved data is to be stored for GET operations.
 *
 *  \return Boolean \c true if the property GET/SET was successful, \c false otherwise
 */
bool CALLBACK_Audio_Device_GetSetInterfaceProperty(USB_ClassInfo_Audio_Device_t* const AudioInterfaceInfo,
                                                   const uint8_t Property,
                                                   const uint8_t EntityAddress,
                                                   const uint16_t Parameter,
                                                   uint16_t* const DataLength,
                                                   uint8_t* Data) 
{
  /* No audio interface entities in the device descriptor, thus no properties to get or set. */
  return false;
}
