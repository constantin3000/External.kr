A basic 8 channel sound card (8kHz 16bit) running on a atmega32u4 controller. C mcp3208 is being used as ADC. The USB device implemets a card with two devices each having 4 streams. 
```
32u4		MCP3208
PB0 CS	 	10	
PB1 CLK  	13
PB2	MOSI 	11
PB3 MISO 	12
```
It is based on the (LUFA library)[http://www.fourwalledcubicle.com/]
To compile you need to download the library and place it in the same root directory. A compiled hex is in the repository. 