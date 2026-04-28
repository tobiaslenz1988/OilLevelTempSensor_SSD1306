/*
  This example is for the combined OilLevel/Temoerature Sensor G266 of the VAG Family.
  There are two types of the Sensor:

  The "old" Version, used in 
  -Audi A4 (B5) 1.8 20V 
  -Audi A2 
  -VW Transporter T4
  -VW Golf 5 1.9 TDI
  -etc...

  The old Version has the Part Numbers 
  - 1J0 907 660 
  - 1J0 907 660 A
  - 1J0 907 660 B
  - 1J0 907 660 C
  - 1J0 907 660 F

  The old Version has a metal big Plate at the Bottom and the Tube to measure the oil is ONE PIECE and is formed in a rectangular form and NOT Square.


  The "new" Version, used in 
  - Audi A4, A6 from 2010
  - Audi Q5 from 2010
  - etc

  The new Version has the Part Numbers 
  03C 907 660 M


  In the new Version the Bottom is made of plastic and the Tube to measure the oil is Two PIECES. The Tube itself is formed in a H-Design and at the top of the Tube is a square 2nd Part



  This code is based on the Information from https://www.mikrocontroller.net/topic/459687 (which uses the "old" Version)

  The Sensor has 3 Pins

  Pinning:
  Pin    Pin    Pin 
   1      2      3
  12V    Gnd    Signal (High 200mV)


  - As Default Pin 2 used as an Input for the Signal of the Sensor 
  - As Default Pin 4 kind of Debug Pin to verify how often the Input ISR is triggered
  - As Default Pin 13 is used for an LED for the Oil level which turns on if theres is to low 
  - As Default Pin 15 is used for an LED to show the oil Temperature
    The LED is on if Engine is two cold
    The LED is off if Engine is in normal temperature area
    The LED toggles if the Engine is too Hot

  Circuit:
  Between the 3.3V output from the ESP 32 and the Input Pin is a 4.7K Ohm resistor as a Pullup resistor.
  The Output Signal Pin is connected to the InputPin of the ESP32
  The Ground of the Sensor is connected to Ground of the ESP32
  The 



        The Signal looks like the following

-        ______      _______                                   __
-        |     |     |     |                                  |
-        |     |     |     |                                  |
-        |     |     |     |                                  |
-      __|     |_____|     |__________________________________|
-      T1   T2   T3    T4                  T5                    T1



You can controle the Module by the help of the implemented UDS Protocol

There are the Services:
- 0x22 (ReadDataByIdentifier)
- 0x2E (ReadDataByIdentifier)
- 0x10 (SessionControl)



The Services 0x22 (ReadDataByIdentifier): has the following Subfunctions

- 0x22 0xF1 0xAB Read SW-Version                                               (returns the SW Version)
- 0x22 0xF1 0x87 Read Debugflag                                                (Returns if the Debug is on (true) or not (false))
- 0x22 0xF1 0x05 Read Modulename                                               (Returns the Name of the Module)
- 0x22 0x06 0x00 0x??                       Read DebugOilLevelPercentage        You can write the DebugValue of OilLevelPercentage ...
- 0x22 0x06 0x01 0x??                       Read DebugOilTemperature            You can write the DebugValue of OilTemperature...
- 0x22 0x06 0x02 0x??                       Read Extra OutputPin                Get The extra Outpin to the desired value
- 0x22 0x06 0x03 0x??                       Read New Old SensorFlag             Get The flag of new or old sensor is equipped
- 0x22 0x06 0x04 0x??                       Read Real Oil Tempeature            Get The evalue of the oil Temperature
- 0x22 0x06 0x05 0x??                       Read New Old SensorFlag             Get The evalue of the oil Level
- 0x22 0x06 0x06 0x??                       Read the OilTempCompareValues       Shows which impulsevalues are used to compare the Oiltemp... starting from Oiltempeatur 30°C...Pay attention the values are uint16_t byte[3] (highbyte) byte[4](lowbyte) tempersture30...
- 0x22 0x06 0x07 0x??                       Read the OilLevelCompareValues      Shows which impulsevalues are used to compare the Oillevel... starting from Oillevel empty...Pay attention the values are uint16_t byte[3] (highbyte) byte[4](lowbyte) oillevel empty...



The Services 0x2E (WriteDataByIdentifier): has the following Subfunctions

- 0x2E 0xF1 0xAB 0x04   0x'' 0x'' 0x'' 0x'' Write SW-Version                   You can write the sw Version... is always 4 ASCIIs therefore the fourth Byte (Byte[3)) has to be always 0x04
- 0x2E 0xF1 0x05 0xlen  0x'' ..             Write to ModuleName (max.15 chars) You can write the Modulename... the foruth byte (BYTE[3] consits the length (max. 15)
- 0x2E 0x06 0x00 0x??                       Write DebugOilLevelPercentage      You can write the DebugValue of OilLevelPercentage ... 
- 0x2E 0x06 0x01 0x??                       Write DebugOilTemperature          You can write the DebugValue of OilTemperature... 
- 0x2E 0x06 0x02 0x??                       Write Extra OutputPin              Set The extra Outpin to the desired value 
- 0x2E 0x06 0x03 0x??                       Write New Old SensorFlag           Set The flag of new or old sensor is equipped 
- 0x2E 0x06 0x06 0xlen                       Read the OilTempCompareValues       Shows which impulsevalues are used to compare the Oiltemp... starting from Oiltempeatur 30°C...Pay attention the values are uint16_t byte[3] (highbyte) byte[4](lowbyte) tempersture30...
- 0x2E 0x06 0x07 0xlen                      Read the OilLevelCompareValues      Shows which impulsevalues are used to compare the Oillevel... starting from Oillevel empty...Pay attention the values are uint16_t byte[3] (highbyte) byte[4](lowbyte) oillevel empty...



0x10 (SessionControl):
0x10 0x00 (Set Session to Default Session-> Debug Off, no extra Information Output to Bluetooth)
0x10 0x03 (Set Session to Extended Session-> Debug On, Information output Output to Bluetooth)

*/
