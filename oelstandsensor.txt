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
  1J0 907 660 
  1J0 907 660 A
  1J0 907 660 B
  1J0 907 660 C
  1J0 907 660 F

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
  - As Default Pin 4 kind of Debug Pin to verify how often the Signalinput ISR is triggered
  - As Default Pin 13 is used for an LED for the Oil level which turns on if theres is to low 
  - As Default Pin 15 is used for an LED to show the oil Temperature
    The LED is on if Engine is two cold
    The LED is off if Engine is in normal temperature area
    The LED toggles if the Engine is too Hot

  Circuit:
  Between the 3.3V output from the ESP 32n is a 4.7K Ohm resistor as a Pullup resistor.
  The Ground if the Sensor is connected to Ground of the Sensor
  The 

*/

/*
The Signal looks like the following

  ______      _______                                   __
  |     |     |     |                                  |
  |     |     |     |                                  |
  |     |     |     |                                  |
__|     |_____|     |__________________________________|
T1   T2   T3    T4                  T5                    T1
*/


#include <BluetoothSerial.h>
#include <Preferences.h>
#include <stdio.h>
#include <string.h>
#include <Arduino.h>

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "oilsensor.h"
#include "NRC_UDS_protocol.h"
#include "oillevel_ssd1603.h"
#include "Oil_Level_Graphic/Image_00_SSD1306.h"
#include "Oil_Level_Graphic/Image_10_SSD1306.h"
#include "Oil_Level_Graphic/Image_20_SSD1306.h"
#include "Oil_Level_Graphic/Image_30_SSD1306.h"
#include "Oil_Level_Graphic/Image_40_SSD1306.h"
#include "Oil_Level_Graphic/Image_50_SSD1306.h"
#include "Oil_Level_Graphic/Image_60_SSD1306.h"
#include "Oil_Level_Graphic/Image_70_SSD1306.h"
#include "Oil_Level_Graphic/Image_80_SSD1306.h"
#include "Oil_Level_Graphic/Image_90_SSD1306.h"
#include "Oil_Level_Graphic/Image_100_SSD1306.h"
#include "oilsensorled.h"

Preferences preferences;
BluetoothSerial SerialBT;
bool Impuls_1_High                      = false;
bool Impuls_1_Low                       = false;
bool Impuls_2_High                      = false;
bool Impuls_2_Low                       = false;
bool debugFlag                          = false;
bool NewOilSensorEquipped               = false;
bool toggleflag                         = false;
uint8_t signalinput                     = 0;
uint16_t cnt                            = 0;
uint8_t oilTemperature                  = OilTemperaturePercentageInitValue; /*init value 254*/
uint8_t oilLevelPercentage              = OilLevelPercentageInitValue; /*init value 254*/
uint8_t testValue_oilTemperature        = 85; /* Debugvalue 255 */
uint8_t testValue_oilLevelPercentage    = 60; /* Debugvalue 255 */

uint8_t sequenceCounter                 = 0;
uint8_t startingsequence[]              = {0,50,100,50,0};
hw_timer_t *timer                       = NULL;
static uint16_t                         cntRawArr[4];
static uint16_t                         returnArray[4];
bool TempToggle                         = false;
bool TimeoutSensorDetected              = true;
char SoftwareVersion[]                  = "X003";       
portMUX_TYPE timerMux                   = portMUX_INITIALIZER_UNLOCKED;
char  Modulename[]                      = "OilSensor";
bool NewData                            = false;
bool statusOfExtraOutputPin             = false;
bool toggleInvertDisplayFlag            = false;

uint16_t OldOilTempCompValues[]         = {Old_sensor_Temperature_30,Old_sensor_Temperature_40,Old_sensor_Temperature_50,Old_sensor_Temperature_55,Old_sensor_Temperature_60,Old_sensor_Temperature_65,Old_sensor_Temperature_70,Old_sensor_Temperature_75,Old_sensor_Temperature_80,Old_sensor_Temperature_85,Old_sensor_Temperature_90,Old_sensor_Temperature_95,Old_sensor_Temperature_100,Old_sensor_Temperature_105,Old_sensor_Temperature_110,Old_sensor_Temperature_115};
uint16_t OldOilLevelCompValues[]        = {Old_sensor_OilLevelEmpty,Old_sensor_OilLevel_10,Old_sensor_OilLevel_20,Old_sensor_OilLevel_30,Old_sensor_OilLevel_40,Old_sensor_OilLevel_50,Old_sensor_OilLevel_60,Old_sensor_OilLevel_70,Old_sensor_OilLevel_80,Old_sensor_OilLevel_90,Old_sensor_OilLevelFull};

uint16_t NewOilTempCompValues[]         = {New_sensor_Temperature_30,New_sensor_Temperature_40,New_sensor_Temperature_50,New_sensor_Temperature_55,New_sensor_Temperature_60,New_sensor_Temperature_65,New_sensor_Temperature_70,New_sensor_Temperature_75,New_sensor_Temperature_80,New_sensor_Temperature_85,New_sensor_Temperature_90,New_sensor_Temperature_95,New_sensor_Temperature_100,New_sensor_Temperature_105,New_sensor_Temperature_110,New_sensor_Temperature_115};
uint16_t NewOilLevelCompValues[]        = {New_sensor_OilLevelEmpty,New_sensor_OilLevel_10,New_sensor_OilLevel_20,New_sensor_OilLevel_30,New_sensor_OilLevel_40,New_sensor_OilLevel_50,New_sensor_OilLevel_60,New_sensor_OilLevel_70,New_sensor_OilLevel_80,New_sensor_OilLevel_90,New_sensor_OilLevelFull};


#define EEPROMNameSpace               "my_variables"
#define SignalInputPin                2
#define ISRDebugTogglePin             4

#define OutputPin                     14
#define timerfrequency                1000000
#define ISRfrequency                  1000
#define TimeoutSignalMS               1500
#define Buffersize                    25
#define SCREEN_WIDTH                  128
#define SCREEN_HEIGHT                 64

uint8_t BT_rx_buffer[Buffersize];

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

void ARDUINO_ISR_ATTR onTimer() {
  /* This toggle is for debug purpose only.. */
  /* You could verify at Pin ISRDebugTogglePin how often the ISR is called. */
  if (toggleflag == false) {
    digitalWrite(ISRDebugTogglePin, LOW);
    toggleflag = true;
  } else {
    digitalWrite(ISRDebugTogglePin, HIGH);
    toggleflag = false;
  }
  signalinput = digitalRead(SignalInputPin);


  if ((signalinput == 0x01) && (Impuls_1_High == false) && (Impuls_1_Low == false) && (Impuls_2_High == false) && (Impuls_2_Low == false)) {
    /*  first high signal*/
    /* T1 */
    Impuls_1_High = true;
    TimeoutSensorDetected = false;
  } else if ((signalinput == 0x00) && (Impuls_1_High == true) && (Impuls_1_Low == false) && (Impuls_2_High == false) && (Impuls_2_Low == false)) {
    /*  first low signal*/
    /* T2 */
    Impuls_1_Low = true;
    portENTER_CRITICAL(&timerMux);
    cntRawArr[0] = cnt;
    portEXIT_CRITICAL(&timerMux);

  } else if ((signalinput == 0x01) && (Impuls_1_High == true) && (Impuls_1_Low == true) && (Impuls_2_High == false) && (Impuls_2_Low == false)) {
    /* 2nd high signal*/
    /* T3 */
    Impuls_2_High = true;
    portENTER_CRITICAL(&timerMux);
    cntRawArr[1] = cnt;
    portEXIT_CRITICAL(&timerMux);

  } else if ((signalinput == 0x00) && (Impuls_1_High == true) && (Impuls_1_Low == true) && (Impuls_2_High == true) && (Impuls_2_Low == false)) {
    /* T4 */
    Impuls_2_Low = true;
    portENTER_CRITICAL(&timerMux);
    cntRawArr[2] = cnt;
    portEXIT_CRITICAL(&timerMux);

  } else if ((signalinput == 0x01) && (Impuls_1_High == true) && (Impuls_1_Low == true) && (Impuls_2_High == true) && (Impuls_2_Low == true)) {
    /* T5 */
    Impuls_1_High = false;
    Impuls_2_High = false;
    Impuls_1_Low  = false;
    Impuls_2_Low  = false;
    portENTER_CRITICAL(&timerMux);
    cntRawArr[3] = cnt;
    portEXIT_CRITICAL(&timerMux);
    cnt = 1;
  }

  if (Impuls_1_High == true) {
    cnt = cnt + 1;
  }


  /* if sensor is disconnected -> cnt is higher than TimeoutSignalMS 1500->set to 0xFE*/
  if(cnt>TimeoutSignalMS)
  { 
    cntRawArr[0] = OilLevelPercentageErrorValue;
    cntRawArr[1] = OilLevelPercentageErrorValue;
    cntRawArr[2] = OilLevelPercentageErrorValue;
    cntRawArr[3] = OilLevelPercentageErrorValue;
    TimeoutSensorDetected = true;
    cnt = 1;
  }
}


void  orderImpulse(uint16_t inputArr[]) {
  /* This Method orders the measured impulses into the correct sequence..*/
  /* There might be the case that the Uc starts at the wrong Time and interprets T4 (see at ine 71) as the start of the sequence ..*/
  /* This method returns a sequence of Four measured where the beginning of the Signal is always stored in returnArray[0] */
  /* SerialBT.print(testval >> 8);         -> 0x01 */
  /* SerialBT.print(testval & 0xFF);       -> 0xF4 */
 
  uint16_t tempArray[4];
  tempArray[0] = inputArr[0];
  tempArray[1] = inputArr[1] - inputArr[0];
  tempArray[2] = inputArr[2] - inputArr[1];
  tempArray[3] = inputArr[3] - inputArr[2];

    /*detect Big LOW T5*/
    if (tempArray[0] > 100) {
        returnArray[0] = tempArray[1]; /* 1) 20ms  High   T2 */
        returnArray[1] = tempArray[2]; /* 2) 20ms  Low    T3 */
        returnArray[2] = tempArray[3]; /* 3) 20ms  High   T4 */
        returnArray[3] = tempArray[0]; /* 4) 160ms BigLow T5 */
    }
    if (tempArray[1] > 100) {
        returnArray[0] = tempArray[2]; /* 1) 20ms  High   T2 */
        returnArray[1] = tempArray[3]; /* 2) 20ms  Low    T3 */
        returnArray[2] = tempArray[0]; /* 3) 20ms  High   T4 */
        returnArray[3] = tempArray[1]; /* 4) 160ms BigLow T5 */
    }
    if (tempArray[2] > 100) {
        returnArray[0] = tempArray[3]; /* 1) 20ms  High    T2 */
        returnArray[1] = tempArray[0]; /* 2) 20ms  Low     T3 */
        returnArray[2] = tempArray[1]; /* 3) 20ms  High    T4 */
        returnArray[3] = tempArray[2]; /* 4) 160ms BigLow  T5 */
    }
    if (tempArray[3] > 100) {
        /* Best case only copy */
        returnArray[0] = tempArray[0]; /* 1) 20ms  High    T2 */
        returnArray[1] = tempArray[1]; /* 2) 20ms  Low     T3 */
        returnArray[2] = tempArray[2]; /* 3) 20ms  High    T4 */
        returnArray[3] = tempArray[3]; /* 4) 160ms BigLow  T5 */
    }
}


void delete_BT_buffer()
{
  for(int z=0;z<Buffersize;z++)
  {
    BT_rx_buffer[z]=0xFF;  
  }
}



void sendInfosToBT(uint8_t temperature, uint8_t OilLevel,uint16_t outputarr[]) {
  if (debugFlag == true)
  {
    uint8_t i;
    //Serial.write(0xFF);
    for (i = 0; i < 4; i++) 
    {
      if (i == 0) {
        SerialBT.print("T2:");
      }
      if (i == 1) {
        SerialBT.print("T3:");
      }
      if (i == 2) {
        SerialBT.print("T4:");
      }
      if (i == 3) {
        SerialBT.print("T5:");
      }
      SerialBT.println(outputarr[i]);
    }
    SerialBT.println("");
  
    SerialBT.print("Softwareversion");
    String stri = SoftwareVersion;
    SerialBT.println(stri);
    SerialBT.print("Sensor TO detected: ");
    SerialBT.println(TimeoutSensorDetected);
    SerialBT.print("SequenceCounter");
    SerialBT.println(sequenceCounter);
  }
  SerialBT.print("Temperature: ");
  SerialBT.println(temperature);
  SerialBT.print("OilLevel: ");
  SerialBT.println(OilLevel);
}


void analyse_BT_Protocol(uint8_t receive_BT_Array[])
{   
  if(NewData == true){
    /*Command to read something*/
    if (receive_BT_Array[0]==UDS_READ_DATA_BY_IDENTIFIER)
    {
	    uint8_t posResponse = (UDS_READ_DATA_BY_IDENTIFIER + 0x40);
      /* 0x22 0xF1 0xAB */
      /* Get SW Version*/
      if((receive_BT_Array[1]==0xF1) && (receive_BT_Array[2]==0xAB))
      {
        SerialBT.print(posResponse);
        SerialBT.print(0xF1);
        SerialBT.print(0xAB); 
        SerialBT.print(SoftwareVersion[0]);
        SerialBT.print(SoftwareVersion[1]);
        SerialBT.print(SoftwareVersion[2]);
        SerialBT.print(SoftwareVersion[3]);
      }else

      /* 0x22 0xF1 0x87 */
      /* Get Debug Flag */
      if((receive_BT_Array[1]==0xF1) && (receive_BT_Array[2]==0x87))
      {
        SerialBT.print(posResponse);
        SerialBT.print(0xF1);
        SerialBT.print(0x87); 
        SerialBT.print(debugFlag);
      }else

      /* 0x22 0xF1 0x05 */
      /* get Modulename */
      if((receive_BT_Array[1]==0xF1) && (receive_BT_Array[2]==0x05))
      {
        SerialBT.print(posResponse);
        SerialBT.print(0xF1);
        SerialBT.print(0x05);
        for(int i=0;i<sizeof(Modulename);i++)
        {
            SerialBT.print(Modulename[i]);
        }
        
      }else
      
      /* 0x22 0x06 0x00 */
      /* This part should get the Debugvalue for the OilTemperature */
      if((receive_BT_Array[1]==0x06) && (receive_BT_Array[2]==0x00))
      {
        SerialBT.print(posResponse);
        SerialBT.print(0x06);
        SerialBT.print(0x00);                  
        SerialBT.print(testValue_oilTemperature);
        
      }else

      /* 0x22 0x06 0x01 */
       /* This part should get the Debugvalue for the OilLevel */
      if((receive_BT_Array[1]==0x06) && (receive_BT_Array[2]==0x01))
      {
        SerialBT.print(posResponse);
        SerialBT.print(0x06);
        SerialBT.print(0x01);                  
        SerialBT.print(testValue_oilLevelPercentage);
        
      }else

      /* 0x22 0x06 0x02 */
      /* get ExtraOutputPin flag*/
      if((receive_BT_Array[1]==0x06) && (receive_BT_Array[2]==0x02))
      {
        SerialBT.print(posResponse);
        SerialBT.print(0x06);
        SerialBT.print(0x02);                  
        SerialBT.print(statusOfExtraOutputPin);
        
      }else


      /* 0x22 0x06 0x03 */
      /* get OldSensorNewSensor flag*/
      if((receive_BT_Array[1]==0x06) && (receive_BT_Array[2]==0x03))
      {
        SerialBT.print(posResponse);
        SerialBT.print(0x06);
        SerialBT.print(0x03);                  
        SerialBT.print(NewOilSensorEquipped);
        
      }else
      
      
      /* 0x22 0x06 0x04 */
      /* get oilTemperature */
      if((receive_BT_Array[1]==0x06) && (receive_BT_Array[2]==0x04))
      {
        SerialBT.print(posResponse);
        SerialBT.print(0x06);
        SerialBT.print(0x04);                  
        SerialBT.print(oilTemperature);
        
      }else
      
      /* 0x22 0x06 0x05 */
      /* get oilLevelPercentage */
      if((receive_BT_Array[1]==0x06) && (receive_BT_Array[2]==0x05))
      {
        SerialBT.print(posResponse);
        SerialBT.print(0x06);
        SerialBT.print(0x05);                  
        SerialBT.print(oilLevelPercentage);
        
      }else
      
      /* 0x22 0x06 0x06 */
      /* get oil temperature compare vlaues */
      if((receive_BT_Array[1]==0x06) && (receive_BT_Array[2]==0x06))
      {
        SerialBT.print(posResponse);
        SerialBT.print(0x06);
        SerialBT.print(0x06); 
        uint8_t i = 0;
        uint8_t tempvar;
        for(i=0;i<sizeof(OldOilTempCompValues);i++)
        {
          tempvar = 
            /* As example testval =         500 == 0x01F4    */
            /* SerialBT.print(testval >> 8);         -> 0x01 */
            /* SerialBT.print(testval & 0xFF);       -> 0xF4 */
          tempvar = (OldOilTempCompValues[i] >> 8) ;
          SerialBT.print(tempvar);
          tempvar = (OldOilTempCompValues[i] & 0xFF);
          SerialBT.print(tempvar);
        }                 
        
        
      }else
      
      /* 0x22 0x06 0x07 */
      /* get oilLevel compare values */
      if((receive_BT_Array[1]==0x06) && (receive_BT_Array[2]==0x07))
      {
        SerialBT.print(posResponse);
        SerialBT.print(0x06);
        SerialBT.print(0x07);                  
        uint8_t i = 0;
        uint8_t tempvar;
        for(i=0;i<sizeof(OldOilLevelCompValues);i++)
        {
          tempvar = 
            /* As example testval =         500 == 0x01F4    */
            /* SerialBT.print(testval >> 8);         -> 0x01 */
            /* SerialBT.print(testval & 0xFF);       -> 0xF4 */
          tempvar = (OldOilLevelCompValues[i] >> 8) ;
          SerialBT.print(tempvar);
          tempvar = (OldOilLevelCompValues[i] & 0xFF);
          SerialBT.print(tempvar);
        } 
        
      }else
      /* 0x22 0x06 0x08 */
      /* get oitemp new sensor */
      if((receive_BT_Array[1]==0x06) && (receive_BT_Array[2]==0x08))
      {
        SerialBT.print(posResponse);
        SerialBT.print(0x06);
        SerialBT.print(0x08);                  
        uint8_t i = 0;
        uint8_t tempvar;
        for(i=0;i<sizeof(NewOilTempCompValues);i++)
        {
          tempvar = 
            /* As example testval =         500 == 0x01F4    */
            /* SerialBT.print(testval >> 8);         -> 0x01 */
            /* SerialBT.print(testval & 0xFF);       -> 0xF4 */
          tempvar = (NewOilTempCompValues[i] >> 8) ;
          SerialBT.print(tempvar);
          tempvar = (NewOilTempCompValues[i] & 0xFF);
          SerialBT.print(tempvar);
        } 
        
      }else

      /* 0x22 0x06 0x09 */
      /* get oitemp new sensor */
      if((receive_BT_Array[1]==0x06) && (receive_BT_Array[2]==0x09))
      {
        SerialBT.print(posResponse);
        SerialBT.print(0x06);
        SerialBT.print(0x09);                  
        uint8_t i = 0;
        uint8_t tempvar;
        for(i=0;i<sizeof(NewOilLevelCompValues);i++)
        {
          tempvar = 
            /* As example testval =         500 == 0x01F4    */
            /* SerialBT.print(testval >> 8);         -> 0x01 */
            /* SerialBT.print(testval & 0xFF);       -> 0xF4 */
          tempvar = (NewOilLevelCompValues[i] >> 8) ;
          SerialBT.print(tempvar);
          tempvar = (NewOilLevelCompValues[i] & 0xFF);
          SerialBT.print(tempvar);
        } 
        
      }else
      {
        SerialBT.print(0x7F);
        SerialBT.print(UDS_READ_DATA_BY_IDENTIFIER);
        SerialBT.print(UDS_NRC_requestOutOfRange);
      }
    
    
    
    
    
    
    }else

    /*Command to write something*/
    if(receive_BT_Array[0] == UDS_WRITE_DATA_BY_IDENTIFIER)
    {
      uint8_t posResponse = (UDS_WRITE_DATA_BY_IDENTIFIER + 0x40);

      /* This part should change the SW Version*/
      /* 0x2E 0xF1 0xAB 0x04 0x!!  0x!! 0x!! 0x!! */
      if((receive_BT_Array[1]==0xF1) && (receive_BT_Array[2]==0xAB))
      {
        if(receive_BT_Array[3]==0x04)
        {
          char Temparr[5]; 
          Temparr[0] =  (char)receive_BT_Array[4];
          Temparr[1] =  (char)receive_BT_Array[5];
          Temparr[2] =  (char)receive_BT_Array[6];
          Temparr[3] =  (char)receive_BT_Array[7];
          Temparr[4]= 0x00;
          SoftwareVersion[0] = (char) Temparr[0];
          SoftwareVersion[1] = (char) Temparr[1];
          SoftwareVersion[2] = (char) Temparr[2];
          SoftwareVersion[3] = (char) Temparr[3];
        
        
          preferences.begin(EEPROMNameSpace, false); 
          preferences.putString("SW_Version",Temparr);
          preferences.end();  
          
          SerialBT.print(posResponse);
          SerialBT.print(0xF1);
          SerialBT.print(0xAB);
          //ESP.restart(); 
        }else{
          SerialBT.print(0x7F);
          SerialBT.print(UDS_WRITE_DATA_BY_IDENTIFIER);
          SerialBT.print(UDS_NRC_incorrectMessageLengthOrInvalidFormat);
        }
      }else


      /* This part should change the name of the BT Module*/
      /* 0x2E 0xF1 0x05 0x?? 0x!!  0x!! 0x!! 0x!!......*/
      if((receive_BT_Array[1]==0xF1) && (receive_BT_Array[2]==0x05))
      {
        uint8_t length_of_name = receive_BT_Array[3];
        if((0<length_of_name) && (length_of_name<=15))
        {

          char Temparr[length_of_name+1];
          uint8_t i; 
          for (i=0;i<length_of_name;i++)
          {
            Temparr[i] =  (char) receive_BT_Array[4+i];
          }
          Temparr[length_of_name]= 0x00;

          for (i=0;i<length_of_name;i++)
          {
            Modulename[i] =  (char) Temparr[i];
          }
          
    
          preferences.begin(EEPROMNameSpace, false); 
          preferences.putString("Modulename",Temparr);
          preferences.end();

          SerialBT.begin(Modulename);

          SerialBT.print(posResponse);
          SerialBT.print(0xF1);
          SerialBT.print(0x05); 
        }else{
          SerialBT.print(0x7F);
          SerialBT.print(UDS_WRITE_DATA_BY_IDENTIFIER);
          SerialBT.print(UDS_NRC_incorrectMessageLengthOrInvalidFormat);
        }
      }else

      /* This part should set the Debugvalue for the OilTemperature
      /* 0x2E 0x06 0x00 0x??......*/
      if((receive_BT_Array[1]==0x06) && (receive_BT_Array[2]==0x00))
      {
        testValue_oilTemperature = receive_BT_Array[3];
        SerialBT.print(posResponse);
        SerialBT.print(0x06);
        SerialBT.print(0x00); 
      }else

      /* This part should set the Debugvalue for the OilLevel
      /* 0x2E 0x06 0x01 0x!! 0x!!  0x!! 0x!! 0x!!......*/
      if((receive_BT_Array[1]==0x06) && (receive_BT_Array[2]==0x01))
      {
        testValue_oilLevelPercentage = receive_BT_Array[3];
        SerialBT.print(posResponse);
        SerialBT.print(0x06);
        SerialBT.print(0x01); 
      }else

      /* This part should set the value of an Output Pin
      /* 0x2E 0x06 0x02 0x!! 0x!!  0x!! 0x!! 0x!!......*/
      if((receive_BT_Array[1]==0x06) && (receive_BT_Array[2]==0x02))
      {
        statusOfExtraOutputPin = receive_BT_Array[3];
        digitalWrite(OutputPin, statusOfExtraOutputPin);
        SerialBT.print(posResponse);
        SerialBT.print(0x06);
        SerialBT.print(0x02); 
      }else

      /* This part should set the old sensor new sensor flag
      /* 0x2E 0x06 0x03 0x!! 0x!!  0x!! 0x!! 0x!!......*/
      if((receive_BT_Array[1]==0x06) && (receive_BT_Array[2]==0x03))
      {

        bool temp = (bool)receive_BT_Array[3];
        if(NewOilSensorEquipped != temp)
        {
          NewOilSensorEquipped = temp;
          preferences.begin(EEPROMNameSpace, false); 
          preferences.putBool("NewSensorflag",temp);
          preferences.end();
        }

        SerialBT.print(posResponse);
        SerialBT.print(0x06);
        SerialBT.print(0x03); 
      }else

      /* This part should set the compare values for LevTemperature
      /* 0x2E 0x06 0x06 0x!! 0x!!  0x!! 0x!! 0x!!......*/
      if((receive_BT_Array[1]==0x06) && (receive_BT_Array[2]==0x06))
      {

        preferences.begin(EEPROMNameSpace, false); 
        uint16_t temp = ((uint16_t) receive_BT_Array[3]<<8)|receive_BT_Array[4];
        preferences.putShort("Old_sensor_Temperature_30",temp);
        temp = ((uint16_t) receive_BT_Array[5]<<8)|receive_BT_Array[6];
        preferences.putShort("Old_sensor_Temperature_40",temp);
        temp = ((uint16_t) receive_BT_Array[7]<<8)|receive_BT_Array[8];
        preferences.putShort("Old_sensor_Temperature_50",temp);
        temp = ((uint16_t) receive_BT_Array[9]<<8)|receive_BT_Array[10];
        preferences.putShort("Old_sensor_Temperature_55",temp);
        temp = ((uint16_t) receive_BT_Array[11]<<8)|receive_BT_Array[12];
        preferences.putShort("Old_sensor_Temperature_60",temp);
        temp = ((uint16_t) receive_BT_Array[13]<<8)|receive_BT_Array[14];
        preferences.putShort("Old_sensor_Temperature_65",temp);
        temp = ((uint16_t) receive_BT_Array[15]<<8)|receive_BT_Array[16];
        preferences.putShort("Old_sensor_Temperature_70",temp);
        temp = ((uint16_t) receive_BT_Array[17]<<8)|receive_BT_Array[18];
        preferences.putShort("Old_sensor_Temperature_75",temp);
        temp = ((uint16_t) receive_BT_Array[19]<<8)|receive_BT_Array[20];
        preferences.putShort("Old_sensor_Temperature_80",temp);
        temp = ((uint16_t) receive_BT_Array[21]<<8)|receive_BT_Array[22];
        preferences.putShort("Old_sensor_Temperature_85",temp);
        temp = ((uint16_t) receive_BT_Array[23]<<8)|receive_BT_Array[24];
        preferences.putShort("Old_sensor_Temperature_90",temp);
        temp = ((uint16_t) receive_BT_Array[25]<<8)|receive_BT_Array[26];
        preferences.putShort("Old_sensor_Temperature_95",temp);
        temp = ((uint16_t) receive_BT_Array[27]<<8)|receive_BT_Array[28];
        preferences.putShort("Old_sensor_Temperature_100",temp);
        temp = ((uint16_t) receive_BT_Array[29]<<8)|receive_BT_Array[30];
        preferences.putShort("Old_sensor_Temperature_105",temp);
        temp = ((uint16_t) receive_BT_Array[31]<<8)|receive_BT_Array[32];
        preferences.putShort("Old_sensor_Temperature_110",temp);
        temp = ((uint16_t) receive_BT_Array[33]<<8)|receive_BT_Array[34];
        preferences.putShort("Old_sensor_Temperature_115",temp);

        preferences.end();
        

        SerialBT.print(posResponse);
        SerialBT.print(0x06);
        SerialBT.print(0x06); 
      }else
       /* This part should set the compare values for Level old sensor
      /* 0x2E 0x06 0x07 0x!! 0x!!  0x!! 0x!! 0x!!......*/
      if((receive_BT_Array[1]==0x06) && (receive_BT_Array[2]==0x07))
      {

        preferences.begin(EEPROMNameSpace, false); 
        uint16_t temp = ((uint16_t) receive_BT_Array[3]<<8)|receive_BT_Array[4];
        preferences.putShort("Old_sensor_OilLevelEmpty",temp);
        temp = ((uint16_t) receive_BT_Array[5]<<8)|receive_BT_Array[6];
        preferences.putShort("Old_sensor_OilLevel_10",temp);
        temp = ((uint16_t) receive_BT_Array[7]<<8)|receive_BT_Array[8];
        preferences.putShort("Old_sensor_OilLevel_20",temp);
        temp = ((uint16_t) receive_BT_Array[9]<<8)|receive_BT_Array[10];
        preferences.putShort("Old_sensor_OilLevel_30",temp);
        temp = ((uint16_t) receive_BT_Array[11]<<8)|receive_BT_Array[12];
        preferences.putShort("Old_sensor_OilLevel_40",temp);
        temp = ((uint16_t) receive_BT_Array[13]<<8)|receive_BT_Array[14];
        preferences.putShort("Old_sensor_OilLevel_50",temp);
        temp = ((uint16_t) receive_BT_Array[15]<<8)|receive_BT_Array[16];
        preferences.putShort("Old_sensor_OilLevel_60",temp);
        temp = ((uint16_t) receive_BT_Array[17]<<8)|receive_BT_Array[18];
        preferences.putShort("Old_sensor_OilLevel_70",temp);
        temp = ((uint16_t) receive_BT_Array[19]<<8)|receive_BT_Array[20];
        preferences.putShort("Old_sensor_OilLevel_80",temp);
        temp = ((uint16_t) receive_BT_Array[21]<<8)|receive_BT_Array[22];
        preferences.putShort("Old_sensor_OilLevel_90",temp);
        temp = ((uint16_t) receive_BT_Array[23]<<8)|receive_BT_Array[24];
        preferences.putShort("Old_sensor_OilLevelFull",temp);

        preferences.end();

        SerialBT.print(posResponse);
        SerialBT.print(0x06);
        SerialBT.print(0x07); 
      }else

      /* This part should set the compare values for LevTemperature
      /* 0x2E 0x06 0x08 0x!! 0x!!  0x!! 0x!! 0x!!......*/
      if((receive_BT_Array[1]==0x06) && (receive_BT_Array[2]==0x08))
      {

        preferences.begin(EEPROMNameSpace, false); 
        uint16_t temp = ((uint16_t) receive_BT_Array[3]<<8)|receive_BT_Array[4];
        preferences.putShort("New_sensor_Temperature_30",temp);
        temp = ((uint16_t) receive_BT_Array[5]<<8)|receive_BT_Array[6];
        preferences.putShort("New_sensor_Temperature_40",temp);
        temp = ((uint16_t) receive_BT_Array[7]<<8)|receive_BT_Array[8];
        preferences.putShort("New_sensor_Temperature_50",temp);
        temp = ((uint16_t) receive_BT_Array[9]<<8)|receive_BT_Array[10];
        preferences.putShort("New_sensor_Temperature_55",temp);
        temp = ((uint16_t) receive_BT_Array[11]<<8)|receive_BT_Array[12];
        preferences.putShort("New_sensor_Temperature_60",temp);
        temp = ((uint16_t) receive_BT_Array[13]<<8)|receive_BT_Array[14];
        preferences.putShort("New_sensor_Temperature_65",temp);
        temp = ((uint16_t) receive_BT_Array[15]<<8)|receive_BT_Array[16];
        preferences.putShort("New_sensor_Temperature_70",temp);
        temp = ((uint16_t) receive_BT_Array[17]<<8)|receive_BT_Array[18];
        preferences.putShort("New_sensor_Temperature_75",temp);
        temp = ((uint16_t) receive_BT_Array[19]<<8)|receive_BT_Array[20];
        preferences.putShort("New_sensor_Temperature_80",temp);
        temp = ((uint16_t) receive_BT_Array[21]<<8)|receive_BT_Array[22];
        preferences.putShort("New_sensor_Temperature_85",temp);
        temp = ((uint16_t) receive_BT_Array[23]<<8)|receive_BT_Array[24];
        preferences.putShort("New_sensor_Temperature_90",temp);
        temp = ((uint16_t) receive_BT_Array[25]<<8)|receive_BT_Array[26];
        preferences.putShort("New_sensor_Temperature_95",temp);
        temp = ((uint16_t) receive_BT_Array[27]<<8)|receive_BT_Array[28];
        preferences.putShort("New_sensor_Temperature_100",temp);
        temp = ((uint16_t) receive_BT_Array[29]<<8)|receive_BT_Array[30];
        preferences.putShort("New_sensor_Temperature_105",temp);
        temp = ((uint16_t) receive_BT_Array[31]<<8)|receive_BT_Array[32];
        preferences.putShort("New_sensor_Temperature_110",temp);
        temp = ((uint16_t) receive_BT_Array[33]<<8)|receive_BT_Array[34];
        preferences.putShort("New_sensor_Temperature_115",temp);

        preferences.end();
        

        SerialBT.print(posResponse);
        SerialBT.print(0x06);
        SerialBT.print(0x08); 
      }else

      /* This part should set the compare values for Level new Sensor
      /* 0x2E 0x06 0x09 0x!! 0x!!  0x!! 0x!! 0x!!......*/
      if((receive_BT_Array[1]==0x06) && (receive_BT_Array[2]==0x09))
      {

        preferences.begin(EEPROMNameSpace, false); 
        uint16_t temp = ((uint16_t) receive_BT_Array[3]<<8)|receive_BT_Array[4];
        preferences.putShort("New_sensor_OilLevelEmpty",temp);
        temp = ((uint16_t) receive_BT_Array[5]<<8)|receive_BT_Array[6];
        preferences.putShort("New_sensor_OilLevel_10",temp);
        temp = ((uint16_t) receive_BT_Array[7]<<8)|receive_BT_Array[8];
        preferences.putShort("New_sensor_OilLevel_20",temp);
        temp = ((uint16_t) receive_BT_Array[9]<<8)|receive_BT_Array[10];
        preferences.putShort("New_sensor_OilLevel_30",temp);
        temp = ((uint16_t) receive_BT_Array[11]<<8)|receive_BT_Array[12];
        preferences.putShort("New_sensor_OilLevel_40",temp);
        temp = ((uint16_t) receive_BT_Array[13]<<8)|receive_BT_Array[14];
        preferences.putShort("New_sensor_OilLevel_50",temp);
        temp = ((uint16_t) receive_BT_Array[15]<<8)|receive_BT_Array[16];
        preferences.putShort("New_sensor_OilLevel_60",temp);
        temp = ((uint16_t) receive_BT_Array[17]<<8)|receive_BT_Array[18];
        preferences.putShort("New_sensor_OilLevel_70",temp);
        temp = ((uint16_t) receive_BT_Array[19]<<8)|receive_BT_Array[20];
        preferences.putShort("New_sensor_OilLevel_80",temp);
        temp = ((uint16_t) receive_BT_Array[21]<<8)|receive_BT_Array[22];
        preferences.putShort("New_sensor_OilLevel_90",temp);
        temp = ((uint16_t) receive_BT_Array[23]<<8)|receive_BT_Array[24];
        preferences.putShort("New_sensor_OilLevelFull",temp);
        preferences.end();
        
        SerialBT.print(posResponse);
        SerialBT.print(0x06);
        SerialBT.print(0x09); 
      }else
      
      {
        SerialBT.print(0x7F);
        SerialBT.print(UDS_WRITE_DATA_BY_IDENTIFIER);
        SerialBT.print(UDS_NRC_requestOutOfRange);
      }
    }else

    /* Session Control */
    if(receive_BT_Array[0] == UDS_Session_Control)
    {
      if(receive_BT_Array[1] == UDS_Session_Control_Default_Session)
      {
          
          if(debugFlag != false)
          {
            debugFlag = false;
            preferences.begin(EEPROMNameSpace, false); 
            preferences.putBool("debugflag",false);
            preferences.end();
          }
          
      } else if(receive_BT_Array[1] == UDS_Session_Control_Extended_Session)
      {
          if(debugFlag != true)
          {
            debugFlag = true;
            preferences.begin(EEPROMNameSpace, false); 
            preferences.putBool("debugflag",true);
            preferences.end();
          }
      }else{
        SerialBT.print(0x7F);
        SerialBT.print(UDS_Session_Control);
        SerialBT.print(UDS_NRC_subFunctionNotSupported);
      }
    }else{
        SerialBT.print(0x7F);
        SerialBT.print(receive_BT_Array[0]);
        SerialBT.print(UDS_NRC_serviceNotSupported);
    }
    delete_BT_buffer();
  }
  NewData = false;
}

void getBTData(const uint8_t *buffer, size_t size)
{
  uint8_t i=0;
  for(i=0;i<size;i++){
    BT_rx_buffer[i] = buffer[i];
  }
  NewData = true;
}

void showOilLevelAtDisplay(uint8_t percentageOillevel,bool initflag)
{
  // Clear the buffer.
  display.clearDisplay();
  if(TimeoutSensorDetected == false)
  {  
      //if(percentageOillevel == 00){display.drawBitmap(0, 0, image_OilLevel_00, 128, 64, 1);} else
      //if(percentageOillevel == 10){ display.drawBitmap(0, 0, image_OilLevel_10, 128, 64, 1);} else
    /* Oillevel Ok */
    if(percentageOillevel >= 20){
      /* Design Reason Background is Dark all Text is White*/
      display.invertDisplay(true);  
      if(percentageOillevel == 20){ display.drawBitmap(0, 0, image_OilLevel_20, 128, 64, 1);} else
      if(percentageOillevel == 30){ display.drawBitmap(0, 0, image_OilLevel_30, 128, 64, 1);} else
      if(percentageOillevel == 40){ display.drawBitmap(0, 0, image_OilLevel_40, 128, 64, 1);} else
      if(percentageOillevel == 50){ display.drawBitmap(0, 0, image_OilLevel_50, 128, 64, 1);} else
      if(percentageOillevel == 60){ display.drawBitmap(0, 0, image_OilLevel_60, 128, 64, 1);} else
      if(percentageOillevel == 70){display.drawBitmap(0, 0, image_OilLevel_70, 128, 64, 1);} else
      if(percentageOillevel == 80){display.drawBitmap(0, 0, image_OilLevel_80, 128, 64, 1);} else
      if(percentageOillevel == 90){display.drawBitmap(0, 0, image_OilLevel_90, 128, 64, 1);} else
      if(percentageOillevel == 100){display.drawBitmap(0, 0, image_OilLevel_100, 128, 64, 1);} 

      display.setTextColor(BLACK);
      display.setTextSize(1);
    
      display.setCursor(24, 6);
      display.print("MAX");

      display.setCursor(24, 53);
      display.print("MIN");

      display.setTextSize(3);
      display.setCursor(37, 22);
      //display.print(char(248));
  
      display.print(oilTemperature);

      display.display();
    }else{
      
      if(initflag==false)
      {
         /* Oillevel not Ok */
        if(percentageOillevel == 00){ display.drawBitmap(0, 0, image_OilLevel_00, 128, 64, 1);} else
        if(percentageOillevel == 10){ display.drawBitmap(0, 0, image_OilLevel_10, 128, 64, 1);} 
        display.setTextColor(BLACK);
        display.setTextSize(1);
      
        display.setCursor(24, 6);
        display.print("MAX");

        display.setCursor(24, 53);
        display.print("MIN");

        display.setTextSize(2);
        /* Position of Text "Check" measured from the Top left corner (0,0) in Pixel */
        display.setCursor(53, 13);
        display.print("CHECK");
        display.setCursor(63, 31);
        display.print("OIL");
        if(toggleInvertDisplayFlag== false)
        {
          toggleInvertDisplayFlag = true;
          display.invertDisplay(true);
        }else{
          toggleInvertDisplayFlag = false;
          display.invertDisplay(false);
        }
        display.display();
      }
    }
  }else{
        /* if Sensor TO detected */
        display.setTextColor(WHITE);

        display.setTextSize(2);
        /* Position of Text "Check" measured from the Top left corner (0,0) in Pixel */
        display.setCursor(30, 12);
        display.print("Sensor");
        display.setCursor(50, 35);
        display.print("OL");
        
        if(toggleInvertDisplayFlag == false)
        {
          toggleInvertDisplayFlag = true;
          display.invertDisplay(true);
        }else{
          toggleInvertDisplayFlag = false;
          display.invertDisplay(false);
        }
        display.display();
  }
}

void controlOfDisplay()
{
  if ((sequenceCounter>=8) && (sequenceCounter<=15) )
  {
    showOilLevelAtDisplay(oilLevelPercentage,false);
  } 
  /* short initialization sequence*/
  if((sequenceCounter>=3) && (sequenceCounter<=7))
  {
    showOilLevelAtDisplay(startingsequence[sequenceCounter-3],true);
    sequenceCounter = sequenceCounter+1;
  }
  /* perform a short Delay*/
  if((sequenceCounter<3))
  {
    sequenceCounter = sequenceCounter+1;
  }
}

void readEepromValues()
{
  preferences.clear();
  preferences.begin(EEPROMNameSpace, false);
    
  debugFlag             = preferences.getBool("debugflag",false);
  NewOilSensorEquipped  = preferences.getBool("NewSensorflag",false);

  String temp = preferences.getString("SW_Version","1234");
  uint8_t leng = temp.length() +1;
  temp.toCharArray(SoftwareVersion, leng);

  temp = preferences.getString("Modulename","Sensor");
  leng = temp.length() +1;
  temp.toCharArray(Modulename, leng);
  
  OldOilTempCompValues[0] = preferences.getUShort("Old_sensor_Temperature_30",Old_sensor_Temperature_30);
  OldOilTempCompValues[1] = preferences.getUShort("Old_sensor_Temperature_40",Old_sensor_Temperature_40);
  OldOilTempCompValues[2] = preferences.getUShort("Old_sensor_Temperature_50",Old_sensor_Temperature_50);
  OldOilTempCompValues[3] = preferences.getUShort("Old_sensor_Temperature_55",Old_sensor_Temperature_55);
  OldOilTempCompValues[4] = preferences.getUShort("Old_sensor_Temperature_60",Old_sensor_Temperature_60);
  OldOilTempCompValues[5] = preferences.getUShort("Old_sensor_Temperature_65",Old_sensor_Temperature_65);
  OldOilTempCompValues[6] = preferences.getUShort("Old_sensor_Temperature_70",Old_sensor_Temperature_70);
  OldOilTempCompValues[7] = preferences.getUShort("Old_sensor_Temperature_75",Old_sensor_Temperature_75);
  OldOilTempCompValues[8] = preferences.getUShort("Old_sensor_Temperature_80",Old_sensor_Temperature_80);
  OldOilTempCompValues[9] = preferences.getUShort("Old_sensor_Temperature_85",Old_sensor_Temperature_85);
  OldOilTempCompValues[10] = preferences.getUShort("Old_sensor_Temperature_90",Old_sensor_Temperature_90);
  OldOilTempCompValues[11] = preferences.getUShort("Old_sensor_Temperature_95",Old_sensor_Temperature_95);
  OldOilTempCompValues[12] = preferences.getUShort("Old_sensor_Temperature_100",Old_sensor_Temperature_100);
  OldOilTempCompValues[13] = preferences.getUShort("Old_sensor_Temperature_105",Old_sensor_Temperature_105);
  OldOilTempCompValues[14] = preferences.getUShort("Old_sensor_Temperature_110",Old_sensor_Temperature_110);
  OldOilTempCompValues[15] = preferences.getUShort("Old_sensor_Temperature_115",Old_sensor_Temperature_115);

  OldOilLevelCompValues[0] = preferences.getUShort("Old_sensor_OilLevelEmpty",Old_sensor_OilLevelEmpty);
  OldOilLevelCompValues[1] = preferences.getUShort("Old_sensor_OilLevel_10",Old_sensor_OilLevel_10);
  OldOilLevelCompValues[2] = preferences.getUShort("Old_sensor_OilLevel_20",Old_sensor_OilLevel_20);
  OldOilLevelCompValues[3] = preferences.getUShort("Old_sensor_OilLevel_30",Old_sensor_OilLevel_30);
  OldOilLevelCompValues[4] = preferences.getUShort("Old_sensor_OilLevel_40",Old_sensor_OilLevel_40);
  OldOilLevelCompValues[5] = preferences.getUShort("Old_sensor_OilLevel_50",Old_sensor_OilLevel_50);
  OldOilLevelCompValues[6] = preferences.getUShort("Old_sensor_OilLevel_60",Old_sensor_OilLevel_60);
  OldOilLevelCompValues[7] = preferences.getUShort("Old_sensor_OilLevel_70",Old_sensor_OilLevel_70);
  OldOilLevelCompValues[8] = preferences.getUShort("Old_sensor_OilLevel_80",Old_sensor_OilLevel_80);
  OldOilLevelCompValues[9] = preferences.getUShort("Old_sensor_OilLevel_90",Old_sensor_OilLevel_90);
  OldOilLevelCompValues[10] = preferences.getUShort("Old_sensor_OilLevelFull",Old_sensor_OilLevelFull);

  NewOilTempCompValues[0] = preferences.getUShort("New_sensor_Temperature_30",New_sensor_Temperature_30);
  NewOilTempCompValues[1] = preferences.getUShort("New_sensor_Temperature_40",New_sensor_Temperature_40);
  NewOilTempCompValues[2] = preferences.getUShort("New_sensor_Temperature_50",New_sensor_Temperature_50);
  NewOilTempCompValues[3] = preferences.getUShort("New_sensor_Temperature_55",New_sensor_Temperature_55);
  NewOilTempCompValues[4] = preferences.getUShort("New_sensor_Temperature_60",New_sensor_Temperature_60);
  NewOilTempCompValues[5] = preferences.getUShort("New_sensor_Temperature_65",New_sensor_Temperature_65);
  NewOilTempCompValues[6] = preferences.getUShort("New_sensor_Temperature_70",New_sensor_Temperature_70);
  NewOilTempCompValues[7] = preferences.getUShort("New_sensor_Temperature_75",New_sensor_Temperature_75);
  NewOilTempCompValues[8] = preferences.getUShort("New_sensor_Temperature_80",New_sensor_Temperature_80);
  NewOilTempCompValues[9] = preferences.getUShort("New_sensor_Temperature_85",New_sensor_Temperature_85);
  NewOilTempCompValues[10] = preferences.getUShort("New_sensor_Temperature_90",New_sensor_Temperature_90);
  NewOilTempCompValues[11] = preferences.getUShort("New_sensor_Temperature_95",New_sensor_Temperature_95);
  NewOilTempCompValues[12] = preferences.getUShort("New_sensor_Temperature_100",New_sensor_Temperature_100);
  NewOilTempCompValues[13] = preferences.getUShort("New_sensor_Temperature_105",New_sensor_Temperature_105);
  NewOilTempCompValues[14] = preferences.getUShort("New_sensor_Temperature_110",New_sensor_Temperature_110);
  NewOilTempCompValues[15] = preferences.getUShort("New_sensor_Temperature_115",New_sensor_Temperature_115);

  NewOilLevelCompValues[0] = preferences.getUShort("New_sensor_OilLevelEmpty",New_sensor_OilLevelEmpty);
  NewOilLevelCompValues[1] = preferences.getUShort("New_sensor_OilLevel_10",New_sensor_OilLevel_10);
  NewOilLevelCompValues[2] = preferences.getUShort("New_sensor_OilLevel_20",New_sensor_OilLevel_20);
  NewOilLevelCompValues[3] = preferences.getUShort("New_sensor_OilLevel_30",New_sensor_OilLevel_30);
  NewOilLevelCompValues[4] = preferences.getUShort("New_sensor_OilLevel_40",New_sensor_OilLevel_40);
  NewOilLevelCompValues[5] = preferences.getUShort("New_sensor_OilLevel_50",New_sensor_OilLevel_50);
  NewOilLevelCompValues[6] = preferences.getUShort("New_sensor_OilLevel_60",New_sensor_OilLevel_60);
  NewOilLevelCompValues[7] = preferences.getUShort("New_sensor_OilLevel_70",New_sensor_OilLevel_70);
  NewOilLevelCompValues[8] = preferences.getUShort("New_sensor_OilLevel_80",New_sensor_OilLevel_80);
  NewOilLevelCompValues[9] = preferences.getUShort("New_sensor_OilLevel_90",New_sensor_OilLevel_90);
  NewOilLevelCompValues[10] = preferences.getUShort("New_sensor_OilLevelFull",New_sensor_OilLevelFull);

  preferences.end();
}



void setup() {
  readEepromValues();
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    for(;;);
  }

  delay(1000);
  //start serial connection
  SerialBT.begin(Modulename);
  
  /*configure pin SignalInputPin from #define as an input and enable the internal pull-up resistor*/
  pinMode(SignalInputPin, INPUT_PULLUP);

  /*configure pin ISRDebugTogglePin from #define as an Output to check how often ISR is called*/
  pinMode(ISRDebugTogglePin, OUTPUT);

  /*configure pin OilLevelLED from #define as an Output to proof that oil Level is ok*/
  //pinMode(OilLevelLED, OUTPUT);

  /*configure pin OilTemperatureLED from #define as an Output to proof that oil Temperature is ok*/
  //pinMode(OilTemperatureLED, OUTPUT);

  /*configure Pin for extra Output ok*/
  pinMode(OutputPin, OUTPUT);

  // Set timer frequency to 1Mhz
  timer = timerBegin(timerfrequency);

  // Attach onTimer function to our timer.
  timerAttachInterrupt(timer, &onTimer);

  // Set alarm to call onTimer function every second (value in microseconds).
  // Repeat the alarm (third parameter) with unlimited count = 0 (fourth parameter).
  timerAlarm(timer, 1000, true, 0);

  SerialBT.onData(getBTData);
  /* Design Reason Background is Dark all Text is White*/
  display.invertDisplay(true);
  
  /* clear display for startup*/
  display.clearDisplay();
}


void loop() {
  portENTER_CRITICAL(&timerMux);
  orderImpulse(cntRawArr);
  portEXIT_CRITICAL(&timerMux);

  convertImpulseToPercentage(returnArray[1], returnArray[3],debugFlag);
  sendInfosToBT(oilTemperature, oilLevelPercentage,returnArray);
  //TempToggle = showLevelAndTempAtLED(TempToggle,oilLevelPercentage,oilTemperature);
  analyse_BT_Protocol(BT_rx_buffer);
  controlOfDisplay();
  delay(1000);
  }
