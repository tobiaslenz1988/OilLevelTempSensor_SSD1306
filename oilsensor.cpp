#include <Arduino.h>
#include <stdio.h>
#include "oilsensor.h"


extern bool TimeoutSensorDetected  ;
extern bool NewOilSensorEquipped; 

extern uint8_t oilTemperature;
extern uint8_t oilLevelPercentage ;


extern uint8_t testValue_oilTemperature;
extern uint8_t testValue_oilLevelPercentage;




/* This methods converts the received impulses to an Percentage for OilLevel and Oil Temperature*/
void convertImpulseToPercentage(uint16_t cntRawData_Temp, uint16_t cntRawData_Level,bool debugFlag) {
  if (debugFlag == false) 
  { /* Debug of... normal Operation*/
    if(TimeoutSensorDetected == false)
	{
      if(NewOilSensorEquipped==true)
	  {
        if (cntRawData_Level > New_sensor_OilLevelFull) { oilLevelPercentage = 253; }else
        if (cntRawData_Level = New_sensor_OilLevelFull) { oilLevelPercentage = 100; }else
        if (cntRawData_Level >= New_sensor_OilLevel_90) { oilLevelPercentage = 90; }else
        if (cntRawData_Level >= New_sensor_OilLevel_80) { oilLevelPercentage = 80; }else
        if (cntRawData_Level >= New_sensor_OilLevel_70) { oilLevelPercentage = 70; }else
        if (cntRawData_Level >= New_sensor_OilLevel_60) { oilLevelPercentage = 60; }else
        if (cntRawData_Level >= New_sensor_OilLevel_50) { oilLevelPercentage = 50; }else
        if (cntRawData_Level >= New_sensor_OilLevel_40) { oilLevelPercentage = 40; }else
        if (cntRawData_Level >= New_sensor_OilLevel_30) { oilLevelPercentage = 30; }else
        if (cntRawData_Level >= New_sensor_OilLevel_20) { oilLevelPercentage = 20; }else
        if (cntRawData_Level >= New_sensor_OilLevel_10) { oilLevelPercentage = 10; }else
        if (cntRawData_Level >= New_sensor_OilLevelEmpty) { oilLevelPercentage = 0; }
    
      }else{
        if (cntRawData_Level > Old_sensor_OilLevelFull) { oilLevelPercentage = 253; }else
        if (cntRawData_Level = Old_sensor_OilLevelFull) { oilLevelPercentage = 100; }else
        if (cntRawData_Level >= Old_sensor_OilLevel_90) { oilLevelPercentage = 90; }else
        if (cntRawData_Level >= Old_sensor_OilLevel_80) { oilLevelPercentage = 80; }else
        if (cntRawData_Level >= Old_sensor_OilLevel_70) { oilLevelPercentage = 70; }else
        if (cntRawData_Level >= Old_sensor_OilLevel_60) { oilLevelPercentage = 60; }else
        if (cntRawData_Level >= Old_sensor_OilLevel_50) { oilLevelPercentage = 50; }else
        if (cntRawData_Level >= Old_sensor_OilLevel_40) { oilLevelPercentage = 40; }else
        if (cntRawData_Level >= Old_sensor_OilLevel_30) { oilLevelPercentage = 30; }else
        if (cntRawData_Level >= Old_sensor_OilLevel_20) { oilLevelPercentage = 20; }else
        if (cntRawData_Level >= Old_sensor_OilLevel_10) { oilLevelPercentage = 10; }else
        if (cntRawData_Level >= Old_sensor_OilLevelEmpty) {oilLevelPercentage = 0; }
      }


      if(NewOilSensorEquipped==true){
        if (oilLevelPercentage > 0) {
          if (cntRawData_Temp > New_sensor_Temperature_115) { oilTemperature = 253; }else
          if (cntRawData_Temp = New_sensor_Temperature_115) { oilTemperature = 115; }else
          if (cntRawData_Temp >= New_sensor_Temperature_110) { oilTemperature = 110; }else
          if (cntRawData_Temp >= New_sensor_Temperature_105) { oilTemperature = 105; }else
          if (cntRawData_Temp >= New_sensor_Temperature_100) { oilTemperature = 100; }else
          if (cntRawData_Temp >= New_sensor_Temperature_95) { oilTemperature = 95; }else
          if (cntRawData_Temp >= New_sensor_Temperature_90) { oilTemperature = 90; }else
          if (cntRawData_Temp >= New_sensor_Temperature_85) { oilTemperature = 85; }else
          if (cntRawData_Temp >= New_sensor_Temperature_80) { oilTemperature = 80; }else
          if (cntRawData_Temp >= New_sensor_Temperature_75) { oilTemperature = 75; }else
          if (cntRawData_Temp >= New_sensor_Temperature_70) { oilTemperature = 70; }else
          if (cntRawData_Temp >= New_sensor_Temperature_65) { oilTemperature = 65; }else
          if (cntRawData_Temp >= New_sensor_Temperature_60) { oilTemperature = 60; }else
          if (cntRawData_Temp >= New_sensor_Temperature_55) { oilTemperature = 55; }else
          if (cntRawData_Temp >= New_sensor_Temperature_50) { oilTemperature = 50; }else
          if (cntRawData_Temp >= New_sensor_Temperature_40) { oilTemperature = 40; }
        }else{ oilTemperature = OilTemperaturePercentageErrorValue;}
      }else{
        if (oilLevelPercentage > 0) 
        { if (cntRawData_Temp > Old_sensor_Temperature_115) { oilTemperature = 253; }else
          if (cntRawData_Temp = Old_sensor_Temperature_115) { oilTemperature = 115; }else
          if (cntRawData_Temp >= Old_sensor_Temperature_110) { oilTemperature = 110; }else
          if (cntRawData_Temp >= Old_sensor_Temperature_105) { oilTemperature = 105; }else
          if (cntRawData_Temp >= Old_sensor_Temperature_100) { oilTemperature = 100; }else
          if (cntRawData_Temp >= Old_sensor_Temperature_95) { oilTemperature = 95; }else
          if (cntRawData_Temp >= Old_sensor_Temperature_90) { oilTemperature = 90; }else
          if (cntRawData_Temp >= Old_sensor_Temperature_85) { oilTemperature = 85; }else
          if (cntRawData_Temp >= Old_sensor_Temperature_80) { oilTemperature = 80; }else
          if (cntRawData_Temp >= Old_sensor_Temperature_75) { oilTemperature = 75; }else
          if (cntRawData_Temp >= Old_sensor_Temperature_70) { oilTemperature = 70; }else
          if (cntRawData_Temp >= Old_sensor_Temperature_65) { oilTemperature = 65; }else
          if (cntRawData_Temp >= Old_sensor_Temperature_60) { oilTemperature = 60; }else
          if (cntRawData_Temp >= Old_sensor_Temperature_55) { oilTemperature = 55; }else
          if (cntRawData_Temp >= Old_sensor_Temperature_50) { oilTemperature = 50; }else
          if (cntRawData_Temp >= Old_sensor_Temperature_40) { oilTemperature = 40; }else
          if (cntRawData_Temp >= Old_sensor_Temperature_30) { oilTemperature = 30; }
        }else{ oilTemperature = OilLevelPercentageErrorValue;}
      }


    }else{
      /* if Oil Sensor is disconnected  */
      oilTemperature      = OilTemperaturePercentageErrorValue;
      oilLevelPercentage  = OilLevelPercentageErrorValue;
    }
  } else {
    /*Debug On*/
    oilTemperature      = testValue_oilTemperature;
    oilLevelPercentage  = testValue_oilLevelPercentage;
  }
}
