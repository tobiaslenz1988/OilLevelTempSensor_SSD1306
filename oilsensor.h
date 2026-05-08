#ifndef OILSENSOR_H
#define OILSENSOR_H


#define New_sensor_OilLevelEmpty	  0x00
#define New_sensor_OilLevel_10        0x00
#define New_sensor_OilLevel_20        0x00
#define New_sensor_OilLevel_30        0x00
#define New_sensor_OilLevel_40		  0x00
#define	New_sensor_OilLevel_50		  0x00
#define New_sensor_OilLevel_60		  0x00
#define New_sensor_OilLevel_70		  0x00
#define New_sensor_OilLevel_80		  0x00
#define New_sensor_OilLevel_90		  0x00
#define New_sensor_OilLevelFull       0x00


#define New_sensor_Temperature_30	  0x00
#define New_sensor_Temperature_40	  0x00
#define New_sensor_Temperature_50	  0x00
#define New_sensor_Temperature_55	  0x00
#define New_sensor_Temperature_60	  0x00
#define New_sensor_Temperature_65	  0x00
#define New_sensor_Temperature_70	  0x00
#define New_sensor_Temperature_75     0x00
#define New_sensor_Temperature_80	  0x00
#define New_sensor_Temperature_85	  0x00
#define New_sensor_Temperature_90	  0x00
#define New_sensor_Temperature_95	  0x00
#define New_sensor_Temperature_100	  0x00
#define New_sensor_Temperature_105	  0x00
#define New_sensor_Temperature_110	  0x00
#define New_sensor_Temperature_115	  0x00


#define Old_sensor_OilLevelEmpty	  0x00
#define Old_sensor_OilLevel_10        0x00
#define Old_sensor_OilLevel_20        0x00
#define Old_sensor_OilLevel_30        0x00
#define Old_sensor_OilLevel_40		  0x00
#define	Old_sensor_OilLevel_50		  0x00
#define Old_sensor_OilLevel_60		  0x00
#define Old_sensor_OilLevel_70		  0x00
#define Old_sensor_OilLevel_80		  0x00
#define Old_sensor_OilLevel_90		  0x00
#define Old_sensor_OilLevelFull       0x00


#define Old_sensor_Temperature_30	  0x00
#define Old_sensor_Temperature_40	  0x00
#define Old_sensor_Temperature_50	  0x00
#define Old_sensor_Temperature_55	  0x00
#define Old_sensor_Temperature_60	  0x00
#define Old_sensor_Temperature_65	  0x00
#define Old_sensor_Temperature_70	  0x00
#define Old_sensor_Temperature_75     0x00
#define Old_sensor_Temperature_80	  0x00
#define Old_sensor_Temperature_85	  0x00
#define Old_sensor_Temperature_90	  0x00
#define Old_sensor_Temperature_95	  0x00
#define Old_sensor_Temperature_100	  0x00
#define Old_sensor_Temperature_105	  0x00
#define Old_sensor_Temperature_110	  0x00
#define Old_sensor_Temperature_115	  0x00
					
																						
										

#define OilLevelPercentageErrorValue			0xFF
#define OilTemperaturePercentageErrorValue      0xFF

#define OilLevelPercentageInitValue				0xFE
#define OilTemperaturePercentageInitValue     	0xFE


#define OilLevelPercentageDebugValue		    0xFD
#define OilTemperaturePercentageDebugValue     	0xFD


extern bool TimeoutSensorDetected;
extern bool NewOilSensorEquipped; 

extern uint8_t oilTemperature;
extern uint8_t oilLevelPercentage ;



extern uint8_t testValue_oilTemperature;
extern uint8_t testValue_oilLevelPercentage;
extern uint16_t OilOldLevelCompareValues[];
extern uint16_t OilOldTemperatureCompareValues[];
void convertImpulseToPercentage(uint16_t cntRawData_Temp, uint16_t cntRawData_Level,uint8_t session);

#endif /* OILSENSOR_H */