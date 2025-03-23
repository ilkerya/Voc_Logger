
#define uint8_t byte
#define uint16_t unsigned int
#define uint32_t unsigned long

#define int8_t signed char
#define int16_t signed int
#define int32_t signed long

#define INTERRUPT_BASED 1
#define LOOP_BASED 0

/*
#define GROVE_GAS_V2_MD5_EXISTS // ADR3
#define GROVE_GAS_V2_MD6_EXISTS // ADR2
#define GROVE_GAS_V2_MD7_EXISTS // ADR5
#define GROVE_GAS_V2_MD8_EXISTS // ADR4


#if (defined GROVE_GAS_V2_MD8_EXISTS  || defined GROVE_GAS_V2_MD7_EXISTS  || defined  GROVE_GAS_V2_MD6_EXISTS || defined GROVE_GAS_V2_MD5_EXISTS )
struct Sensor_GroveGas_V2
{
  uint32_t NO2 = 0;
  uint32_t C2H5OH = 0;  
  uint32_t VOC = 0;
  uint32_t CO = 0;  
};
Sensor_GroveGas_V2 Multi_Gas;
#endif

#ifdef GROVE_GAS_V2_MD5_EXISTS 
  Sensor_GroveGas_V2 Multi_Gas_1;
#endif
#ifdef GROVE_GAS_V2_MD6_EXISTS 
  Sensor_GroveGas_V2 Multi_Gas_2;
#endif
#ifdef GROVE_GAS_V2_MD7_EXISTS 
  Sensor_GroveGas_V2 Multi_Gas_3;
#endif
#ifdef GROVE_GAS_V2_MD8_EXISTS 
  Sensor_GroveGas_V2 Multi_Gas_4;
#endif

struct
{
  float Humidity_OnBoard;
  float Temperature_OnBoard; // 27   
}Values;
*/

void DAQ_Send_Data(bool Int_Loop);


/*
uint8_t Counter_100mS;
uint8_t Counter_10mS;
bool Tick_100mS;
bool Tick_10mS;
*/
struct DAQ{
  uint8_t Busy;
  uint8_t SendEnable; // SeriPort.SendEnable
  uint8_t ContBytes;
  uint16_t Length;
  uint8_t SendBuf[DEFAULT_TX_LENGTH];
  uint16_t CRC_Send;
  uint16_t CRC_Receive;
  uint16_t CRC_Calc_Receive;
  bool Ack;
}DAQ;

struct Accelometer{
  float  x;
  float  y;
  float  z;   
}Accelometer;

int32_t GetMod_Float(float Acc){
   if(Acc > 128)  return 1280000;
   if(Acc < -128) return -1280000;
   return (int32_t)(Acc*10000)  ;
  }
  uint16_t ShowCounter; 
void DAQ_Send_Data(bool Int_Loop){

  uint8_t *p;
  p = &DAQ.SendBuf[0];

  uint8_t i = 0; //Uint32 CRC = 0;
     
  p[0] = 0XAA; // preamble D170
  p[1] = 0XAA; // preamble D170
    DAQ.Length = DEFAULT_TX_LENGTH;
  p[2] = DAQ.Length >> 8;
  p[3] = DAQ.Length; //  packet total size
  if(DAQ.Ack) p[4] = ACKNOWLEDGE;
  else        p[4] = DATA_BYTE;
  DAQ.Ack = OFF;

uint8_t Index = 4;
  p[++Index] = (uint8_t)(Multi_Gas_1.VOC >> 24);
  p[++Index] = (uint8_t)(Multi_Gas_1.VOC >> 16);
  p[++Index] = (uint8_t)(Multi_Gas_1.VOC >> 8);
  p[++Index] = Multi_Gas_1.VOC;

  p[++Index] = (uint8_t)(Multi_Gas_1.C2H5OH >> 24);
  p[++Index] = (uint8_t)(Multi_Gas_1.C2H5OH >> 16);
 p[++Index] = (uint8_t)(Multi_Gas_1.C2H5OH >> 8);
 p[++Index] = Multi_Gas_1.C2H5OH;

 p[++Index] = (uint8_t)(Multi_Gas_1.CO >> 24);
 p[++Index] = (uint8_t)(Multi_Gas_1.CO >> 16);
 p[++Index] = (uint8_t)(Multi_Gas_1.CO >> 8);
 p[++Index] = Multi_Gas_1.CO;

  p[++Index] = (uint8_t)(Multi_Gas_1.NO2 >> 24);
  p[++Index] = (uint8_t)(Multi_Gas_1.NO2 >> 16);
  p[++Index] = (uint8_t)(Multi_Gas_1.NO2 >> 8);
  p[++Index] = Multi_Gas_1.NO2;
  ////////////////////////////////////////////////////////////////
  p[++Index] = (uint8_t)(Multi_Gas_2.VOC >> 24);
  p[++Index] = (uint8_t)(Multi_Gas_2.VOC >> 16);
  p[++Index] = (uint8_t)(Multi_Gas_2.VOC >> 8);
  p[++Index] = Multi_Gas_2.VOC;

  p[++Index] = (uint8_t)(Multi_Gas_2.C2H5OH >> 24);
  p[++Index] = (uint8_t)(Multi_Gas_2.C2H5OH >> 16);
  p[++Index] = (uint8_t)(Multi_Gas_2.C2H5OH >> 8);
  p[++Index] = Multi_Gas_2.C2H5OH;

  p[++Index] = (uint8_t)(Multi_Gas_2.CO >> 24);
  p[++Index] = (uint8_t)(Multi_Gas_2.CO >> 16);
  p[++Index] = (uint8_t)(Multi_Gas_2.CO >> 8);
  p[++Index] = Multi_Gas_2.CO;

  p[++Index] = (uint8_t)(Multi_Gas_2.NO2 >> 24);
  p[++Index] = (uint8_t)(Multi_Gas_2.NO2 >> 16);
  p[++Index] = (uint8_t)(Multi_Gas_2.NO2 >> 8);
  p[++Index] = Multi_Gas_2.NO2;  
  /////////////////////////////////////////////////////////////
  p[++Index] = (uint8_t)(Multi_Gas_3.VOC >> 24);
  p[++Index] = (uint8_t)(Multi_Gas_3.VOC >> 16);
  p[++Index] = (uint8_t)(Multi_Gas_3.VOC >> 8);
  p[++Index] = Multi_Gas_3.VOC;  

  p[++Index] = (uint8_t)(Multi_Gas_3.C2H5OH >> 24); 
  p[++Index] = (uint8_t)(Multi_Gas_3.C2H5OH >> 16); 
  p[++Index] = (uint8_t)(Multi_Gas_3.C2H5OH >> 8); 
  p[++Index] = Multi_Gas_3.C2H5OH ; 

  p[++Index] = (uint8_t)(Multi_Gas_3.CO >> 24); 
  p[++Index] = (uint8_t)(Multi_Gas_3.CO >> 16); 
  p[++Index] = (uint8_t)(Multi_Gas_3.CO >> 8); 
  p[++Index] = Multi_Gas_3.CO ; 

  p[++Index] = (uint8_t)(Multi_Gas_3.NO2 >> 24); 
  p[++Index] = (uint8_t)(Multi_Gas_3.NO2 >> 16); 
  p[++Index] = (uint8_t)(Multi_Gas_3.NO2 >> 8); 
  p[++Index] = Multi_Gas_3.NO2 ; 
  /////////////////////////////////////////////////////////////
  p[++Index] = (uint8_t)(Multi_Gas_4.VOC >> 24);
  p[++Index] = (uint8_t)(Multi_Gas_4.VOC >> 16);
  p[++Index] = (uint8_t)(Multi_Gas_4.VOC >> 8);
  p[++Index] = Multi_Gas_4.VOC;  

  p[++Index] = (uint8_t)(Multi_Gas_4.C2H5OH >> 24); 
  p[++Index] = (uint8_t)(Multi_Gas_4.C2H5OH >> 16); 
  p[++Index] = (uint8_t)(Multi_Gas_4.C2H5OH >> 8); 
  p[++Index] = Multi_Gas_4.C2H5OH ; 

  p[++Index] = (uint8_t)(Multi_Gas_4.CO >> 24); 
  p[++Index] = (uint8_t)(Multi_Gas_4.CO >> 16); 
  p[++Index] = (uint8_t)(Multi_Gas_4.CO >> 8); 
  p[++Index] = Multi_Gas_4.CO ; 

  p[++Index] = (uint8_t)(Multi_Gas_4.NO2 >> 24); 
  p[++Index] = (uint8_t)(Multi_Gas_4.NO2 >> 16); 
  p[++Index] = (uint8_t)(Multi_Gas_4.NO2 >> 8); 
  p[++Index] = Multi_Gas_4.NO2 ; 
//////////////////////////////////////////////////////////

  //  Values.Temperature_OnBoard = 23.56;
 //   Values.Humidity_OnBoard = 52.45;

    //    Bosch_BME688_1.Temperature = bme.temperature;
     //   Bosch_BME688_1.Humidity = bme.humidity;
    //    Bosch_BME688_1.Pressure = bme.pressure  / 100.0;
     //   Bosch_BME688_1.Gas = bme.gas_resistance / 1000.0;

  int32_t Temp;

  Temp = (int32_t)roundf(Bosch_BME688_1.Temperature *100);
  p[++Index] = (uint8_t)(Temp >> 24); 
  p[++Index] = (uint8_t)(Temp >> 16); 
  p[++Index] = (uint8_t)(Temp >> 8); 
  p[++Index] = Temp ;

 Temp = (int32_t)roundf(Bosch_BME688_1.Humidity *100);
  p[++Index] = (uint8_t)(Temp >> 24); 
  p[++Index] = (uint8_t)(Temp >> 16); 
  p[++Index] = (uint8_t)(Temp >> 8); 
  p[++Index] = Temp ;

 Temp = (int32_t)roundf(Bosch_BME688_1.Gas *100);
  p[++Index] = (uint8_t)(Temp >> 24); 
  p[++Index] = (uint8_t)(Temp >> 16); 
  p[++Index] = (uint8_t)(Temp >> 8); 
  p[++Index] = Temp ;

 Temp = (int32_t)roundf(Bosch_BME688_1.Pressure *100);
  p[++Index] = (uint8_t)(Temp >> 24); 
  p[++Index] = (uint8_t)(Temp >> 16); 
  p[++Index] = (uint8_t)(Temp >> 8); 
  p[++Index] = Temp ;

  Temp = (int32_t)roundf(Bosch_BME688_2.Temperature *100);
  p[++Index] = (uint8_t)(Temp >> 24); 
  p[++Index] = (uint8_t)(Temp >> 16); 
  p[++Index] = (uint8_t)(Temp >> 8); 
  p[++Index] = Temp ;

 Temp = (int32_t)roundf(Bosch_BME688_2.Humidity *100);
  p[++Index] = (uint8_t)(Temp >> 24); 
  p[++Index] = (uint8_t)(Temp >> 16); 
  p[++Index] = (uint8_t)(Temp >> 8); 
  p[++Index] = Temp ;

 Temp = (int32_t)roundf(Bosch_BME688_2.Gas *100);
  p[++Index] = (uint8_t)(Temp >> 24); 
  p[++Index] = (uint8_t)(Temp >> 16); 
  p[++Index] = (uint8_t)(Temp >> 8); 
  p[++Index] = Temp ;

 Temp = (int32_t)roundf(Bosch_BME688_2.Pressure *100);
  p[++Index] = (uint8_t)(Temp >> 24); 
  p[++Index] = (uint8_t)(Temp >> 16); 
  p[++Index] = (uint8_t)(Temp >> 8); 
  p[++Index] = Temp ;

  Temp = (int32_t)roundf(Bosch_BME688_3.Temperature *100);
  p[++Index] = (uint8_t)(Temp >> 24); 
  p[++Index] = (uint8_t)(Temp >> 16); 
  p[++Index] = (uint8_t)(Temp >> 8); 
  p[++Index] = Temp ;

 Temp = (int32_t)roundf(Bosch_BME688_3.Humidity *100);
  p[++Index] = (uint8_t)(Temp >> 24); 
  p[++Index] = (uint8_t)(Temp >> 16); 
  p[++Index] = (uint8_t)(Temp >> 8); 
  p[++Index] = Temp ;

 Temp = (int32_t)roundf(Bosch_BME688_3.Gas *100);
  p[++Index] = (uint8_t)(Temp >> 24); 
  p[++Index] = (uint8_t)(Temp >> 16); 
  p[++Index] = (uint8_t)(Temp >> 8); 
  p[++Index] = Temp ;

 Temp = (int32_t)roundf(Bosch_BME688_3.Pressure *100);
  p[++Index] = (uint8_t)(Temp >> 24); 
  p[++Index] = (uint8_t)(Temp >> 16); 
  p[++Index] = (uint8_t)(Temp >> 8); 
  p[++Index] = Temp ;

  Temp = (int32_t)roundf(Bosch_BME688_4.Temperature *100);
  p[++Index] = (uint8_t)(Temp >> 24); 
  p[++Index] = (uint8_t)(Temp >> 16); 
  p[++Index] = (uint8_t)(Temp >> 8); 
  p[++Index] = Temp ;

 Temp = (int32_t)roundf(Bosch_BME688_4.Humidity *100);
  p[++Index] = (uint8_t)(Temp >> 24); 
  p[++Index] = (uint8_t)(Temp >> 16); 
  p[++Index] = (uint8_t)(Temp >> 8); 
  p[++Index] = Temp ;

 Temp = (int32_t)roundf(Bosch_BME688_4.Gas *100);
  p[++Index] = (uint8_t)(Temp >> 24); 
  p[++Index] = (uint8_t)(Temp >> 16); 
  p[++Index] = (uint8_t)(Temp >> 8); 
  p[++Index] = Temp ;

 Temp = (int32_t)roundf(Bosch_BME688_4.Pressure *100);
  p[++Index] = (uint8_t)(Temp >> 24); 
  p[++Index] = (uint8_t)(Temp >> 16); 
  p[++Index] = (uint8_t)(Temp >> 8); 
  p[++Index] = Temp ;

  p[++Index] = Duty_Cycle;  // 129
  p[++Index] = Duty_Cycle;  // 130
/*
  p[4] = (uint8_t)(Multi_Gas_1.VOC >> 24);
  p[5] = (uint8_t)(Multi_Gas_1.VOC >> 16);
  p[6] = (uint8_t)(Multi_Gas_1.VOC >> 8);
  p[7] = Multi_Gas_1.VOC;

  p[8] = (uint8_t)(Multi_Gas_1.C2H5OH >> 24);
  p[9] = (uint8_t)(Multi_Gas_1.C2H5OH >> 16);
 p[10] = (uint8_t)(Multi_Gas_1.C2H5OH >> 8);
 p[11] = Multi_Gas_1.C2H5OH;

 p[12] = (uint8_t)(Multi_Gas_1.CO >> 24);
 p[13] = (uint8_t)(Multi_Gas_1.CO >> 16);
 p[14] = (uint8_t)(Multi_Gas_1.CO >> 8);
 p[15] = Multi_Gas_1.CO;

  p[16] = (uint8_t)(Multi_Gas_1.NO2 >> 24);
  p[17] = (uint8_t)(Multi_Gas_1.NO2 >> 16);
  p[18] = (uint8_t)(Multi_Gas_1.NO2 >> 8);
  p[19] = Multi_Gas_1.NO2;
  ////////////////////////////////////////////////////////////////
  p[20] = (uint8_t)(Multi_Gas_2.VOC >> 24);
  p[21] = (uint8_t)(Multi_Gas_2.VOC >> 16);
  p[22] = (uint8_t)(Multi_Gas_2.VOC >> 8);
  p[23] = Multi_Gas_2.VOC;

  p[24] = (uint8_t)(Multi_Gas_2.C2H5OH >> 24);
  p[25] = (uint8_t)(Multi_Gas_2.C2H5OH >> 16);
  p[26] = (uint8_t)(Multi_Gas_2.C2H5OH >> 8);
  p[27] = Multi_Gas_2.C2H5OH;

  p[28] = (uint8_t)(Multi_Gas_2.CO >> 24);
  p[29] = (uint8_t)(Multi_Gas_2.CO >> 16);
  p[30] = (uint8_t)(Multi_Gas_2.CO >> 8);
  p[31] = Multi_Gas_2.CO;

  p[32] = (uint8_t)(Multi_Gas_2.NO2 >> 24);
  p[33] = (uint8_t)(Multi_Gas_2.NO2 >> 16);
  p[34] = (uint8_t)(Multi_Gas_2.NO2 >> 8);
  p[35] = Multi_Gas_2.NO2;  
  /////////////////////////////////////////////////////////////
  p[36] = (uint8_t)(Multi_Gas_3.VOC >> 24);
  p[37] = (uint8_t)(Multi_Gas_3.VOC >> 16);
  p[38] = (uint8_t)(Multi_Gas_3.VOC >> 8);
  p[39] = Multi_Gas_3.VOC;  

  p[40] = (uint8_t)(Multi_Gas_3.C2H5OH >> 24); 
  p[41] = (uint8_t)(Multi_Gas_3.C2H5OH >> 16); 
  p[42] = (uint8_t)(Multi_Gas_3.C2H5OH >> 8); 
  p[43] = Multi_Gas_3.C2H5OH ; 

  p[44] = (uint8_t)(Multi_Gas_3.CO >> 24); 
  p[45] = (uint8_t)(Multi_Gas_3.CO >> 16); 
  p[46] = (uint8_t)(Multi_Gas_3.CO >> 8); 
  p[47] = Multi_Gas_3.CO ; 

  p[48] = (uint8_t)(Multi_Gas_3.NO2 >> 24); 
  p[49] = (uint8_t)(Multi_Gas_3.NO2 >> 16); 
  p[50] = (uint8_t)(Multi_Gas_3.NO2 >> 8); 
  p[51] = Multi_Gas_3.NO2 ; 
  /////////////////////////////////////////////////////////////
  p[52] = (uint8_t)(Multi_Gas_4.VOC >> 24);
  p[53] = (uint8_t)(Multi_Gas_4.VOC >> 16);
  p[54] = (uint8_t)(Multi_Gas_4.VOC >> 8);
  p[55] = Multi_Gas_4.VOC;  

  p[56] = (uint8_t)(Multi_Gas_4.C2H5OH >> 24); 
  p[57] = (uint8_t)(Multi_Gas_4.C2H5OH >> 16); 
  p[58] = (uint8_t)(Multi_Gas_4.C2H5OH >> 8); 
  p[59] = Multi_Gas_4.C2H5OH ; 

  p[60] = (uint8_t)(Multi_Gas_4.CO >> 24); 
  p[61] = (uint8_t)(Multi_Gas_4.CO >> 16); 
  p[62] = (uint8_t)(Multi_Gas_4.CO >> 8); 
  p[63] = Multi_Gas_4.CO ; 

  p[64] = (uint8_t)(Multi_Gas_4.NO2 >> 24); 
  p[65] = (uint8_t)(Multi_Gas_4.NO2 >> 16); 
  p[66] = (uint8_t)(Multi_Gas_4.NO2 >> 8); 
  p[67] = Multi_Gas_4.NO2 ; 
//////////////////////////////////////////////////////////

  //  Values.Temperature_OnBoard = 23.56;
 //   Values.Humidity_OnBoard = 52.45;

    //    Bosch_BME688_1.Temperature = bme.temperature;
     //   Bosch_BME688_1.Humidity = bme.humidity;
    //    Bosch_BME688_1.Pressure = bme.pressure  / 100.0;
     //   Bosch_BME688_1.Gas = bme.gas_resistance / 1000.0;

  int32_t Temp;

  Temp = (int32_t)roundf(Bosch_BME688_1.Temperature *100);
  p[68] = (uint8_t)(Temp >> 24); 
  p[69] = (uint8_t)(Temp >> 16); 
  p[70] = (uint8_t)(Temp >> 8); 
  p[71] = Temp ;

 Temp = (int32_t)roundf(Bosch_BME688_1.Humidity *100);
  p[72] = (uint8_t)(Temp >> 24); 
  p[73] = (uint8_t)(Temp >> 16); 
  p[74] = (uint8_t)(Temp >> 8); 
  p[75] = Temp ;

 Temp = (int32_t)roundf(Bosch_BME688_1.Gas *100);
  p[76] = (uint8_t)(Temp >> 24); 
  p[77] = (uint8_t)(Temp >> 16); 
  p[78] = (uint8_t)(Temp >> 8); 
  p[79] = Temp ;

 Temp = (int32_t)roundf(Bosch_BME688_1.Pressure *100);
  p[80] = (uint8_t)(Temp >> 24); 
  p[81] = (uint8_t)(Temp >> 16); 
  p[82] = (uint8_t)(Temp >> 8); 
  p[83] = Temp ;

  Temp = (int32_t)roundf(Bosch_BME688_2.Temperature *100);
  p[84] = (uint8_t)(Temp >> 24); 
  p[85] = (uint8_t)(Temp >> 16); 
  p[86] = (uint8_t)(Temp >> 8); 
  p[87] = Temp ;

 Temp = (int32_t)roundf(Bosch_BME688_2.Humidity *100);
  p[88] = (uint8_t)(Temp >> 24); 
  p[89] = (uint8_t)(Temp >> 16); 
  p[90] = (uint8_t)(Temp >> 8); 
  p[91] = Temp ;

 Temp = (int32_t)roundf(Bosch_BME688_2.Gas *100);
  p[92] = (uint8_t)(Temp >> 24); 
  p[93] = (uint8_t)(Temp >> 16); 
  p[94] = (uint8_t)(Temp >> 8); 
  p[95] = Temp ;

 Temp = (int32_t)roundf(Bosch_BME688_2.Pressure *100);
  p[96] = (uint8_t)(Temp >> 24); 
  p[97] = (uint8_t)(Temp >> 16); 
  p[98] = (uint8_t)(Temp >> 8); 
  p[99] = Temp ;

  Temp = (int32_t)roundf(Bosch_BME688_3.Temperature *100);
  p[100] = (uint8_t)(Temp >> 24); 
  p[101] = (uint8_t)(Temp >> 16); 
  p[102] = (uint8_t)(Temp >> 8); 
  p[103] = Temp ;

 Temp = (int32_t)roundf(Bosch_BME688_3.Humidity *100);
  p[104] = (uint8_t)(Temp >> 24); 
  p[105] = (uint8_t)(Temp >> 16); 
  p[106] = (uint8_t)(Temp >> 8); 
  p[107] = Temp ;

 Temp = (int32_t)roundf(Bosch_BME688_3.Gas *100);
  p[108] = (uint8_t)(Temp >> 24); 
  p[109] = (uint8_t)(Temp >> 16); 
  p[110] = (uint8_t)(Temp >> 8); 
  p[111] = Temp ;

 Temp = (int32_t)roundf(Bosch_BME688_3.Pressure *100);
  p[112] = (uint8_t)(Temp >> 24); 
  p[113] = (uint8_t)(Temp >> 16); 
  p[114] = (uint8_t)(Temp >> 8); 
  p[115] = Temp ;

  Temp = (int32_t)roundf(Bosch_BME688_4.Temperature *100);
  p[116] = (uint8_t)(Temp >> 24); 
  p[117] = (uint8_t)(Temp >> 16); 
  p[118] = (uint8_t)(Temp >> 8); 
  p[119] = Temp ;

 Temp = (int32_t)roundf(Bosch_BME688_4.Humidity *100);
  p[120] = (uint8_t)(Temp >> 24); 
  p[121] = (uint8_t)(Temp >> 16); 
  p[122] = (uint8_t)(Temp >> 8); 
  p[123] = Temp ;

 Temp = (int32_t)roundf(Bosch_BME688_4.Gas *100);
  p[124] = (uint8_t)(Temp >> 24); 
  p[125] = (uint8_t)(Temp >> 16); 
  p[126] = (uint8_t)(Temp >> 8); 
  p[127] = Temp ;

 Temp = (int32_t)roundf(Bosch_BME688_4.Pressure *100);
  p[128] = (uint8_t)(Temp >> 24); 
  p[129] = (uint8_t)(Temp >> 16); 
  p[130] = (uint8_t)(Temp >> 8); 
  p[131] = Temp ;
*/



  DAQ.CRC_Send = DEFAULT_CRC_INIT;//2-14
  for(i=PREAMBLE_BYTES; i< (DEFAULT_TX_LENGTH-CRC_BYTES); i++){ // crc haric
    DAQ.CRC_Send ^= p[i];
  }
  
  DAQ.CRC_Send <<= 8;
  for(i=PREAMBLE_BYTES; i< (DEFAULT_TX_LENGTH-CRC_BYTES); i++){ // crc haric
    DAQ.CRC_Send ^= p[i];
  }

  p[DEFAULT_TX_LENGTH-2] = DAQ.CRC_Send >> 8;// DEFAULT_LENGTH-2
  p[DEFAULT_TX_LENGTH-1] = DAQ.CRC_Send; // DEFAULT_LENGTH-1
   
   //for(i=0; i< DEFAULT_LENGTH; i++){ // crc haric
   //Serial1.write(p[i]);
    //}
   Serial2.write(p, DEFAULT_TX_LENGTH);
   
  if(!Int_Loop){
    for(i=0; i< DEFAULT_TX_LENGTH; i++){ // crc haric
      Serial.print(i);Serial.print(": ");
      Serial.print(p[i]);Serial.print(" ");
    }
    Serial.println();
  }
}

  