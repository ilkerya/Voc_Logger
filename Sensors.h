

#define TCAADDR 0x70  //  
//#define CHANNEL_VOC 2 //  
#define CHANNEL_TEMPHUM 7  // 0x40 default address 
#define CHANNEL_LIGHT 6  // 0x29 default address sio71 
#define CHANNEL_BAROMETRIC 5  // 0x77 default address bmp388 
#define CHANNEL_ACCEL_GYRO 1  // 0x77 default address LSM9DS1 accoloremoter/Gyroscope/MAgnetometer 
//                            0x68 RTC sdefault address
// https://cdn-learn.adafruit.com/downloads/pdf/adafruit-tsl2591.pdf
// https://cdn-shop.adafruit.com/datasheets/TSL25911_Datasheet_EN_v1.pdf
//https://cdn-learn.adafruit.com/downloads/pdf/adafruit-bmp388.pdf?timestamp=1556108471
// https://cdn-learn.adafruit.com/downloads/pdf/adafruit-si7021-temperature-plus-humidity-sensor.pdf
// https://learn.adafruit.com/adafruit-lsm9ds1-accelerometer-plus-gyro-plus-magnetometer-9-dof-breakout  
// https://moderndevice.com/product/wind-sensor/
// https://cdn.sparkfun.com/assets/parts/1/2/2/7/5/SDS021_laser_PM2.5_sensor_specification-V1.0.pdf    // PM2.5 dust particle sensor

// https://www.lem.com/en/5-b10   current sensor LEM AT5B10
// https://www.microchip.com/wwwproducts/en/MCP6002 for Buffering Cuurent Sensor AT5B10
   
// Differenatial I2C communication
// https://www.sparkfun.com/products/14685      //I2c multiplexer
// https://www.sparkfun.com/products/14589      // I2c single ended 2 differential 
// https://cdn.sparkfun.com/assets/c/4/7/0/9/SparkFun_Differential_I2C_Breakout_PCA9615_Qwiic.pdf

// https://cdn.sparkfun.com/assets/parts/1/2/2/7/5/Laser_Dust_Sensor_Control_Protocol_V1.3.pdf
// https://cdn.sparkfun.com/assets/parts/1/2/2/7/5/SDS021_laser_PM2.5_sensor_specification-V1.0.pdf
/*
int rxPin = 19;
int txPin = 18;
SdsDustSensor sds(rxPin, txPin);
*/
//SdsDustSensor sds(Serial1); // passing HardwareSerial& as parameter
//SdsDustSensor sds(HardwareSerial); // passing HardwareSerial& as parameter
#if (defined TEMP_HUM_1_SENSOR_EXISTS  || defined TEMP_HUM_2_SENSOR_EXISTS  || defined  TEMP_HUM_3_SENSOR_EXISTS || defined TEMP_HUM_ONBOARD_SENSOR_EXISTS )
 // #include "Adafruit_Si7021.h"
  Adafruit_Si7021 THsensor = Adafruit_Si7021();
#endif


#if (defined BME688_SENSOR_MD1_EXISTS  || defined BME688_SENSOR_MD2_EXISTS  || defined  BME688_SENSOR_MD3_EXISTS || defined BME688_SENSOR_MD4_EXISTS )
  #include <Adafruit_Sensor.h>
  #include "Adafruit_BME680.h"
  #define SEALEVELPRESSURE_HPA (1013.25)
  Adafruit_BME680 bme; // I2C
#endif

#if (defined GROVE_GAS_V2_MD5_EXISTS  || defined GROVE_GAS_V2_MD6_EXISTS  || defined  GROVE_GAS_V2_MD7_EXISTS || defined GROVE_GAS_V2_MD8_EXISTS )
    //#include <Multichannel_Gas_GMXXX.h>
    // #include <M_Gas.h>
     #include "M_Gas.h"
    GAS_GMXXX<TwoWire> gas;
#endif


#ifdef LIGHT_SENSOR_EXISTS  
  #include "Adafruit_TSL2591.h"
  Adafruit_TSL2591 tsl = Adafruit_TSL2591(2591); // pass in a number for the sensor identifier (for your use later)
#endif

#ifdef BAR_PRES_SENSOR_EXISTS 
  #include "Adafruit_BMP3XX.h"
  #define SEALEVELPRESSURE_HPA (1013.25)
  Adafruit_BMP3XX bmp; // I2C  //BAROMETRIC PRESSURE
#endif

#ifdef ACCL_GYRO_SENSOR_EXISTS 
  #include "Adafruit_LSM9DS1.h"
  Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1(); 
#endif
// Arduino BH1750FVI Light sensor
//https://github.com/mysensors/MySensorsArduinoExamples/blob/master/examples/LightLuxSensor/LightLuxSensor.ino

void Sensors_PeripInit(void){

  RTC_Init();

  #ifdef TEMP_HUM_ONBOARD_SENSOR_EXISTS
    SensorInit_Si072(SI072_ONBOARD_SENSOR_ADDR); // TEMP HUM
  #else
    Serial.print(F("No OnBoard Sensor !!"));Serial.println(F("SI072_ONBOARD_SENSOR"));
  #endif 

  #ifdef TEMP_HUM_1_SENSOR_EXISTS
    SensorInit_Si072(SI072_FIRST_SENSOR_ADDR); // TEMP HUM
  #else
    //Serial.print(F("No Sensor On Channel!!"));Serial.print(F("SI072_FIRST_SENSOR"));
  #endif 

  #ifdef TEMP_HUM_2_SENSOR_EXISTS
    SensorInit_Si072(SI072_SECOND_SENSOR_ADDR); // TEMP HUM
  #else
   // Serial.print(F("No Sensor On Channel!!"));Serial.print(F("SI072_SECOND_SENSOR"));
  #endif 

  #ifdef TEMP_HUM_3_SENSOR_EXISTS
    SensorInit_Si072(SI072_THIRD_SENSOR_ADDR); // TEMP HUM
  #else
  // Serial.print(F("No Sensor On Channel!!"));Serial.print(F("SI072_THIRD_SENSOR"));
  #endif 



 #ifdef  BME688_SENSOR_MD1_EXISTS
    SensorVoc_BME680_Init(ADR_MD1);   
#endif 
 #ifdef  BME688_SENSOR_MD2_EXISTS
    SensorVoc_BME680_Init(ADR_MD2);   
#endif 
 #ifdef  BME688_SENSOR_MD3_EXISTS
    SensorVoc_BME680_Init(ADR_MD3);   
#endif 
 #ifdef  BME688_SENSOR_MD4_EXISTS
    SensorVoc_BME680_Init(ADR_MD4);   
#endif 

#ifdef GROVE_GAS_V2_MD5_EXISTS
    SensorGroveV2_Init(ADR_MD5);
#endif 
#ifdef GROVE_GAS_V2_MD6_EXISTS
    SensorGroveV2_Init(ADR_MD6);
#endif 
#ifdef GROVE_GAS_V2_MD7_EXISTS
    SensorGroveV2_Init(ADR_MD7);
#endif 
#ifdef GROVE_GAS_V2_MD8_EXISTS
    SensorGroveV2_Init(ADR_MD8);
#endif 
      
  #ifdef BAR_PRES_SENSOR_EXISTS 
     SensorAlt_Init();     //BAROMETRIC PRESSURE
  #else
   // Serial.println(F("No Bar.Pressure_Sensor!!"));
  #endif 
      
  #ifdef LIGHT_SENSOR_EXISTS  
    SensorLight_Init();  // LIGHT 
  #else
   // Serial.println(F("No Light_Sensor!!"));
  #endif
      
  #ifdef ACCL_GYRO_SENSOR_EXISTS 
    SensorACccel_GyroInit(); // ACCEL GYRO  
   #else
  //  Serial.println(F("No Accel Gyro_Sensor!!"));
  #endif  
  #ifdef PM25_DUST_SENSOR_EXISTS 
    Serial1.begin(9600);
    Serial.print(F("PMSensor Serial Port Init "));
   #endif 
}


#if (defined GROVE_GAS_V2_MD5_EXISTS  || defined GROVE_GAS_V2_MD6_EXISTS  || defined  GROVE_GAS_V2_MD7_EXISTS || defined GROVE_GAS_V2_MD8_EXISTS )
void SensorGroveV2_Init(uint8_t Channel){
  tcaselect(Channel);    
  gas.begin(Wire, 0x08); // use the hardware I2C
}
void SensorGroveV2_Read(uint8_t Channel){
   // uint32_t val1,uint32_t val2,uint32_t val3,uint32_t val4,
    uint8_t LineNo;
    tcaselect(Channel);

    switch(Channel){
      #ifdef GROVE_GAS_V2_MD5_EXISTS
      case ADR_MD5 : // Multi_Gas_1 = Multi_Gas;
                      LineNo = ADR_MD5;
      break;
    #endif 
      #ifdef GROVE_GAS_V2_MD6_EXISTS    
      case ADR_MD6 : //Multi_Gas_2 = Multi_Gas;
                     LineNo = ADR_MD6;
      break;
    #endif     
      #ifdef GROVE_GAS_V2_MD7_EXISTS      
      case ADR_MD7 ://Multi_Gas_3 = Multi_Gas;
                     LineNo = ADR_MD7;
      break;
    #endif   
       #ifdef GROVE_GAS_V2_MD8_EXISTS       
      case ADR_MD8 ://Multi_Gas_4 = Multi_Gas;
                     LineNo = ADR_MD8;
      break;
    #endif       
      default:

      break; 
    }
   #ifdef SERIAL_MONITOR_ACTIVE   
    Serial.print(F("Winsen_Ch:"));Serial.print(LineNo);
    #endif 
    Multi_Gas.VOC = gas.measure_VOC(); 
       #ifdef SERIAL_MONITOR_ACTIVE   
    Serial.print(F(" VOC:")); Serial.print(Multi_Gas.VOC); Serial.print(F(" /"));
    Serial.print(gas.calcVol(Multi_Gas.VOC)); Serial.print("V  ");
    #endif 
    Multi_Gas.CO = gas.measure_CO(); 
       #ifdef SERIAL_MONITOR_ACTIVE   
    Serial.print(F(" CO:")); Serial.print(Multi_Gas.CO); Serial.print(F(" /"));
    Serial.print(gas.calcVol(Multi_Gas.CO)); Serial.print("V  ");  
    #endif 
    Multi_Gas.NO2 = gas.measure_NO2(); 
      #ifdef SERIAL_MONITOR_ACTIVE    
    Serial.print(F(" NO2:")); Serial.print(Multi_Gas.NO2); Serial.print(F(" /"));
    Serial.print(gas.calcVol(Multi_Gas.NO2)); Serial.print("V  ");
       #endif  
    Multi_Gas.C2H5OH = gas.measure_C2H5OH(); 
      #ifdef SERIAL_MONITOR_ACTIVE    
    Serial.print(F(" C2H5OH:")); Serial.print(Multi_Gas.C2H5OH); Serial.print(F(" /"));
    Serial.print(gas.calcVol(Multi_Gas.C2H5OH)); Serial.println("V");
        #endif 
    switch(Channel){
      #ifdef GROVE_GAS_V2_MD5_EXISTS
      case ADR_MD5 :  Multi_Gas_1 = Multi_Gas;                  
      break;
    #endif 
      #ifdef GROVE_GAS_V2_MD6_EXISTS    
      case ADR_MD6 : Multi_Gas_2 = Multi_Gas;
      break;
    #endif     
      #ifdef GROVE_GAS_V2_MD7_EXISTS      
      case ADR_MD7 : Multi_Gas_3 = Multi_Gas;
      break;
    #endif   
       #ifdef GROVE_GAS_V2_MD8_EXISTS       
      case ADR_MD8 : Multi_Gas_4 = Multi_Gas;
      break;
    #endif       
      default:

      break; 
    }
    
   
        
}

 #endif 

#if (defined BME688_SENSOR_MD1_EXISTS  || defined BME688_SENSOR_MD2_EXISTS  || defined  BME688_SENSOR_MD3_EXISTS || defined BME688_SENSOR_MD4_EXISTS )
void SensorVoc_BME680_Init(uint8_t Channel){
      tcaselect(Channel);    
      if (bme.begin()) {
       // Set up oversampling and filter initialization
        bme.setTemperatureOversampling(BME680_OS_8X);
        bme.setHumidityOversampling(BME680_OS_2X);
        bme.setPressureOversampling(BME680_OS_4X);
        bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
        bme.setGasHeater(320, 150); // 320*C for 150 ms
      }
      else {
        Serial.print(F("Could not find a valid BME680 sensor On Channel:"));Serial.println(Channel);       
       // while (1);
    }
} 
void SensorVoc_BME680_Read(uint8_t Channel){
    tcaselect(Channel);
    if (! bme.performReading()) {
      Serial.print(F("Failed to perform BME680 reading On Channel :"));     
      Serial.println(Channel);
      switch(Channel){
        case ADR_MD4 :Bosch_BME688_4.Exists = 0;
        break;
        case ADR_MD3 :Bosch_BME688_3.Exists = 0;
        break;
        case ADR_MD2 :Bosch_BME688_2.Exists = 0;
        break;
        case ADR_MD1 :Bosch_BME688_1.Exists = 0;
        break;        
        default:
        break;          
      } 
      return;
    }
    uint8_t LineNo;
    switch(Channel){
      #ifdef BME688_SENSOR_MD4_EXISTS
      case ADR_MD4 : 
        Bosch_BME688_4.Temperature = bme.temperature;
        Bosch_BME688_4.Humidity = bme.humidity;
        Bosch_BME688_4.Pressure = bme.pressure  / 100.0;
        Bosch_BME688_4.Gas = bme.gas_resistance / 1000.0;
        Bosch_BME688_4.Exists = 1;
        LineNo = ADR_MD4;
      break;
    #endif 
      #ifdef BME688_SENSOR_MD3_EXISTS    
      case ADR_MD3 : 
        Bosch_BME688_3.Temperature = bme.temperature;
        Bosch_BME688_3.Humidity = bme.humidity;
        Bosch_BME688_3.Pressure = bme.pressure  / 100.0;
        Bosch_BME688_3.Gas = bme.gas_resistance / 1000.0;
        Bosch_BME688_3.Exists = 1;
        LineNo = ADR_MD3;
      break;
    #endif     
      #ifdef BME688_SENSOR_MD2_EXISTS      
      case ADR_MD2 : 
        Bosch_BME688_2.Temperature = bme.temperature;
        Bosch_BME688_2.Humidity = bme.humidity;
        Bosch_BME688_2.Pressure = bme.pressure  / 100.0;
        Bosch_BME688_2.Gas = bme.gas_resistance / 1000.0;
        Bosch_BME688_2.Exists = 1;
        LineNo = ADR_MD2;
      break;
    #endif   
       #ifdef BME688_SENSOR_MD1_EXISTS       
      case ADR_MD1 : 
        Bosch_BME688_1.Temperature = bme.temperature;
        Bosch_BME688_1.Humidity = bme.humidity;
        Bosch_BME688_1.Pressure = bme.pressure  / 100.0;
        Bosch_BME688_1.Gas = bme.gas_resistance / 1000.0;
        Bosch_BME688_1.Exists = 1;
        LineNo = ADR_MD1;
      break;
    #endif       
      default:

      break; 
    }  

     Values.DAQ_Temperature = (int32_t)roundf(bme.temperature*100);
     Values.DAQ_Humidity = (int32_t)roundf(bme.humidity*100);

   //byte a0,a1,a2,a3;
  //  int32_t TemperatureM,TemperatureE;

//int expon;
//float mant = frexpf(bme.temperature, &expon);

 //float Tempfloat = roundf(bme.temperature*100);
 // int32_t TempInt = (int32_t)Tempfloat;



//printf("%g = %g * 2^%d\n", n, mant, expon);

   // Serial.println(F("BME688 Sensors")); 
   #ifdef SERIAL_MONITOR_ACTIVE    
    Serial.print(F("BME688_Ch:")); Serial.print(LineNo);    
    Serial.print(F(" VOC:")); Serial.print(bme.gas_resistance / 1000.0); Serial.print(F(" KOhms "));  
    Serial.print(F(" Temp:"));Serial.print(bme.temperature); Serial.print(F("*C "));
    Serial.print(F(" Temp:"));Serial.print(Values.DAQ_Temperature); Serial.print(F("*C "));

    Serial.print(F(" Hum:%"));Serial.print(bme.humidity); 
    Serial.print(F(" Hum%:"));Serial.print(Values.DAQ_Humidity); 

    Serial.print(F("  Prs:"));Serial.print(bme.pressure / 100.0); Serial.println(F("hPa"));
  #endif

}

#endif // endof #if (defined BME688_SENSOR_ADR4_EXISTS  || d.....



#ifdef ACCL_GYRO_SENSOR_EXISTS 
void SensorACccel_GyroInit(){
    tcaselect(CHANNEL_ACCEL_GYRO);
    if (!IMU.begin()) {
      Serial.println(F("Failed to initialize IMU!"));
     // while (1);
    }
    else       
      Serial.println(F("Acceloreometer & Gyro Initialized"));
    Serial.print(F("Accelerometer sample rate = "));
    Serial.print(IMU.accelerationSampleRate());
    Serial.println(F(" Hz"));
    Serial.println();
    Serial.println(F("Acceleration in G's"));
    Serial.println(F("X\tY\tZ"));

    Serial.print(F("Gyroscope sample rate = "));
    Serial.print(IMU.gyroscopeSampleRate());
    Serial.println(F(" Hz"));
    Serial.println();
    Serial.println(F("Gyroscope in degrees/second"));
    Serial.println(F("X\tY\tZ"));
  }
void SensorAcccel_GyroRead(){
   tcaselect(CHANNEL_ACCEL_GYRO);
 //  float x, y, z;

  if (IMU.accelerationAvailable()) {
      IMU.readAcceleration(Accelometer.x, Accelometer.y, Accelometer.z);

      Serial.print(Accelometer.x);
      Serial.print('\t');
      Serial.print(Accelometer.y);
      Serial.print('\t');
      Serial.println(Accelometer.z);
  }
    else Serial.println(F("Accelometer Reading Problem"));
  
  if (IMU.gyroscopeAvailable()) {
      IMU.readGyroscope(Gyro.x, Gyro.y, Gyro.z);

      Serial.print(Gyro.x);
      Serial.print('\t');
      Serial.print(Gyro.y);
      Serial.print('\t');
      Serial.println(Gyro.z);
  }
  else Serial.println(F("Gyroscope Reading Problem"));
}
#endif  
#if (defined TEMP_HUM_1_SENSOR_EXISTS  || defined TEMP_HUM_2_SENSOR_EXISTS  || defined  TEMP_HUM_3_SENSOR_EXISTS || defined TEMP_HUM_ONBOARD_SENSOR_EXISTS )
void SensorInit_Si072(uint8_t Channel){
  if(Channel == NO_IC2_MULTIPLEXER){
     Serial.print(F("No Sensor_Channel:"));
    return;
  }
  tcaselect(Channel);
  Serial.print(F("Sensor_Channel:"));Serial.println(Channel);
  if (!THsensor.begin()) {
    Serial.print(F("No Si7021 sensor On Channel: "));Serial.println(Channel);
    return;
  }
    Serial.println(F(" Si7021 sensor found!"));
    Serial.print(F("Revision of Th sensor!"));Serial.println(THsensor.getRevision());
    
 //    Serial.print(" Serial "); //Serial.print(THsensor.sernum_a, HEX); Serial.println(THsensor.sernum_b, HEX);
  //   Serial.print(F("Sensor_Channel:"));Serial.println(Channel);

    uint32_t Id = THsensor.sernum_a;
    Serial.print(F("Current Sensor Id:"));Serial.println(Id,HEX); 
    switch(Channel){
      case SI072_ONBOARD_SENSOR_ADDR: SensorId.OnBoard = Id;     
      break;     
      case SI072_FIRST_SENSOR_ADDR:   SensorId.No1 = Id;     
      break;
      case SI072_SECOND_SENSOR_ADDR:  SensorId.No2 = Id;         
      break;
      case SI072_THIRD_SENSOR_ADDR:   SensorId.No3 = Id;                
      break; 
      default: Serial.print(F("error"));
      break;
    }  
}
/*
void GerSerialNo(void){
      Serial.print(F(" Rev("));
    Serial.print(THsensor.getRevision());
    Serial.print(")");
    Serial.print(F(" Serial #")); Serial.print(THsensor.sernum_a, HEX); Serial.println(THsensor.sernum_b, HEX);
}
*/

void SensorRead_Si072(unsigned char Channel){
    if(Channel == NO_IC2_MULTIPLEXER){
      Serial.println(F("Not Defined Any Sensor!"));
      return;
    } 
    tcaselect(Channel);
    float Temperature = THsensor.readTemperature(); 
    float Humidity = THsensor.readHumidity(); 
    /*
    Serial.print(F("Channel->"));Serial.print(Channel);
    Serial.print(F("    Temperature:"));Serial.print(Temperature);Serial.print(F("'C     Humidity:%"));Serial.println(Humidity);   
    */
    switch(Channel){
     case SI072_ONBOARD_SENSOR_ADDR:
        Values.Temperature_OnBoard = Temperature;
        Values.Humidity_OnBoard = Humidity; 
      break;       
      case SI072_FIRST_SENSOR_ADDR: 
        Values.Temperature_Ch1 = Temperature;
        Values.Humidity_Ch1 = Humidity;        
       break;
      case SI072_SECOND_SENSOR_ADDR: 
        Values.Temperature_Ch2 = Temperature;      
        Values.Humidity_Ch2 = Humidity;
       break;
      case SI072_THIRD_SENSOR_ADDR: 
        Values.Temperature_Ch3 = Temperature;      
        Values.Humidity_Ch3 = Humidity;
       break; 
       default:
        Serial.print(F("error"));
       break;
    }
 /*
    switch(Channel){
      case SI072_ONBOARD_SENSOR_ADDR:
       
      break;        
      case SI072_FIRST_SENSOR_ADDR: 
        Values.Temperature_Ch1 = THsensor.readTemperature();
   //     Serial.println(Values.Temperature_Ch1, 2);
       break;
      case SI072_SECOND_SENSOR_ADDR: 
        Values.Temperature_Ch2 = THsensor.readTemperature();
    //    Serial.println(Values.Temperature_Ch2, 2);
       break;
      case SI072_THIRD_SENSOR_ADDR: 
        Values.Temperature_Ch3 = THsensor.readTemperature();
   //     Serial.println(Values.Temperature_Ch3, 2);
       break; 
       default:
        Serial.print(F("error"));
       break;
    } 
    */
}
#endif
#ifdef BAR_PRES_SENSOR_EXISTS 
void SensorAlt_Init() {
  tcaselect(CHANNEL_BAROMETRIC);
  Serial.println(F("BMP388 test"));
  if (!bmp.begin()) {
    Serial.println(F("Could not find a valid BMP3 sensor, check wiring!"));
    //while (1);
  }
  else {
      Serial.println(F(" BMP388 sensor Valid!"));
      // Set up oversampling and filter initialization
      bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
      bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
      bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
      //bmp.setOutputDataRate(BMP3_ODR_50_HZ);
  }
}

void SensorAlt_Read(){
    tcaselect(CHANNEL_BAROMETRIC);
    if (! bmp.performReading()) {
    Serial.println(F("Failed to perform reading :("));
   // return;
  }
  else{
    Serial.print(F("Temperature = "));
    Values.TemperatureBMP = bmp.temperature;
    Serial.print(Values.TemperatureBMP);
    Serial.print(F(" *C"));
    Serial.print(F("  Pressure = "));
    Values.Pressure = bmp.pressure / 100.0;
    Serial.print(Values.Pressure);
    Serial.println(F(" hPa"));
    Serial.print(F("Altitude = "));
    Values.Altitude = bmp.readAltitude(SEALEVELPRESSURE_HPA);
    Serial.print(Values.Altitude);
    Serial.println(F(" m"));
  }
}
#endif
#ifdef LIGHT_SENSOR_EXISTS
void displaySensorDetails(void){
  sensor_t sensor;
  tsl.getSensor(&sensor);
  Serial.println(F("------------------------------------"));
  Serial.print  (F("Sensor:       ")); Serial.println(sensor.name);
  Serial.print  (F("Driver Ver:   ")); Serial.println(sensor.version);
  Serial.print  (F("Unique ID:    ")); Serial.println(sensor.sensor_id);
  Serial.print  (F("Max Value:    ")); Serial.print(sensor.max_value); Serial.println(F(" lux"));
  Serial.print  (F("Min Value:    ")); Serial.print(sensor.min_value); Serial.println(F(" lux"));
  Serial.print  (F("Resolution:   ")); Serial.print(sensor.resolution, 4); Serial.println(F(" lux"));  
  Serial.println(F("------------------------------------"));
  Serial.println(F(""));
 // delay(500);
}
void configureSensor(void){
  // You can change the gain on the fly, to adapt to brighter/dimmer light situations
  //tsl.setGain(TSL2591_GAIN_LOW);    // 1x gain (bright light)
  tsl.setGain(TSL2591_GAIN_MED);      // 25x gain
  //tsl.setGain(TSL2591_GAIN_HIGH);   // 428x gain
  
  // Changing the integration time gives you a longer time over which to sense light
  // longer timelines are slower, but are good in very low light situtations!
  //tsl.setTiming(TSL2591_INTEGRATIONTIME_100MS);  // shortest integration time (bright light)
  // tsl.setTiming(TSL2591_INTEGRATIONTIME_200MS);
  tsl.setTiming(TSL2591_INTEGRATIONTIME_300MS);
  // tsl.setTiming(TSL2591_INTEGRATIONTIME_400MS);
  // tsl.setTiming(TSL2591_INTEGRATIONTIME_500MS);
  // tsl.setTiming(TSL2591_INTEGRATIONTIME_600MS);  // longest integration time (dim light)

  /* Display the gain and integration time for reference sake */  
  Serial.println(F("------------------------------------"));
  Serial.print  (F("Gain:         "));
  tsl2591Gain_t gain = tsl.getGain();
  switch(gain){
    case TSL2591_GAIN_LOW:
      Serial.println(F("1x (Low)"));
      break;
    case TSL2591_GAIN_MED:
      Serial.println(F("25x (Medium)"));
      break;
    case TSL2591_GAIN_HIGH:
      Serial.println(F("428x (High)"));
      break;
    case TSL2591_GAIN_MAX:
      Serial.println(F("9876x (Max)"));
      break;
  }
  Serial.print  (F("Timing:       "));
  Serial.print((tsl.getTiming() + 1) * 100, DEC); 
  Serial.println(F(" ms"));
  Serial.println(F("------------------------------------"));
  Serial.println(F(""));
}
void SensorLight_Init(void) {
  tcaselect(CHANNEL_LIGHT);
  Serial.println(F("Starting Adafruit TSL2591 Test!"));
  if (tsl.begin()) {
    Serial.println(F("Found a TSL2591 sensor"));
  } 
  else {
    Serial.println(F("No sensor found ... check your wiring?"));
   // while (1);
  }    
  /* Display some basic information on this sensor */
  displaySensorDetails();  
  /* Configure the sensor */
  configureSensor();
  // Now we're ready to get readings ... move on to loop()!
}
void SensorLight_Read(void) {
    tcaselect(CHANNEL_LIGHT);
    // Simple data read example. Just read the infrared, fullspecrtrum diode 
  // or 'visible' (difference between the two) channels.
  // This can take 100-600 milliseconds! Uncomment whichever of the following you want to read
//  uint16_t x = tsl.getLuminosity(TSL2591_VISIBLE);
  Values.Luminosity = tsl.getLuminosity(TSL2591_VISIBLE);
  //uint16_t x = tsl.getLuminosity(TSL2591_FULLSPECTRUM);
  //uint16_t x = tsl.getLuminosity(TSL2591_INFRARED);

//  Serial.print(F("[ ")); Serial.print(millis()); Serial.print(F(" ms ] "));
  Serial.print(F("Luminosity: "));
  Serial.println(Values.Luminosity, DEC);
}
#endif

#ifdef WIND_SENSOR_EXISTS
void WindSpeed_Calculation(){
    tempReading = (double)Values.TemperatureSi072;
    // calculate wind speed 
 //   #define analogPinForRV 1 // Analog pin0 for mega 
    RV_ADunits = analogRead(RV_PINOUT);      // read analog input of wind sensor RV output
    for (int i = 0; i < 7; i++) {                 // read another 7 times to improve reading quality and reduce noise
      RV_ADunits += analogRead(RV_PINOUT);
    }
    RV_ADunits /= 8;                              // devide by 8, to get average
    RV_ADunits_dev1000 = RV_ADunits / 1000;       // devide by 1000, to match calibration factors and calculation
    Values.WindRaw = RV_ADunits_dev1000;
    velReading = (2.285 * pow(RV_ADunits_dev1000, 3) - 12.417 * pow(RV_ADunits_dev1000, 2) 
      + 22.831 * RV_ADunits_dev1000 - 14.19) / (T_rev_wind / tempReading) * v_calc_F4 
      - ((T_rev_wind - tempReading) * v_calc_F1 * pow(RV_ADunits_dev1000, v_calc_F3)) 
      - (T_rev_wind - tempReading) * v_calc_F2 ; // wind speed calculation based on own calibration - see reference on authors' website
    velReading = velReading - zeroWindAdjustment; // adjust zero windspeed
    if (velReading < 0) {
      velReading = 0; // error handling in case of false zeroWindAdjustment setting
    }
}
 
 //  This Part is based on the Sensors Manufacturer Data sheet
  //  Values.WindMPH
  // #define WND_OUT_Pin  0;// A0;   // wind sensor analog pin  hooked up to Wind P sensor "OUT" pin
  //  Values.WindTemp  
  // #define WND_TEMP_Pin 1;// A1;   // temp sesnsor analog pin hooked up to Wind P sensor "TMP" pin

void WinSensor(){
        int windADunits = analogRead(WND_OUT_Pin);
    // Serial.print("RW ");   // print raw A/D for debug
    // Serial.print(windADunits);
    // Serial.print("\t");
    if (windADunits < 350 ) windADunits = 350; // min 350 for 
  
    Serial.print(F("windADunits"));Serial.println(windADunits);
    Values.WindMPH =  pow((((float)windADunits - 264.0) / 85.6814), 3.36814);
  //  float windMPH =  pow((((float)windADunits - 264.0) / 85.6814), 3.36814);
   // Serial.print(windMPH);
  //  Serial.print(" MPH\t");    

    // temp routine and print raw and temp C
    int tempRawAD = analogRead(WND_TEMP_Pin);  
 //   Serial.print("tempRawAD");Serial.println(tempRawAD); 
    // Serial.print("RT ");    // print raw A/D for debug
    // Serial.print(tempRawAD);
    // Serial.print("\t");
    
    // convert to volts then use formula from datatsheet 
    // Vout = ( TempC * .0195 ) + .400
    // tempC = (Vout - V0c) / TC   see the MCP9701 datasheet for V0c and TC
    
   // float tempC = ((((float)tempRawAD * 5.0) / 1024.0) - 0.400) / .0195; 
    Values.WindTemp = ((((float)tempRawAD * 5.0) / 1024.0) - 0.400) / .0195; 
 //   Serial.print(tempC);
 //   Serial.println(" C");
    //delay(750);
}
#endif

#ifdef VOLTAGE_MEASURE_EXISTS
void CurrentVolt_Read() {
  // https://www.onlinegdb.com/edit/Hkmlxi_08

 
#ifdef ARDUINO_MEGA
  ADCSRA |= (1 << ADEN); // enable adc
#endif
  delay(1);
  Current_Mains_Raw = analogRead(4);
  //  Current_Mains_Raw_Trim = Current_Mains_Raw;
  //  delay(1);
  //  239vac -> 4.94vdc
  // 242  vac  5vdc  242/1024 = 0.2363281

  //  238vac -> 3.10vdc  // 33K/8K2
  // 383  vac  5vdc  383/1024 = 0.374874

  CurrentArray[CurrentIndexer] = Current_Mains_Raw;
  CurrentIndexer++;
  if (CurrentIndexer >= 10) {
    CurrentIndexer = 0;
    unsigned int Cumulative = 0;
    for (unsigned int i = 0; i < 10; i++) {
      Cumulative += CurrentArray[i];
    }
#define Volt_Coeff 4.8828125 // rms 5v
#define PeakToRms  1.4142135
    // #define Rms_Coeff  6.90533966  // Volt_Coeff *  PeakToRms
#define Rms_Coeff  4.8828125  // Volt_Coeff *  PeakToRms       
    /*
        Current_MainsAverage = ((Cumulative%10) * Volt_Coeff )/10000;
       // then the decimal part
       Current_MainsAverage += ((float)(Cumulative/10) * Volt_Coeff )/1000;
    */
    Current_Mains_Rms = ((Cumulative % 10) * Rms_Coeff ) / 10000;
    // then the decimal part
    Current_Mains_Rms += ((float)(Cumulative / 10) * Rms_Coeff ) / 1000;

    //   Current_Mains_Rms = Current_MainsAverage * 1.414213;
    //   Current_Mains_Av = Current_Mains_Rms / 1,11;
    //  Current_MainsAverage = ((float)Cumulative * 5.044)/1000; // 2K2 // 4.7K // 2K2
    //   Current_MainsAverage = ((float)Cumulative * 9.766)/1000;  //direct
  }

  //  Current_Mains = ((float)Current_Mains_Raw * Rms_Coeff)/1000;
  // Current_Mains *= PeakToRms;

  //   Current_Mains = ((float)Current_Mains_Raw * 5.044)/1000;
  //   Current_Mains = ((float)Current_Mains_Raw * 9.766)/1000; // direct no voltage divider
  Mains_Volt_Raw =  analogRead(1);
  Mains_Volt =   (unsigned int)((float)Mains_Volt_Raw * 0.374874);

#ifdef ARDUINO_MEGA
  ADCSRA &= ~ (1 << ADEN);            // turn off ADC
#endif

}
#endif 

void SDS_DustSensor(void) {
    #ifdef PM25_DUST_SENSOR_EXISTS
      SerialPortPMSensor();
/*
 // PmResult pm = sds.queryPm();// queay mode
  
     PmResult pm = sds.readPm();// active mode
  if (pm.isOk()) {
    Values.PM25 = pm.pm25;
    Values.PM10 = pm.pm10;
    if (Values.PM25 >= 250.0)Values.PM25 = 250.0;
    if (Values.PM10 >= 250.0)Values.PM25 = 250.0;

    // if you want to just print the measured values, you can use toString() method as well
      Serial.println(pm.toString());
    
  } else {
    Serial.print("Pm2.5 Problem: ");
 //   Serial.println(pm.statusToString());
  }
   */ 
#endif

}
void tcaselect(uint8_t i) {
 // if (i > 7) return;
  Wire.beginTransmission(TCAADDR);
    //  https://learn.adafruit.com/adafruit-tca9548a-1-to-8-i2c-multiplexer-breakout
    Wire.write(1 << i);
    Wire.endTransmission();
    delay(2);
}
/*
void I2_ACK_TimeoutSet(void){
  I2CSet =ON;
}

void I2_ACK_TimeoutReset(void){
   I2CSet = OFF;
   I2CTimer = ACK_TIMEOUT;
}

void I2_ACK_Reset(void){
    if(!I2CSet)return;
    if(I2CTimer){
      I2CTimer--;
      return;
    } 
     pinMode(I2C_TIMEOUT, OUTPUT);  // 
     digitalWrite(I2C_TIMEOUT, LOW); 
     delay(1);
     I2CTimer = ACK_TIMEOUT;
     I2CSet = OFF;
     pinMode(I2C_TIMEOUT, INPUT);  // 
     I2Error = ON;
}
*/
#ifdef PM25_DUST_SENSOR_EXISTS  
void PrintPMValues(uint8_t PMError, uint8_t PMCount){
  /*
      Serial.println();
      for(int i=0; i<BUF_LENGTH;i++){  
          Serial.print(i); Serial.print('.'); Serial.print(PMBuffer[i]);Serial.print("    ");
          if(i==9)Serial.println();  
          if(i==19)Serial.println();  
          if(i==29)Serial.println();  
          if(i==39)Serial.println(); 
          if(i==49)Serial.println();       
      }
      */
     Serial.println();  
      Serial.print(F("PMCount "));Serial.println(PMCount);
      Serial.print(F("PM25 "));Serial.println(Values.PM25);
      Serial.print(F("PM10 "));Serial.println(Values.PM10);
      Serial.print(F("PMError "));Serial.println(PMError);         
}
 void SerialPortPMSensor() {   
    if (Serial1.available()) {
    //  uint8_t Rx;
      uint8_t RxCount=0;
      uint16_t PM25Val=0;
      uint16_t PM10Val=0;
      uint8_t PMError = 0; 
      uint8_t PMCount = 0; 
      unsigned char checksum = 32; 
      
        for(int i=0; i<BUF_LENGTH;i++){   
          PMBuffer[i] = 0;
        }   
        while (Serial1.available()) {
            uint8_t Rx = Serial1.read();
            if(RxCount < BUF_LENGTH)PMBuffer[RxCount] =  Rx; 
            RxCount++;      
        }
        Serial1.flush();      
        for(int i=0; i<BUF_LENGTH;i++){
            if(PMBuffer[i] == 0XAA){
              if(PMBuffer[i+1] == 0XC0){
                    checksum=(unsigned char)(PMBuffer[i+2]+PMBuffer[i+3]+PMBuffer[i+4]+PMBuffer[i+5]+ PMBuffer[i+6]+PMBuffer[i+7]);                         
                    if(checksum == PMBuffer[i+8]){ 
                        PMCount++;  
                        PM25Val += (256 * PMBuffer[i+3])+PMBuffer[i+2];
                        PM10Val += (256 * PMBuffer[i+5])+PMBuffer[i+4];                                  
                    }
                    else PMError++;  
              }
          }       
      }
      if(PMCount){
          Values.PM25 =  (float)(PM25Val / PMCount)/10;
          Values.PM10 =  (float)(PM10Val / PMCount)/10;
          if(Values.PM25 >= 250)Values.PM25 = 250;
          if(Values.PM10 >= 250)Values.PM25 = 250;       
      }
      else{
          Values.PM25 =  0;
          Values.PM10 =  0;   
      }
      PrintPMValues(PMError,PMCount);  
    }
}
 #endif 

void WindSensorRead() {
  /*
     #ifdef ARDUINO_MEGA
    ADCSRA |= (1 << ADEN); // enable adc
         #endif
    delay(1);
    //  Values.WindRaw = analogRead(2);delay(1);
    Values.WindMPH = analogRead(OUT_PINOUT);delay(1);
    Values.WindTemp = analogRead(TMP_PINOUT);
    WindSpeed_Calculation();
    #ifdef ARDUINO_MEGA
    ADCSRA &= ~ (1 << ADEN);            // turn off ADC
    #endif
  */
}

    
