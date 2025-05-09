
// 001BC5067010E312
/*
typedef  byte               uint8;
typedef  signed char        int8;
typedef  unsigned char      uint8;
//typedef  signed short       int16;
//typedef  unsigned short     uint16;
typedef  signed int         int16;
typedef  unsigned int       uint16;
typedef  signed long        int32;
typedef  unsigned long      uint32;
typedef  signed long long   int64;
typedef  unsigned long long uint64
C:\Program Files (x86)\Arduino\libraries
*/
#define INTERRUPT_1MSEC

#ifdef INTERRUPT_1MSEC
  #define  TIMER_INT_VAL  49536
  #define  TIMER_INT_PRESCALE  0x00000001 // prescale 1
  #define  SCALE_10MSEC  10
  #define  SCALE_100MSEC  100
  #define  SCALE_250MSEC  250
  #define  SCALE_500MSEC 500
  #define  SCALE_1SEC 1000
  #define  SCALE_2SEC 2000
  #define  SCALE_5SEC 5000
  #define  SCALE_10SEC 10000
  #define  SCALE_20SEC 20000
  #define  SCALE_60SEC 60000
#endif
#ifdef INTERRUPT_4MSEC
  #define  TIMER_INT_VAL  1536
  #define  TIMER_INT_PRESCALE  0x00000001 // prescale 1
  #define  SCALE_10MSEC  2  
  #define  SCALE_100MSEC  25
  #define  SCALE_250MSEC  62 // 62,5
  #define  SCALE_500MSEC 125
  #define  SCALE_1SEC 250
  #define  SCALE_2SEC 500
  #define  SCALE_5SEC 1250
  #define  SCALE_10SEC 2500
  #define  SCALE_20SEC 5000
  #define  SCALE_60SEC 15000
#endif
#ifdef INTERRUPT_10MSEC
  #define  TIMER_INT_VAL  64911
  #define  TIMER_INT_PRESCALE  0x00000100 // prescale 256
  #define  SCALE_10MSEC  1 
  #define  SCALE_100MSEC  10
  #define  SCALE_250MSEC  25
  #define  SCALE_500MSEC 50
  #define  SCALE_1SEC 100
  #define  SCALE_2SEC 200
  #define  SCALE_5SEC 500
  #define  SCALE_10SEC 1000
  #define  SCALE_20SEC 2000
  #define  SCALE_60SEC 6000
#endif

#define DIPSWITCH_1 13
#define DIPSWITCH_2 12

#define DOWNROLL 0
#define UPROLL 1
#define TEMP_HUM_ONBOARD_SENSOR_EXISTS
//#define TEMP_HUM_1_SENSOR_EXISTS
//#define TEMP_HUM_2_SENSOR_EXISTS
//#define TEMP_HUM_3_SENSOR_EXISTS  

//#define PM25_DUST_SENSOR_EXISTS
//#define SDCARD_EXISTS
#define OLEDDISPLAY_EXISTS
//#define ENERGYMETER_EXISTS 
//#define PROGRELAY_EXISTS 

//#define SERIAL_MONITOR_ACTIVE  
#define ACKNOWLEDGE  0XC3//26 
#define DATA_BYTE    0X55
#define  PREAMBLE_BYTES  2 // preamble+length 4 byte + 2 byte
#define DATALENGTH_BYTES  2
#define CRC_BYTES 2
#define DEFAULT_PREAMBLE = 0XAAAA;
#define DEFAULT_CRC_INIT 0XAAAA; // preamble+length 4 byte + 4 byte
  #define DEFAULT_TX_LENGTH  136//26 // The whole packet including preamble and CRC
  #define  DEFAULT_RX_LENGTH 30
 #define  ATMEGA32_BUFFER_SIZE 64


/*
                                  // ON PCB DESIGNATOR
#define BME688_SENSOR_ADR7_EXISTS // MD1
#define BME688_SENSOR_ADR6_EXISTS // MD2
#define BME688_SENSOR_ADR0_EXISTS // MD3
#define BME688_SENSOR_ADR1_EXISTS // MD4

#define GROVE_GAS_V2_ADR3_EXISTS // MD5
#define GROVE_GAS_V2_ADR2_EXISTS // MD6
#define GROVE_GAS_V2_ADR5_EXISTS // MD7
#define GROVE_GAS_V2_ADR4_EXISTS // MD8
*/
#define BME688_SENSOR_MD1_EXISTS // ADR7
#define BME688_SENSOR_MD2_EXISTS // ADR6
#define BME688_SENSOR_MD3_EXISTS // ADR0
#define BME688_SENSOR_MD4_EXISTS // ADR1

#define GROVE_GAS_V2_MD5_EXISTS // ADR3
#define GROVE_GAS_V2_MD6_EXISTS // ADR2
#define GROVE_GAS_V2_MD7_EXISTS // ADR5
#define GROVE_GAS_V2_MD8_EXISTS // ADR4

#define ADR_MD1 7  //PCB ADRESS MATCHING
#define ADR_MD2 6
#define ADR_MD3 0
#define ADR_MD4 1

#define ADR_MD5 3
#define ADR_MD6 2
#define ADR_MD7 5
#define ADR_MD8 4


//#define LIGHT_SENSOR_EXISTS  
//#define BAR_PRES_SENSOR_EXISTS  
//#define ACCL_GYRO_SENSOR_EXISTS  
//#define WIND_SENSOR_EXISTS  
//#define LEM_CURRENT_EXISTS
//#define VOLTAGE_MEASURE_EXISTS

//#define KEY_DIGITAL
#define KEY_ANALOG

#define AD9153_PROTOTYPE  // AD9153 Power Monitoring IC Related IOs

 // #define DEBUG_SIMULATOR_MODE // For DEbugging As A Simulator
// Select Hardware Type
//#define FIRST_PROTOTYPE  // with LEM current Transdcucer

#define ARM_MATH_CM0PLUS

//#define RELAY_OUT_1 23
//#define RELAY_OUT_2 53

//#include "SdsDustSensor.h" // https://github.com/lewapek/sds-dust-sensors-arduino-library
/*
 #define SI072_FIRST_SENSOR 7  // multiplexer Channel 7 first blu box prot
 #define SI072_SECOND_SENSOR 1 // first prot  0      0
 #define SI072_THIRD_SENSOR 2 // sec1                2 
*/
/*
 // first ADE9153 prototypes
 #define SI072_FIRST_SENSOR 2  // multiplexer Channel 7 first blu box prot
 #define SI072_SECOND_SENSOR 3 // first prot  0      0
 #define SI072_THIRD_SENSOR 4 // sec1                2 
*/

#define POWERIC_SETUP1 0  // Init
#define POWERIC_SETUP2 1  // Delay
#define POWERIC_SETUP3 2 // End
#define POWERIC_NORMAL 3
#define POWERIC_CALB1  4
#define POWERIC_CALB2  5 
#define POWERIC_CALB3  6
#define POWERIC_CALB4  7
#define POWERIC_CALB5  8
#define POWERIC_CALB6  9
#define POWERIC_CALB7  10
#define POWERIC_CALB8  11
#define POWERIC_CALB9  12
#define POWERIC_CALB10 13

 #define SI072_ONBOARD_SENSOR_ADDR 1  // multiplexer Channel 7 first blu box prot
 #define SI072_FIRST_SENSOR_ADDR 7  // multiplexer Channel 7 first blu box prot
 #define SI072_SECOND_SENSOR_ADDR 3 // first prot  0      0
 #define SI072_THIRD_SENSOR_ADDR 2 // sec1                2 

#define NO_IC2_MULTIPLEXER 16
#define DEBUG_KEY

#define ON 1 //
#define OFF 0 //


#ifdef FIRST_PROTOTYPE
  //  const int chipSelect = 10; // mega SS for SD Card  
  #define KEY_UP 5 // 12//4 //RED
  #define KEY_RIGHT 2//12//2 //
  #define LED_GREEN 3// 11//3 // GREEN
  #define LED_RED 4 // 12//4 //RED
  #define KEY_MID 5// 11//5 //
  #define KEY_LEFT 6//13//6 // ok
#endif

  #define TDI         A7 //9 Atmel ice JTAG  // 0 2 GND
  #define TDO         A6 //3 Atmel ice JTAG  // 4 Vdd -> 5V mega 3V3 Due
  #define TMS         A5 //5 Atmel ice JTAG  // 6 Reset
  #define TCK         A4 //1 Atmel ice JTAG
  
  #define DEBUG_OUT      A4  
  #define ANALOG3        A3
  #define ANALOG2         A2
  #define ANALOG1         A1
  #define KEY_ANALOG_IN   A0  
     
    #if  defined KEY_DIGITAL
  #define KEY_RIGHT       13
  #define KEY_DOWN        12
  #define KEY_LEFT        11
  #define KEY_UP          12    
    #endif

#if defined (ARDUINO_MEGA)  | defined (ARDUINO_DUE) 
   //   const int chipSelect = 10;
   //   const int SD_CS_PINOUT = 10;
      #define SD_CS_PINOUT    10  
#endif

  #define LED_RED        9  // L1
    #ifdef ENERGYMETER_EXISTS
  #define ADE9153A_CS_PIN  8 
      #endif      
  #define  LED_GREEN         7 //L2
#ifdef ENERGYMETER_EXISTS 
  #define ADE9153A_RED_LED 6                 //On-board LED pin 
  #define ADE9153A_CALB_BUTTON   5         
  #define ADE9153A_RESET_PIN     4  
  #define ADE9153A_IRQ_PIN       3
  #define ADE9153A_ZX_DREADY_PIN 2
#endif
  #define TX_OUTPUT_PIN          1 //ON BOARD PROGRAMMING & DEBUG RESERVED
  #define RX_INPUT_PIN           0  //ON BOARD PROGRAMMING & DEBUG RESERVED

  #define OUT_PINOUT 2 // Out pin of the sensor
  #define RV_PINOUT 1 // RV output of the sensor
  #define TMP_PINOUT 0 // analogRead(TMP_PINOUT);

#define MAX_DISPLAY_CHAR 21
#define  MAXSHOWLINE 6  // define how many lines for sensorts to show including fw info line 

#define DISPSHOWLINE4
#define DISPSHOWLINE3
#define DISPSHOWLINE2
#define DISPSHOWLINE1

#define DISPROLL_LINE0 0
#define DISPROLL_LINE1 1
#define DISPROLL_LINE2 2
#define DISPROLL_LINE3 3
#define DISPROLL_LINE4 4
#define DISPROLL_LINE5 5
#define DISPROLL_LINE6 6
#define DISPROLL_LINE7 7
#define DISPROLL_LINE8 8
#define DISPROLL_LINE9 9

#define MENU_NULL 0
#define MENU1   16
#define MENU2   32
#define MENU3   48
#define MENU4   64
#define MENU5   80
#define MENU6   96

#define MENU1_SUB1 17 // +=4
#define MENU1_SUB2 18
#define MENU1_SUB3 19 // +=4
#define MENU1_SUB4 20

#define MENU2_SUB1  33  // +=4
#define MENU2_SUB2  34
#define MENU2_SUB3  35
#define MENU2_SUB4  36
#define MENU2_SUB5  37
#define MENU2_SUB6  38
#define MENU2_SUB7  39
#define MENU2_SUB8  40

#define MENU3_SUB1  49 // +=4
#define MENU3_SUB2  50
#define MENU3_SUB3  51 // +=4
#define MENU3_SUB4  52

#define MENU4_SUB1 65
#define MENU4_SUB2 66
#define MENU4_SUB3 67

#define MENU5_SUB1 81
#define MENU5_SUB2 82
#define MENU5_SUB3 83
#define MENU5_SUB4 84
#define MENU5_SUB5 85
#define MENU5_SUB6 86
#define MENU5_SUB7 87
#define MENU5_SUB8 88

#define MENU6_SUB1 97
#define MENU6_SUB2 98
#define MENU6_SUB3 99

#define KEYDISP_TIMER 40

#define SD_NOT_Present 0
#define SD1_TYPE 1
#define SD2_TYPE 2
#define SDHC_TYPE 3
#define UNKNOWN_TYPE 4

#if defined (ARDUINO_MEGA)  & defined (ARDUINO_DUE) 
    #error Select Only One Platform-> ARDUINO_MEGA or ARDUINO_DUE
#endif

 #if !(!defined (ARDUINO_MEGA) ^ !defined (ARDUINO_DUE)) 
    #error Select At Least One Platform -> ARDUINO_MEGA or ARDUINO_DUE
#endif

#if defined (KEY_DIGITAL)  & defined (KEY_ANALOG) 
    #error Select Only One Type -> KEY_DIGITAL or KEY_ANALOG
#endif

 #if !(!defined (KEY_DIGITAL) ^ !defined (KEY_ANALOG)) 
    #error Select At Least One Type -> KEY_DIGITAL or KEY_ANALOG
#endif

// function prototypes
void Common_Loop(); 
void ResetCasePrint();
void IO_Settings();
void MicroInit(void);
void Display_ReInit_Start(uint8_t Timer);
void Display_ReInit_End(void);

void  UI_SerialPort_Receive(void);

void SD_Card_Info(void);
void SD_Card_Init(void);
void SD_Card_Data_Preparation(void);
void SD_Card_Header_Preparation(void);

void RTC_Init();
void SensorInit_Si072(uint8_t);
void SensorAlt_Init();
void SensorLight_Init();
void SensorACccel_GyroInit();
void Sensors_PeripInit();
void SensorVoc_BME680_Init(uint8_t Channel);
void SensorVoc_BME680_Read(uint8_t Channel);
void SensorGroveV2_Init(uint8_t Channel);
void SensorGroveV2_Read(uint8_t Channel);


void CurrentVolt_Read();
void AdcRead();
void WindSensorRead();
void SensorRead_Si072(unsigned char);
void SensorAlt_Read();
void SensorLight_Read();
void SensorAcccel_GyroRead();
void SDS_DustSensor(void);
void UpdateSensorInfo(void);

void UpdateInfoLine();
void UpdateDisplayMenu();
void UpdateSD_LogTime();
void UpdateFileSize();
void ConvertFileSize(uint32_t);// Line3  
void UpdateProperLine(uint8_t Index, uint8_t Line);

void EscMenuKey(void);
void EnterMenuKey(void);
void DownMenuKey(void);
void UpMenuKey(void);
void SetSampling(uint16_t Time);
void DispEnable(bool Enable, uint8_t Timer);
void DispEnable_4SD_Prblm(bool Enable, uint8_t Timer);

void  DispExtTimeout(void);
void   DisplayMenu(void);
void KeyTimeOutCheck(void);
void SD_CardLogTask(void);
void SD_Log_File(void);
void SD_Info_Only(void);
void DisplayFullSensors(void);
void DisplayTestDevices(void);
void SerialPortRx(void);
void UpdateDispRoll(uint8_t);
void Log_Data_Write_SD(void);

void Parse_FileString(void);
void Relay_loop(void) ;
float GetValue(uint8_t Relay);
String LimitCopyDisplayStr(String str, uint8_t MaxNumber);
void EnergyMeterIC_Operation(void);
void I2_ACK_Reset(void);
void tcaselect(uint8_t i);

void SetResetLog(bool Enable);
void NVRam_Write_LogStatus(bool Mode);
void NVRam_Read_SampleTime(void);
void NVRam_Write_SampleTime(uint8_t Sample);
void NVRam_Read_Standbye(void);
void NVRam_Write_Standbye(bool Mode);
void NVRam_Read_SerNo(void);
void NVRam_Write_SerNo(char* p);
void NVRam_Read_QueNo(void);
void NVRam_Write_QueNo(char* p);

void UpdateLogFileId(void);
char* CopyFlashToRam(const char* );

void Due_Memory();
void Print_ARM_SPI_Regs(void);

/*
C:\Users\ilker\Documents\Atmel Studio\7.0\ArduinoSketch6\ArduinoSketch6\ArduinoCore\src\libraries\SD\utility\Sd2Card.cpp 
 // send command and return error code.  Return zero for OK
uint8_t Sd2Card::cardCommand(uint8_t cmd, uint32_t arg) {
  chipSelectLow();


  static uint8_t chip_select_asserted = 0;

void Sd2Card::chipSelectHigh(void) {
  digitalWrite(chipSelectPin_, HIGH);
  #ifdef USE_SPI_LIB
  if (chip_select_asserted) {
    chip_select_asserted = 0;
    SDCARD_SPI.endTransaction();
  }
  #endif
}
//------------------------------------------------------------------------------
void Sd2Card::chipSelectLow(void) {
  #ifdef USE_SPI_LIB
  if (!chip_select_asserted) {
    chip_select_asserted = 1;
    SDCARD_SPI.beginTransaction(settings);
  }
  #endif
  digitalWrite(chipSelectPin_, LOW);
}


 * /
 */
