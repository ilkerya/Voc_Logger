void Set_Duty_Cycle(){
  if(Duty_Cycle > 97)Duty_Cycle = 97;
  if(Duty_Cycle < 3)Duty_Cycle = 3;

    OCR3A = (100 -  Duty_Cycle)*3;
  if(OCR3A >= 300) OCR3A = 0;

}
void Set_FanPWM(void){
    pinMode(5,OUTPUT);//PE3 OC3A
  //  pinMode(2,OUTPUT);//PE4 OC3B
   // pinMode(3,OUTPUT);//PE5 OC3C

//https://noctua.at/pub/media/wysiwyg/Noctua_PWM_specifications_white_paper.pdf
// https://medesign.seas.upenn.edu/index.php/Guides/MaEvArM-timer3
  TCCR3A = (1<<COM3A1)|(1<<COM3A0); // presclaler
 // TCCR3B = (1<<WGM33)|(1<<CS31)|(1<<CS30);  // 64 presclaler
 //    TCCR3B = (1<<WGM33)|(1<<CS31); // 8 presclae 



/*
    TCCR3B = (1<<WGM33)|(1<<CS31); // 8 presclae 
  ICR3 = 40;//10000; freq  //25Khz -> 40uSec 40
  OCR3A = 20;//8500;duty cycle   20

*/
  TCCR3B = (1<<WGM33)|(1<<CS30); // no prescale 1 
  ICR3 = 300;//10000; freq  //25Khz -> 40uSec 40
  OCR3A = 150;//8500;duty cycle   20

  TCNT3 = 0;
  sei();

}
// interrupt vector
    #ifdef ARDUINO_MEGA
ISR(TIMER1_OVF_vect){        // interrupt service routine that wraps a user defined function supplied by attachInterrupt

  TCNT1 = TIMER_INT_VAL;
    #endif
    
    #ifdef ARDUINO_DUE
void TC3_Handler(){
        TC_GetStatus(TC1, 0);
    #endif

     #ifdef ARDUINO_MKRZERO
void TC5_Handler (void) {
  
      #endif 
    
     #ifdef CHIPKIT_MAX32
  //  void __ISR(_TIMER_2_VECTOR, i p l 3 ) myISR function ( void ){
void JustAFunction(){
    //  ISR(TIMER1_OVF_vect){
    #endif  

 /*   
    #if defined (ARDUINO_MEGA)  | defined (ARDUINO_DUE)//  | defined (ARDUINO_MKRZERO)
          digitalWrite(DEBUG_OUT, digitalRead(DEBUG_OUT) ^ 1);   
    #endif
    #if defined (ARDUINO_MKRZERO) | defined (CHIPKIT_MAX32)
          digitalWrite(DEBUG_OUT, digitalRead(DEBUG_OUT) ^ 1);   
    #endif     
*/

    #if  defined KEY_DIGITAL
      Key_Functions_Digital();
    #endif
    #if  defined KEY_ANALOG  
      
      Key_Functions_Analog(analogRead(KEY_ANALOG_IN));
    #endif
    Loop.IntTimer_10++; 
    Loop.IntTimer_100++;
    Loop.IntTimer_250++;
    Loop.IntTimer_500 ++;
    Loop.IntTimer_1_Sec ++;
    Loop.IntTimer_2_Sec ++;
    Loop.IntTimer_5_Sec ++;
    Loop.IntTimer_10_Sec ++;
    Loop.IntTimer_20_Sec ++;   
    Loop.IntTimer_60_Sec ++;  

    Loop.Task_SameInt = ON;
    //DAQ_Send_Data(INTERRUPT_BASED); 
    if(Loop.IntTimer_10 >= SCALE_10MSEC){
      Loop.IntTimer_10 = 0;
      Loop.Task_10msec = ON;
      // DAQ_Send_Data(INTERRUPT_BASED); 
    }

    if(Loop.IntTimer_100 >= SCALE_100MSEC){
      Loop.IntTimer_100 = 0;
      Loop.Task_100msec = ON;
     //  DAQ_Send_Data(INTERRUPT_BASED); 
    }
    if(Loop.IntTimer_250 >= SCALE_250MSEC){
      Loop.IntTimer_250 = 0;
      Loop.Task_250msec = ON;
            DAQ_Send_Data(INTERRUPT_BASED); 
    }
    if(Loop.IntTimer_500 >= SCALE_500MSEC){ // 500 msec
      Loop.IntTimer_500 = 0;
      Loop.Task_500msec = ON;  

    }
    if(Loop.IntTimer_1_Sec >= SCALE_1SEC){  // 1 sec
      Loop.IntTimer_1_Sec = 0;
      Loop.Task_1Sec = ON;
      digitalWrite(LED_GREEN, digitalRead(LED_GREEN) ^ 1);  
 
      if(Display.SleepEnable == ON){
        if(Display.OLED_Timer) Display.OLED_Timer--;   // sleep active
      }
      else Display.OLED_Timer = 32768; // no sleep    
      if(Display.InitDelay == OFF)Display.InitDelay = ON;           
    }
    
    if(Loop.IntTimer_2_Sec >= SCALE_2SEC){ // 2 sec
      Loop.IntTimer_2_Sec = 0;
      Loop.Task_2Sec = ON;
      //PrintDisplayBuffer();
    }
    if(Loop.IntTimer_5_Sec >= SCALE_5SEC){  // 5 sec
      Loop.IntTimer_5_Sec = 0;
      Loop.Task_5Sec = ON;
    }
    if(Loop.IntTimer_10_Sec >= SCALE_10SEC){  // 10 sec
      Loop.IntTimer_10_Sec = 0;
      Loop.Task_10Sec = ON;
    }
    if(Loop.IntTimer_20_Sec >= SCALE_20SEC){  // 20 sec
      Loop.IntTimer_20_Sec = 0;
      Loop.Task_20Sec = ON;
    }
    if(Loop.IntTimer_60_Sec >= SCALE_60SEC){  // 60 sec
      Loop.IntTimer_60_Sec = 0;
      Loop.Task_60Sec = ON;
    }  
          

       //  digitalWrite(LED_RED, digitalRead(LED_RED) ^ 1);
     //   if(!digitalRead(KEY_LEFT) || !digitalRead(KEY_MID) || !digitalRead(KEY_RIGHT))
     #ifdef ARDUINO_MKRZERO
         TC5->COUNT16.INTFLAG.bit.MC0 = 1; //Writing a 1 to INTFLAG.bit.MC0 clears the interrupt so that it will run again
     #endif

}
// https://www.onlinegdb.com/edit/Hkmlxi_08

void Log_Data_Write_SD(){
    #ifdef SDCARD_EXISTS
      SD_CardLogTask(); // Data 2 SD card Write
    #endif 
}

void Common_Loop(){

  if (Loop.Task_100msec) {
    Loop.Task_100msec = OFF;
    // DAQ_Send_Data(LOOP_BASED); 
    SerialPortRx_UI();

     Set_Duty_Cycle();
 
  }
  if (Loop.Task_250msec) {
    Loop.Task_250msec = OFF;
	  #ifdef OLEDDISPLAY_EXISTS
    // One time after wake up form sleep
    if (Display.OLED_Init == ON) {
      Display_ReInit_Start(20);
      Display.OLED_Init = OFF;
    }
    if (Display.OLED_Timer) {
      displayValues();
    }
    else {
      Display_SwitchOff();
    }
    Display_ReInit_End();
    #endif
    #ifdef LEM_CURRENT_EXISTS
        AnalogValRead();
    #endif
  
  }
  if (Loop.Task_500msec) {
    Loop.Task_500msec = OFF;
    if (SampleTime == TASK_500MSEC) Log_Data_Write_SD();
     
  }

  if (Loop.Task_1Sec) {
    Loop.Task_1Sec = OFF;
#ifndef DEBUG_SIMULATOR_MODE

    SerialPortRx(); // for date & time
    RTC_TimeClock();
   // UI_SerialPort_Receive();
  


#ifdef WIND_SENSOR_EXISTS
    WindSensorRead();
#endif
#ifdef TEMP_HUM_ONBOARD_SENSOR_EXISTS
    SensorRead_Si072(SI072_ONBOARD_SENSOR_ADDR); // MULTIPLEXER NO  
  //  Serial.print(F("Temperature_OnBoard "));Serial.println(Values.Temperature_OnBoard);
   // Serial.print(F("Humidity_OnBoard "));Serial.println(Values.Humidity_OnBoard);
    
#endif
#ifdef TEMP_HUM_1_SENSOR_EXISTS
    SensorRead_Si072(SI072_FIRST_SENSOR_ADDR); // MULTIPLEXER NO  
#endif
#ifdef TEMP_HUM_2_SENSOR_EXISTS  
    SensorRead_Si072(SI072_SECOND_SENSOR_ADDR); // MULTIPLEXER NO  
#endif
#ifdef TEMP_HUM_3_SENSOR_EXISTS 
    SensorRead_Si072(SI072_THIRD_SENSOR_ADDR); // MULTIPLEXER NO
#endif

 #ifdef  BME688_SENSOR_MD1_EXISTS
    SensorVoc_BME680_Read(ADR_MD1);   
#endif 
 #ifdef  BME688_SENSOR_MD2_EXISTS
    SensorVoc_BME680_Read(ADR_MD2);   
#endif 
 #ifdef  BME688_SENSOR_MD3_EXISTS
    SensorVoc_BME680_Read(ADR_MD3);   
#endif 
 #ifdef  BME688_SENSOR_MD4_EXISTS
    SensorVoc_BME680_Read(ADR_MD4);   
#endif 

#ifdef GROVE_GAS_V2_MD5_EXISTS
    SensorGroveV2_Read(ADR_MD5);
#endif 
#ifdef GROVE_GAS_V2_MD6_EXISTS
    SensorGroveV2_Read(ADR_MD6);
#endif 
#ifdef GROVE_GAS_V2_MD7_EXISTS
    SensorGroveV2_Read(ADR_MD7);
#endif 
#ifdef GROVE_GAS_V2_MD8_EXISTS
    SensorGroveV2_Read(ADR_MD8);
#endif 

#ifdef ENERGYMETER_EXISTS
     EnergymeterCalbLed( );
#endif

#ifdef  BAR_PRES_SENSOR_EXISTS
    SensorAlt_Read();
#endif
#ifdef LIGHT_SENSOR_EXISTS
    SensorLight_Read();
#endif
#ifdef ACCL_GYRO_SENSOR_EXISTS
    SensorAcccel_GyroRead();
#endif
#endif // end of  #ifndef DEBUG_SIMULATOR_MODE

    KeyTimeOutCheck();
    if (SampleTime == TASK_1SEC) Log_Data_Write_SD();
     if((MainMenu == MENU5_SUB7)||(MainMenu == MENU2_SUB8)||(MainMenu == MENU3_SUB3) ||(MainMenu == MENU3_SUB4)||(MainMenu == MENU1_SUB3)||(MainMenu == MENU1_SUB4)||(MainMenu == MENU6_SUB3) ){
      Display.MenuTimeout++;
      if(Display.MenuTimeout > 4){
        Display.MenuTimeout = 0;
        MainMenu = MENU_NULL;
      }
    }
   
/*
      if( (Values.PM25 > 64) &&  !digitalRead(RELAY_OUT_1) ) digitalWrite(RELAY_OUT_1,HIGH);
      if( (Values.PM25 < 16) &&   digitalRead(RELAY_OUT_1) ) digitalWrite(RELAY_OUT_1,LOW);

      //    digitalWrite(RELAY_OUT_1, digitalRead(RELAY_OUT_1) ^ 1);  

      if(Values.Current > 4 && !digitalRead(RELAY_OUT_2) ) digitalWrite(RELAY_OUT_2,HIGH);
      if(Values.Current < 1 && digitalRead(RELAY_OUT_2) ) digitalWrite(RELAY_OUT_2,LOW);     
*/
           //     digitalWrite(RELAY_OUT_2, digitalRead(RELAY_OUT_2) ^ 1);  
  }  
  if (Loop.Task_2Sec) {
    Loop.Task_2Sec = OFF;
    if (SampleTime == TASK_2SEC) Log_Data_Write_SD();
      UpdateDispRoll(DOWNROLL);
      PrintDisplayBuffer();

      if(SDCard.PauseTimer){
         SDCard.PauseTimer--;    
      }
        #ifdef ENERGYMETER_EXISTS   
             EnergyMeterIC_Operation();
        #endif
  }
  if (Loop.Task_5Sec) {
    Loop.Task_5Sec = OFF;
    if (SampleTime == TASK_5SEC) Log_Data_Write_SD();
      #ifdef PM25_DUST_SENSOR_EXISTS  
          SDS_DustSensor();
      #endif
//Serial1.print("abcde"); // max 32
   // Relay_loop();
 
    Display.ValueTimer++;
    if (Display.ValueTimer > 4)Display.ValueTimer = 0;
  }
  if (Loop.Task_10Sec) {
    Loop.Task_10Sec = OFF;
    if (SampleTime == TASK_10SEC) Log_Data_Write_SD();
  }
  if (Loop.Task_20Sec) {
    Loop.Task_20Sec = OFF;
    if (SampleTime == TASK_20SEC) Log_Data_Write_SD();
  }
  if (Loop.Task_60Sec) {
    Loop.Task_60Sec = OFF;
    if (SampleTime == TASK_60SEC) Log_Data_Write_SD();
  }
}

void IO_Settings() {
  #ifdef FIRST_PROTOTYPE

  #endif

  #ifdef ARDUINO_DUE
    // default i2c pin is for Mega not Due so set pins to input
    pinMode(70, INPUT);  // 
    pinMode(71, INPUT);  // 
  #endif

  #ifdef ARDUINO_MKRZERO
    digitalWrite(OLED_GND, LOW);
    pinMode(OLED_GND, OUTPUT);  // 

    pinMode(A4, INPUT);  
    digitalWrite(OLED_CS, LOW);
    pinMode(OLED_CS, OUTPUT);  //
    digitalWrite(OLED_RESET, LOW);
    pinMode(OLED_RESET, OUTPUT);  // 

    digitalWrite(OLED_DC, LOW);
    pinMode(OLED_DC, OUTPUT);  // 
    digitalWrite(OLED_CLK, LOW);
    pinMode(OLED_CLK, OUTPUT);  // 
    digitalWrite(OLED_MOSI, LOW);
    pinMode(OLED_MOSI, OUTPUT);  // 

    digitalWrite(OLED_POWER, HIGH);
    pinMode(OLED_POWER, OUTPUT);  // 
  
  #endif

  #ifdef ENERGYMETER_EXISTS
  /*
  pinMode(A5, INPUT);  // 
  pinMode(A4, INPUT);  // 
  pinMode(A3, INPUT);  // 
  pinMode(A2, INPUT);  // 
  pinMode(A1, INPUT);  // 
  digitalWrite(A4, HIGH);
  digitalWrite(A3, HIGH);
  digitalWrite(A2, HIGH);
  digitalWrite(A1, HIGH);
  pinMode(A5, INPUT_PULLUP);  //
  pinMode(A4, INPUT_PULLUP);  //
  pinMode(A3, INPUT_PULLUP);  //
  pinMode(A2, INPUT_PULLUP);  //
  pinMode(A1, INPUT_PULLUP);  //  
  */
  //  pinMode(I2C_TIMEOUT, INPUT);  // 
 // pinMode(DEBUG_OUT, OUTPUT);  // 

  pinMode(ADE9153A_RESET_PIN, OUTPUT);  // ADE9153A_RESET_PIN
  digitalWrite(ADE9153A_RESET_PIN, HIGH);
  pinMode(ADE9153A_CS_PIN, OUTPUT);  // ADE9153A_SPI_SS_PIN
  digitalWrite(ADE9153A_CS_PIN, HIGH);
  pinMode(ADE9153A_ZX_DREADY_PIN, INPUT);  // ADE9153A_ZX_DREADY_PIN
  pinMode(ADE9153A_IRQ_PIN, INPUT);  // ADE9153A_IRQ_PIN
  pinMode(ADE9153A_CALB_BUTTON, INPUT);  // ADE9153A_USER_BUTTON 
  
  pinMode(ADE9153A_ZX_DREADY_PIN, INPUT_PULLUP);
  pinMode(ADE9153A_IRQ_PIN, INPUT_PULLUP);
  pinMode(ADE9153A_CALB_BUTTON, INPUT_PULLUP);
  #endif

/*  
  digitalWrite(RELAY_OUT_1, LOW);
  pinMode(RELAY_OUT_1, OUTPUT);  // SS Pin high to avoid miscommunication

  digitalWrite(RELAY_OUT_2, LOW);
  pinMode(RELAY_OUT_2, OUTPUT);  // SS Pin high to avoid miscommunication
*/
  pinMode(10, OUTPUT);
  digitalWrite(10, HIGH);
  pinMode(LED_GREEN, OUTPUT);           // set pin to input
  digitalWrite(LED_GREEN, LOW);       // turn on pullup resistors
  pinMode(LED_RED, OUTPUT);           // set pin to input
  digitalWrite(LED_RED, LOW);       // turn on pullup resistors
  pinMode(DIPSWITCH_1, INPUT);           // set pin to input
  pinMode(DIPSWITCH_2, INPUT);           // set pin to input
/*
  pinMode(KEY_LEFT, INPUT);           // set pin to input
  pinMode(KEY_LEFT, INPUT_PULLUP);

  pinMode(KEY_UP, INPUT);           // set pin to input
  pinMode(KEY_UP, INPUT_PULLUP);

  pinMode(KEY_DOWN, INPUT);           // set pin to input
  pinMode(KEY_DOWN, INPUT_PULLUP);

  pinMode(KEY_RIGHT, INPUT);           // set pin to input
  pinMode(KEY_RIGHT, INPUT_PULLUP);
*/
//  pinMode(KEY_ANALOG, INPUT);  // 
}

void UI_SerialPort_Receive(){
          // UIPort.println(); // max 32
        //   UIPort.print("abcde"); // max 32
           //UIPort.print('\n');//end marker
    static uint8_t ndx = 0;
    //char endMarker = '\n';
   // char endMarker = 'e';
    //uint8_t Counter = 0;
 /*
        // if (Serial.available() > 0) {
     while (UIPort.available() > 0 && UIPort_newData == false) {
        char rc = UIPort.read();
        Counter++; // 2020,05,27,21,14,23 19 characters + \0' // in total 21

        if (rc != endMarker) {
            UIPort_rxarr[ndx] = rc;
            ndx++;
            if (ndx >= numChars) {
              ndx = numChars - 1;
          }
        }
        else {
          UIPort_rxarr[ndx] = '\0'; // terminate the string
          ndx = 0;
          UIPort_newData = true;
      }
   }
      if (UIPort_newData == true) {
        Serial.print(F("This just in .................................. "));
          Serial.println(rxarr);
          UIPort_newData = false;            
          Serial.print(F("Counter:"));  
          Serial.println(Counter);


      }
*/
         while (Serial1.available()> 0) {
            char rc = Serial1.read();
            Serial.print(F("UI 2-> "));
            Serial.println(rc);
                   // if (rc != endMarker) {
            UIPort_rxarr[ndx] = rc;
            ndx++;
            if (ndx >= numChars) {
              ndx = numChars - 1;
            }
         }
          Serial.print(F("UI Port........................................................ "));       
          Serial.println(UIPort_rxarr);                 
          //UIPort_rxarr[ndx] = '\0'; // terminate the string
          ndx = 0;       
}

void MicroInit() {
 // Display_Line4[5] ='\0';
  Serial.begin(115200);

  //SoftwareSerial UIPort(17, 16); //(RX,TX)
 pinMode(19, INPUT); //17
  pinMode(19, INPUT_PULLUP); //17
  pinMode(18, OUTPUT);  //16
  Serial1.begin(115200);

  pinMode(17, INPUT); //17
  pinMode(16, OUTPUT);  //16
  Serial2.begin(115200);
  //pinMode(15, INPUT_PULLUP); //17
  //pinMode(14, OUTPUT);  //16

  /*   //SoftwareSerial portThree(15, 14);
  pinMode(15, INPUT_PULLUP);
  pinMode(14, OUTPUT);
  portThree.begin(115200);
  */
  IO_Settings();
  #ifdef OLEDDISPLAY_EXISTS
  DisplaySetPowerIO();
  #endif
  ResetCasePrint();
  //  SDCard.LogStatus = 0;      // default start with log off;
  NVRam_Read_Standbye();
  NVRam_Read_SampleTime();
  SDCard.LogBootInit = 0;  // put the header of the csv file

  Serial.print(F("SDCard.LogEnable: "));
  Serial.print(SDCard.LogEnable);
  Serial.print(F("    SampleTime: "));
  Serial.println(SampleTime);
  Serial.print(F("    DisplaySleep: "));
  Serial.println(Display.SleepEnable);

#ifdef ARDUINO_MEGA
  wdt_reset();
  wdt_enable(WDTO_8S);   //wdt_enable(WDT0_1S);
  //https://www.nongnu.org/avr-libc/user-manual/group__avr__watchdog.html
#endif

#ifdef ARDUINO_DUE
  wdt_reset();
  wdt_enable();   
  //https://forum.arduino.cc/t/due-watchdog-timer-not-working/525122/2
#endif

#ifdef ARDUINO_MEGA
//  ADCSRA &= ~ (1 << ADEN);            // turn off ADC to save power ,, enable when needed and turn off again
    ADCSRA |= (1 << ADEN); // enable adc
#endif
  Serial.print(F("Compiled: "));
  Serial.println( __DATE__ ", " __TIME__ ", " __VERSION__); 
 // Serial.println( F("Compiled: ") __DATE__ ", " __TIME__ ", " __VERSION__);
  //Compiled: Jul 21 2020 15:55:39 7.3.0
  //  ShowSerialCode();
  NVRam_Read_SerNo();
  NVRam_Read_QueNo();
  UpdateLogFileNo();
  UpdateLogFileId();
  
//#ifndef DEBUG_SIMULATOR_MODE
   #ifdef SDCARD_EXISTS
    SD_Card_Info();
   SD_Card_Init();
    GetFileSize();
   ReadConfigFile(); // 2020.12.25
  #endif  
  
  Sensors_PeripInit();
  DateTimeBuf.Init = ON;
  #ifdef OLEDDISPLAY_EXISTS
  DisplayInit();   
 // #endif
#endif

#ifdef ARDUINO_MEGA
  // initialize timer1
    noInterrupts();           // disable all interrupts
    TCCR1A = 0;
    TCCR1B = 0;
        TCNT1 = TIMER_INT_VAL;            // preload timer 65536-16MHz/256/2Hz 500mS
   //TCNT1 = 64286;            // preload timer 65536-16MHz/256/50Hz 20 ms
  //  TCCR1B |= (1 << CS12);    // 256 prescaler
    // TCCR1B |= 0B00000100; // b2,b1,b0 -> 100 -> 256
   //   TCCR1B = 0B00000100; // b2,b1,b0 -> 100 -> 256
   //   TCCR1B |= 0x00000001; // b2,b1,b0 -> 001 -> 1   no prescaler
  //    TCCR1B |= 0x00000010; // b2,b1,b0 -> 010 -> 8   
  TCCR1B |=  TIMER_INT_PRESCALE;
    TIMSK1 |= (1 << TOIE1);   // enable timer overflow interrupt
    interrupts();
    Set_FanPWM();
#endif

#ifdef ARDUINO_DUE
    startTimer(TC1, 0, TC3_IRQn, 50); //TC1 channel 0, the IRQ for that channel and the desired frequency 64
        // 20 -> 50ms   1000/20 = 50
        // 32 -> 32ms       
        // 50 -> 20ms
        // 51 -> 19.62 m2
        // 52 -> 19.22 ms      
        // 54 -> 18.63
        // 64 -> 15.68 ms
#endif
 #ifdef ARDUINO_MKRZERO // ARM Cortex M0
  tcConfigure(50000); //configure the timer to run at <sampleRate>Hertz
  tcStartCounter(); //starts the timer
#endif

}
/*
void Parse_FileString(){
  int DelimCount=0;
  int j = 0;
  int index;
  for (unsigned int i = 0; i < Config_Str.length(); i++) {
    if (Config_Str.substring(i, i+1) == ",") {
      switch(DelimCount){
        case 0:Relay1str = Config_Str.substring(j, i);
        break;
        case 1:RlStr2 = Config_Str.substring(j,i);
        break;
        case 2:RLlVal = Config_Str.substring(j,i);
        break;
        case 3:RlStr4 = Config_Str.substring(j,i);
        break;
        case 4:Relay2str = Config_Str.substring(j,i);
        break;
        case 5:RlStr6 = Config_Str.substring(j,i);
        break; 
        case 6:RL2Val = Config_Str.substring(j,i);
             j = i+1;
             RlStr8 = Config_Str.substring(j);
        break; 
        default:
        break;   
      }
      j = i+1;
      DelimCount++;
    }
  }
 
    index = ELEMENTS;
    Relay1str.trim();//remove leadig & last space characters
    if(Relay1str == "Relay1"){
     // index == 0; ?????????????????
      RLlVal.trim();  // Temperature // Current // PM25
      for( index = 0; index < ELEMENTS; index++){
        if(RLlVal == KeyWords[index]){      
          RlStr2.trim();
          RlStr4.trim();       
          RL1Min =RlStr2.toFloat();
          RL1Max= RlStr4.toFloat();
          
          break;  
        }  
      } 
   }
   if(index == ELEMENTS) {
        Relay1str = "---";     
        RLlVal = "Nan";   
        RlStr2 = "----";
        RlStr4 = "----";
    }
    index = ELEMENTS;
    Relay2str.trim();//remove leadig & last space characters
    if(Relay2str == "Relay2"){
      index = 0;
      RL2Val.trim();  // Temperature // Current // PM25
      for( index = 0; index < ELEMENTS; index++){
        if(RL2Val == KeyWords[index]){      
          RlStr6.trim();
          RlStr8.trim();       
          RL2Min= RlStr6.toFloat();
          RL2Max= RlStr8.toFloat();
          break;  
        }  
      }     
   }
    if(index == ELEMENTS) {
        Relay2str = "---";     
        RL2Val = "Nan";   
        RlStr6 = "----";
        RlStr8 = "----";
    }
   // Serial.println("'''''''''''''''''");
  Serial.println(Relay1str);
  Serial.println(RlStr2);
  Serial.println(RLlVal);
  Serial.println(RlStr4);
  Serial.println(Relay2str);
  Serial.println(RlStr6);
  Serial.println(RL2Val);
  Serial.println(RlStr8);
}
#define Relay1_Val 8
#define Relay2_Val 4

float GetValue(uint8_t Relay){
  String Val = "";
  if(Relay == Relay1_Val) Val =  RLlVal;
  else if(Relay == Relay2_Val) Val =  RL2Val;
  if(Val == KeyWords[0])return Values.Temperature_Ch1;
  if(Val == KeyWords[1])return Values.Temperature_Ch2;
  if(Val == KeyWords[2])return  Values.Temperature_Ch3;
  if(Val == KeyWords[3])return Values.Humidity_Ch1;
  if(Val == KeyWords[4])return  Values.Humidity_Ch2;
  if(Val == KeyWords[5])return  Values.Humidity_Ch3;
  if(Val == KeyWords[6])return  Values.PM25;
  if(Val == KeyWords[7])return  Values.PM10;
  if(Val == KeyWords[8]) return  Values.Current;
  if(Val == KeyWords[9]) return  Values.Voltage;
  if(Val == KeyWords[10]) return  Values.ActivePower;  
  if(Val == KeyWords[11]) return  Values.PowerFactor;     
  return 0;
}

void Relay_loop() {
 // Parse_FileString();
  CompValue = 0;
  if( RLlVal != "Nan"){
    CompValue = GetValue(Relay1_Val);
    if(CompValue > RL1Max && !digitalRead(RELAY_OUT_1) ) digitalWrite(RELAY_OUT_1,HIGH);
    if(CompValue < RL1Min &&  digitalRead(RELAY_OUT_1) ) digitalWrite(RELAY_OUT_1,LOW); 
  }
  else{
      digitalWrite(RELAY_OUT_1,LOW);
  }
  Serial.print(F("RL1Min: "));Serial.println(RL1Min);
  Serial.print(RLlVal+":  ");Serial.print(CompValue); 
  Serial.print(F("     RELAY1: "));Serial.println(RELAY_OUT_1); 
  Serial.print(F("RL1Max: "));Serial.println(RL1Max); 
  CompValue = 0;
  if( RL2Val != "Nan"){
    CompValue = GetValue(Relay2_Val);
    //if((CompValue < RL2Min)&& RELAY2) RELAY2 = 0;
    //if((CompValue > RL2Max)&& !RELAY2) RELAY2 = 1;  
    if(CompValue > RL2Max && !digitalRead(RELAY_OUT_2) ) digitalWrite(RELAY_OUT_2,HIGH);
    if(CompValue < RL2Min &&  digitalRead(RELAY_OUT_2) ) digitalWrite(RELAY_OUT_2,LOW);  
  }
  else{
       digitalWrite(RELAY_OUT_2,LOW);  
  }
  Serial.print(F("RL2Min: "));Serial.println(RL2Min);
  Serial.print(RL2Val+":  ");Serial.print(CompValue);
  Serial.print(F("      RELAY2: "));Serial.println(RELAY_OUT_2);
  Serial.print(F("RL2Max: "));Serial.println(RL2Max);  
}
*/
