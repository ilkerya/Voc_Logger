 
//https://github.com/adafruit/Data-Logger-shield
// https://forum.arduino.cc/index.php?topic=523216.0

RTC_PCF8523 rtc; // I2c Addres 0x68
//char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};
//static const char char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesdy", "Wedns.", "Thurs.", "Friday", "Satur."};

void RTC_TimeClock(){

  if(DateTimeBuf.RTC_Update == ON){
    DateTimeBuf.RTC_Update = OFF;
    rtc.adjust(DateTime(DateTimeBuf.Year, DateTimeBuf.Month, DateTimeBuf.Day, DateTimeBuf.Hour, DateTimeBuf.Minute, DateTimeBuf.Second));
    return;
  }
    DateTime now = rtc.now();

    Str_DispTime = "";
    Str_DispTime += String(now.year(),DEC);   
    Str_DispTime += ',';    
    if(now.month()<10)Str_DispTime += '0';  
     Str_DispTime += String(now.month(),DEC);
     Str_DispTime += ',';       
    if(now.day()<10)Str_DispTime += '0';  
     Str_DispTime += String(now.day(),DEC);  
     Str_DispTime += ",";
    if(now.hour()<10)Str_DispTime += ' ';
     Str_DispTime += String(now.hour(),DEC);
      Str_DispTime += ',';    
    if(now.minute()<10)Str_DispTime += '0';   
     Str_DispTime += String(now.minute(),DEC);
    Str_DispTime += ',';   
    if(now.second()<10)Str_DispTime += '0';   
     Str_DispTime += String(now.second(),DEC);
     Str_DispTime +=  ','; 

    Str_Date = "";
    Str_Date += String(now.year(),DEC);   
    Str_Date += '.';    
    if(now.month()<10)Str_Date += '0';  
     Str_Date += String(now.month(),DEC);
     Str_Date += '.';       
    if(now.day()<10)Str_Date += '0';  
     Str_Date += String(now.day(),DEC);
    // Str_Date += ''; 
     
     Str_Time = "";
    if(now.hour()<10)Str_Time += '0';
     Str_Time += String(now.hour(),DEC);
      Str_Time += ':';    
    if(now.minute()<10)Str_Time += '0';   
     Str_Time += String(now.minute(),DEC);
    Str_Time += ':';   
    if(now.second()<10)Str_Time += '0';   
     Str_Time += String(now.second(),DEC);
   //  Str_Time +=  ''; 

    if(DateTimeBuf.Init == ON){
      DateTimeBuf.Init = OFF;
      DateTimeBuf.Year = now.year();
      DateTimeBuf.Month = now.month();
      DateTimeBuf.Day = now.day();
      DateTimeBuf.Hour = now.hour();
      DateTimeBuf.Minute = now.minute();
      DateTimeBuf.Second = now.second();
    }
}
void SerialPortRx_UI() {
      uint8_t i = 0 ;
      uint8_t j = 0;
    //  Serial.println("REceive");
      //while (Serial1.available()) {
       if (Serial2.available() > 0) {
        Serial.println();
/*
        for(j = 0; j < PREAMBLE_BYTES ; j++) {
          RX_UI_ARR[j] =  VS_Port.read();
        }
*/
         RX_UI_ARR[0] =  Serial2.read();
         RX_UI_ARR[1] =  Serial2.read();         
         if((RX_UI_ARR[0] != 0xAA) || (RX_UI_ARR[1] != 0xAA)){
            Serial.print(F("Preamble Mismatch "));
            uint8_t temp;
            for(j = 0; j < DEFAULT_RX_LENGTH ; j++) {
              temp =  Serial2.read();
            }
            Serial2.flush();
            return;
        }        
        
        Serial.print(RX_UI_ARR[0],HEX);Serial.print("."); 
        Serial.print(RX_UI_ARR[1],HEX);Serial.print("."); 

        for(j = PREAMBLE_BYTES; j < DEFAULT_RX_LENGTH ; j++) {
          RX_UI_ARR[j] =  Serial2.read();
          Serial.print(RX_UI_ARR[j],HEX);Serial.print(".");        
        }



         uint16_t CrcRx_Calc =  RX_UI_ARR[DEFAULT_RX_LENGTH-2]<<8;      
        CrcRx_Calc += RX_UI_ARR[DEFAULT_RX_LENGTH-1];
        Serial.print(F("."));Serial.print(CrcRx_Calc,HEX);

        DAQ.CRC_Calc_Receive = DEFAULT_CRC_INIT;//2-14
        for(i=PREAMBLE_BYTES; i< (DEFAULT_RX_LENGTH-CRC_BYTES); i++){ // crc haric 2-62 0-63  62 63 CRC 
            DAQ.CRC_Calc_Receive ^= RX_UI_ARR[i];
        }
        DAQ.CRC_Calc_Receive <<= 8;
        for(i=PREAMBLE_BYTES; i< (DEFAULT_RX_LENGTH-CRC_BYTES); i++){ // crc haric
          DAQ.CRC_Calc_Receive ^= RX_UI_ARR[i];
        }
        Serial.print(F(" CRC "));Serial.print(DAQ.CRC_Calc_Receive,HEX);

        if(DAQ.CRC_Calc_Receive != CrcRx_Calc) {
            Serial.println(F("CRC Mismatch "));
            Serial2.flush();
            uint8_t temp;
            for(j = 0; j < ATMEGA32_BUFFER_SIZE ; j++) { // Atmega buffer size 
              temp =  Serial2.read(); 
            }
               return; 
        }
        DAQ.Ack = ON; 

        Duty_Cycle = RX_UI_ARR[5];
       }
}

void  RTC_Init(){
  delay(10);
  if (! rtc.begin()) {
    Serial.println(F("Couldn't find RTC"));
    //while (1);
  }
  else {
    if (! rtc.initialized()) {
      Serial.println(F("RTC is NOT running!"));
      // following line sets the RTC to the date & time this sketch was compiled
     rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
      //   RTC_Test = 18;
      // This line sets the RTC with an explicit date & time, for example to set
      // January 21, 2014 at 3am you would call:
      // rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));
    }
    else{
      Serial.println(F("RTC is Running!"));
    } 

    if(Serial) { // if serial port connected adjust it from computer
    //  rtc.adjust(DateTime(F(__DATE__), F(__TIME__))); 
    //  Serial.println("RTC Time Adjusted!");
    //  RTC_Test = 18;
    }
  }
}

 void SerialPortRx() {
    static uint8_t ndx = 0;
    char endMarker = '\n';
    uint8_t Counter = 0;
    
        // if (Serial.available() > 0) {
     while (Serial.available() > 0 && newData == false) {
        char rc = Serial.read();
        Counter++; // 2020,05,27,21,14,23 19 characters + \0' // in total 21

        if (rc != endMarker) {
            rxarr[ndx] = rc;
            ndx++;
            if (ndx >= numChars) {
              ndx = numChars - 1;
          }
        }
        else {
          rxarr[ndx] = '\0'; // terminate the string
          ndx = 0;
          newData = true;
      }
   }
  //////////////////// end of while loop 
   if (newData == true) {
        Serial.print(F("This just in .................................. "));
          Serial.println(rxarr);
          newData = false;            
          Serial.print(F("Counter:"));  
          Serial.println(Counter);

               
        if((Counter == 21) && (rxarr[4] == ',' )&&(rxarr[7] == ',') && 
           (rxarr[10] == ',')&&  (rxarr[13] == ',' )&& (rxarr[16] == ',' )){
            for(int i = 0; i<32; i ++ ){ // ascii 2 byte
                rxarr[i] -= 48;          
            }
          unsigned int Year=0; 
          unsigned int Month=0; 
          unsigned int Day=0;                       
          unsigned int Hour=0; 
          unsigned int Minute=0; 
          unsigned int Second=0; 
            
            Year  = 1000 * rxarr[0];
            Year  += 100 * rxarr[1]; 
            Year  += 10 * rxarr[2];  
            Year  += rxarr[3];  
            Month  += 10 * rxarr[5];      
            Month  += rxarr[6];  
            Day  += 10 * rxarr[8];       
            Day  += rxarr[9];            
            Hour  += 10 * rxarr[11];  
            Hour  += rxarr[12]; 
            Minute  += 10 * rxarr[14];
            Minute  += rxarr[15]; 
            Second  += 10 * rxarr[17];    
            Second  += rxarr[18];       
            rtc.adjust(DateTime(Year, Month, Day, Hour, Minute, Second));
            MainMenu = MENU5_SUB7;
            DispExtTimeout();
        }
        if((Counter == 11) && (rxarr[0] == 'F' )&&(rxarr[1] == 'I') && (rxarr[2] == 'L')&&  (rxarr[3] == 'E' )
                                                &&(rxarr[4] == 'Q') && (rxarr[5] == 'U')&&  (rxarr[6] == 'E' )){  
            NVRam_Write_QueNo(&rxarr[7]);                
            delay(10);
            NVRam_Read_QueNo();
            UpdateLogFileId(); 
            SDCard.LogBootInit = 0; // restart logging from 0                       
        }  
        if((Counter == 11) && (rxarr[0] == 'D' )&&(rxarr[1] == 'E') && (rxarr[2] == 'V')&& 
                                                    (rxarr[3] == 'I' ) &&(rxarr[4] == 'D' )){       
            NVRam_Write_SerNo(&rxarr[5]); 
            delay(10);
            NVRam_Read_SerNo();
            UpdateLogFileId(); 
            SDCard.LogBootInit = 0; // restart logging from 0                       
        }        
    }
}
