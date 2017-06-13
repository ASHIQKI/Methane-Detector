#define POLYNOMIAL 0x8005             //polynomial for finding crc of the message
#define WIDTH  16
#define TOPBIT (1 << (WIDTH - 1))


#define DLE 0x10
#define DAT 0x1A
#define EOF1 0x1F
#define RD  0x13
#define WR 0x15
#define WP1 0xE5
#define WP2 0xA2
#define ACK 0x16

#define crcorchecksum cs16  //defined to select whether crc or check sum.

char pc_cmd[32];
char sensor_resp_full[300];
char sensor_resp_data[300];
uint8_t sensor_resp_full_index=0,sensor_resp_data_index=0,pc_cmd_index=0,pc_cmd_rcvd=0,sensor_resp_rcvd=0;
char identifier;
uint8_t cal_flag=0;
char tempbuff[128];




//FUNCTION TO SETUP SERIAL PORTS
void setup(){
  Serial.begin(9600);// for debugging purpose
  Serial1.begin(9600);//connected with the sensor
}






//FUNCTION TO CALCULATE CHECK SUM
uint16_t cs16(const void *vptr, int nBytes) {
   uint16_t chk_sum=0x0000;
   const unsigned char *data = (unsigned char *)vptr;
   for(uint8_t i=0;i<nBytes;i++,data++)  {
        chk_sum+=*data;
    }
    return chk_sum;
}

//FUNCTION TO CALCULATE CRC OF THE MESSAGE BYTES
uint16_t crc16(uint8_t const message[], int nBytes) {
    static const unsigned short CRCTab[256] = {0x0000, 0x8005, 0x800f, 0x000a, 0x801b, 0x001e, 0x0014, 0x8011,0x8033, 0x0036, 0x003c, 0x8039, 0x0028, 0x802d, 0x8027, 0x0022,0x8063, 0x0066, 0x006c, 0x8069, 0x0078, 0x807d, 0x8077, 0x0072,0x0050, 0x8055,
    0x805f, 0x005a, 0x804b, 0x004e, 0x0044, 0x8041,0x80c3, 0x00c6, 0x00cc, 0x80c9, 0x00d8, 0x80dd, 0x80d7, 0x00d2,0x00f0, 0x80f5, 0x80ff, 0x00fa, 0x80eb, 0x00ee, 0x00e4, 0x80e1,0x00a0, 0x80a5, 0x80af, 0x00aa, 0x80bb, 0x00be, 0x00b4, 0x80b1,0x8093, 0x0096,
    0x009c, 0x8099, 0x0088, 0x808d, 0x8087, 0x0082,0x8183, 0x0186, 0x018c, 0x8189, 0x0198, 0x819d, 0x8197, 0x0192,0x01b0, 0x81b5, 0x81bf, 0x01ba, 0x81ab, 0x01ae, 0x01a4, 0x81a1,0x01e0, 0x81e5, 0x81ef, 0x01ea, 0x81fb, 0x01fe, 0x01f4, 0x81f1,0x81d3, 0x01d6,
    0x01dc, 0x81d9, 0x01c8, 0x81cd, 0x81c7, 0x01c2,0x0140, 0x8145, 0x814f, 0x014a, 0x815b, 0x015e, 0x0154, 0x8151,0x8173, 0x0176, 0x017c, 0x8179, 0x0168, 0x816d, 0x8167, 0x0162,0x8123, 0x0126, 0x012c, 0x8129, 0x0138, 0x813d, 0x8137, 0x0132,0x0110, 0x8115,
    0x811f, 0x011a, 0x810b, 0x010e, 0x0104, 0x8101,0x8303, 0x0306, 0x030c, 0x8309, 0x0318, 0x831d, 0x8317, 0x0312,0x0330, 0x8335, 0x833f, 0x033a, 0x832b, 0x032e, 0x0324, 0x8321,0x0360, 0x8365, 0x836f, 0x036a, 0x837b, 0x037e, 0x0374, 0x8371,0x8353, 0x0356,
    0x035c, 0x8359, 0x0348, 0x834d, 0x8347, 0x0342,0x03c0, 0x83c5, 0x83cf, 0x03ca, 0x83db, 0x03de, 0x03d4, 0x83d1,0x83f3, 0x03f6, 0x03fc, 0x83f9, 0x03e8, 0x83ed, 0x83e7, 0x03e2,0x83a3, 0x03a6, 0x03ac, 0x83a9, 0x03b8, 0x83bd, 0x83b7, 0x03b2,0x0390, 0x8395,
    0x839f, 0x039a, 0x838b, 0x038e, 0x0384, 0x8381,0x0280, 0x8285, 0x828f, 0x028a, 0x829b, 0x029e, 0x0294, 0x8291,0x82b3, 0x02b6, 0x02bc, 0x82b9, 0x02a8, 0x82ad, 0x82a7, 0x02a2,0x82e3, 0x02e6, 0x02ec, 0x82e9, 0x02f8, 0x82fd, 0x82f7, 0x02f2,0x02d0, 0x82d5,
    0x82df, 0x02da, 0x82cb, 0x02ce, 0x02c4, 0x82c1,0x8243, 0x0246, 0x024c, 0x8249, 0x0258, 0x825d, 0x8257, 0x0252,0x0270, 0x8275, 0x827f, 0x027a, 0x826b, 0x026e, 0x0264, 0x8261,0x0220, 0x8225, 0x822f, 0x022a, 0x823b, 0x023e, 0x0234, 0x8231,0x8213, 0x0216,
    0x021c, 0x8219, 0x0208, 0x820d, 0x8207, 0x0202};
    uint8_t data;
    uint16_t remainder = 0;
    /*
     * Divide the message by the polynomial, a byte at a time.
     */
    for (int i = 0; i < nBytes; ++i)
    {
        data = message[i] ^ (remainder >> (WIDTH - 8));
        remainder = CRCTab[data] ^ (remainder << 8);
    }

    /*
     * The final remainder is the CRC.
     */
    return (remainder);
}


//FUNCTION TO CONSTRUCT IEEE754 VALUES
char * ieee754creator(float data){
    static char array[4];
    union {
      unsigned char c[4];
      float f;
    } u;
    u.f=data;
    array[0]=u.c[0];
    array[1]=u.c[1];
    array[2]=u.c[2];
    array[3]=u.c[3];
    return array;
}





//FUNCTION TO CALCULATE IEE754 floating format
float ieee754_Conv(const void *vptr, int nBytes) {
    /*int i;
    char myString[16]={};
    memset(myString,'\0',sizeof(myString));
    float reading;
    char* buf=myString;
    uint32_t dataresult;
    const unsigned char *data = (unsigned char *)vptr;
    for(i=0; i<nBytes; i++, data--) {
      if(i==0)
       {
           buf+=sprintf(buf,"0x");
       }
      buf+=sprintf(buf,"%02x",*data);
    }
    */
    const unsigned char *data1 = (unsigned char *)vptr; 
    union {
      char c[4];
      double f;
    } u;
    u.c[3] = *data1;
    data1--;
    u.c[2] = *data1;
    data1--;
    u.c[1] = *data1;
    data1--;
    u.c[0] = *data1;
   /* Serial.write("Data:");
    for(uint8_t i=0;i<4;i++)
      Serial.write(u.c[3-i]);*/
    //Serial.print(u.f);
    return u.f;
    
    /*Serial.write("Data:");Serial.print(myString);
    sscanf(myString, "%x", &dataresult);
    Serial.print(dataresult);
    signed long long number = strtol( &myString[2], NULL, 16);
    //dataresult=0x3f322e3f;
    reading = *(float*)&number;
    reading = *(float*)&dataresult;
    return reading;*/
}




//EXECUTES LOOP CONTINUESLY
void loop(){
  char fch4[9],ftemp[9],variation;
  float readings;
  uint16_t status_live;
  uint8_t j=0;
  uint8_t vrsn; char serno[25],bdrate[5],senno[12],date[15],time[15],version_fw[10];
  uint16_t bd;
  unsigned char calibration_zero[11];
  memset(fch4,'\0',sizeof(fch4));
  memset(ftemp,'\0',sizeof(ftemp));
  if(pc_cmd_rcvd){
    unsigned char sensor_Cmds[20]={};
    if(strstr(pc_cmd,(char *)"AT+ALIVE=?"))
    {
      Serial.println("+ALIVE:OK");
    }
    if(strstr(pc_cmd,(char *)"MD+LIVE=?"))    //read the live data from sensor CMD: 0x10, 0x13, 0x01, 0x10, 0x1F, 0x00, 0x53
    {  
      uint8_t i=0;
      j=0;
      identifier='l';
      sensor_Cmds[i++]=DLE; 
      sensor_Cmds[i++]=RD;
      sensor_Cmds[i++]=0x01;
      sensor_Cmds[i++]=0x10;
      sensor_Cmds[i++]=EOF1;
      uint16_t crcVal=crcorchecksum(sensor_Cmds,i);
      sensor_Cmds[i++]=((crcVal)>>8);
      sensor_Cmds[i++]=((crcVal&0x00FF));
      for(uint8_t j=0;j<i;j++) {
        Serial1.write(sensor_Cmds[j]);
        //Serial.write(sensor_Cmds[j]);
      }
    }
    if(strstr(pc_cmd,(char *)"MD+SIMPLE=?"))
    {
      uint8_t i=0;
      identifier='s';
      sensor_Cmds[i++]=DLE; 
      sensor_Cmds[i++]=RD;
      sensor_Cmds[i++]=0x06;
      sensor_Cmds[i++]=0x10;
      sensor_Cmds[i++]=EOF1;
      uint16_t crcVal=crcorchecksum(sensor_Cmds,i);
      sensor_Cmds[i++]=((crcVal)>>8);
      sensor_Cmds[i++]=((crcVal&0x00FF));
      for(uint8_t j=0;j<i;j++) {
        Serial1.write(sensor_Cmds[j]);
        //Serial.write(sensor_Cmds[j]);
      }      
    }
    if(strstr(pc_cmd,(char *)"MD+CONFIG=?"))
    {
      uint8_t i=0;
      identifier='c';
      sensor_Cmds[i++]=DLE; 
      sensor_Cmds[i++]=RD;
      sensor_Cmds[i++]=0x00;
      sensor_Cmds[i++]=0x10;
      sensor_Cmds[i++]=EOF1;
      uint16_t crcVal=crcorchecksum(sensor_Cmds,i);
      sensor_Cmds[i++]=((crcVal)>>8);
      sensor_Cmds[i++]=((crcVal&0x00FF));
      for(j=0;j<i;j++) {
        Serial1.write(sensor_Cmds[j]);
        //Serial.write(sensor_Cmds[j]);
      }      
    }
    if(strstr(pc_cmd,(char *)"MD+ZERO=?"))
    {
      uint8_t i=0;
      identifier='z';
      cal_flag=1;    
      sensor_Cmds[i++]=DLE;
      sensor_Cmds[i++]=WR;
      sensor_Cmds[i++]=WP1;
      sensor_Cmds[i++]=WP2;
      sensor_Cmds[i++]=0x02;
      sensor_Cmds[i++]=DLE;
      sensor_Cmds[i++]=EOF1;
      uint16_t crcVal=crcorchecksum(sensor_Cmds,i);
      sensor_Cmds[i++]=((crcVal)>>8);
      sensor_Cmds[i++]=((crcVal&0x00FF));
      for(j=0;j<i;j++) {
        Serial1.write(sensor_Cmds[j]);
        //Serial.write(sensor_Cmds[j]);
      }                  
    }
    if(strstr(pc_cmd,(char *)"MD+SPAN=?"))
    {
      uint8_t i=0;
      identifier='S';
      cal_flag=1;   
      sensor_Cmds[i++]=DLE;
      sensor_Cmds[i++]=WR; 
      sensor_Cmds[i++]=WP1;
      sensor_Cmds[i++]=WP2;
      sensor_Cmds[i++]=0x03;
      sensor_Cmds[i++]=DLE;
      sensor_Cmds[i++]=EOF1;
      uint16_t crcVal=crcorchecksum(sensor_Cmds,i);
      sensor_Cmds[i++]=((crcVal)>>8);
      sensor_Cmds[i++]=((crcVal&0x00FF));
      for(j=0;j<i;j++) {
        Serial1.write(sensor_Cmds[j]);
        //Serial.write(sensor_Cmds[j]);
      }      
    }
    if(strstr(pc_cmd,(char *)"MD+PRIVATE=?"))
    {
      uint8_t i=0;  
      identifier='p';
      sensor_Cmds[i++]=DLE;
      sensor_Cmds[i++]=RD;
      sensor_Cmds[i++]=0x07;
      sensor_Cmds[i++]=DLE;
      sensor_Cmds[i++]=EOF1;
      uint16_t crcVal=crcorchecksum(sensor_Cmds,i);
      sensor_Cmds[i++]=((crcVal)>>8);
      sensor_Cmds[i++]=((crcVal&0x00FF));
      for(j=0;j<i;j++) {
        Serial1.write(sensor_Cmds[j]);
        //Serial.write(sensor_Cmds[j]);
      }                        
    }
    if(strstr(pc_cmd,(char *)"MD+FW=?"))
    {
      uint8_t i=0;
      identifier='f';
      sensor_Cmds[i++]=DLE;
      sensor_Cmds[i++]=RD;
      sensor_Cmds[i++]=0x04;
      sensor_Cmds[i++]=DLE;
      sensor_Cmds[i++]=EOF1;
      uint16_t crcVal=crcorchecksum(sensor_Cmds,i);     
      sensor_Cmds[i++]=((crcVal)>>8);
      sensor_Cmds[i++]=((crcVal&0x00FF));
      for(j=0;j<i;j++) {
        Serial1.write(sensor_Cmds[j]);
        //Serial.write(sensor_Cmds[j]);
      }                        
    }    
    memset(pc_cmd,'\0',sizeof(pc_cmd));
    pc_cmd_rcvd=0;
    pc_cmd_index=0;
  }

  
    
  if(sensor_resp_rcvd) {
    uint16_t crcVal;
    switch(identifier) {
      case 'l': memset(ftemp,'\0',sizeof(ftemp));
                memset(fch4,'\0',sizeof(fch4));
                status_live=(sensor_resp_data[6]<<8);
                status_live|=sensor_resp_data[5];
                readings= ieee754_Conv(&sensor_resp_data[10],4);
                dtostrf(readings,7,4,fch4);   //Converts float data to string              
                fch4[8]='\0';
                readings= ieee754_Conv(&sensor_resp_data[14],4);
                dtostrf(readings,7,4,ftemp);      //Converts Float data to string
                ftemp[8]='\0' ;
                sprintf(tempbuff,"+LIVE:%04x,%s,%s",status_live,fch4,ftemp);
                Serial.println(tempbuff);
                break;
      case 'c'://VERSION
               vrsn=sensor_resp_data[3];
               memset(serno,'\0',sizeof(serno));
               memset(senno,'\0',sizeof(senno));
               //SENSOR TYPE
               for(uint8_t i=5,j=0;i<=12;i++){
                 senno[j++]=sensor_resp_data[i];}
               //SERIAL NUMBER
               for(uint8_t i=63,j=0;i<=72;i++)
                 serno[j++]=(char)sensor_resp_data[i];
               //Baudrate
               switch(vrsn){
                 case 0x01:
                           switch(sensor_resp_data[77]){
                             case 0x01:bd=9600;break;
                             case 0x00:bd=4800;break;
                             case 0x02:bd=19200;break;
                             case 0x03:bd=38400;break;
                           }
                           break;
                 case 0x08:
                           switch(sensor_resp_data[159]){
                             case 0x01:bd=9600;break;
                             case 0x00:bd=4800;break;
                             case 0x02:bd=19200;break;
                             case 0x03:bd=38400;break;
                           }
                           break;
                 default:
                           bd=0x0000;
               }
               sprintf(tempbuff,"+CON:%02x,%s,%d",vrsn,senno,bd);
               Serial.println(tempbuff);
               break;             
      case 's'://VERSION
               readings= ieee754_Conv(&sensor_resp_data[10],4);
               dtostrf(readings,7,4,fch4);
               fch4[8]='\0';
               sprintf(tempbuff,"+SIMPLE:%02x%02x,%s",sensor_resp_data[5],sensor_resp_data[6],fch4);
               Serial.println(tempbuff);    
               break;
      case 'Z'://Zero sensor acknowledgement
               Serial.println("+CAL:success");
               cal_flag=0;
               break;
      case 'z'://Zero sensor writing
               j=0;
               calibration_zero[j++]=0x10;calibration_zero[j++]=0x1A;calibration_zero[j++]=0x00;calibration_zero[j++]=0x10;calibration_zero[j++]=0x1F;calibration_zero[j++]=0x00;calibration_zero[j++]=0x59;
               //calibration_zero[j++]=0x10;calibration_zero[j++]=0x1A;calibration_zero[j++]=0x04;calibration_zero[j++]=0x00;calibration_zero[j++]=0x00;calibration_zero[j++]=0x20;calibration_zero[j++]=0x40;calibration_zero[j++]=0x10;calibration_zero[j++]=0x1F;calibration_zero[j++]=0x00;calibration_zero[j++]=0xBD;
               for(uint8_t i=0;i<j;i++) {
                 Serial1.write(calibration_zero[i]);
                 //Serial.write(calibration_zero[i]);
               }
               cal_flag=2;               
               break; 
      case 'p'://Private Data
               memset(serno,'\0',sizeof(serno));
               switch(sensor_resp_data[3]) {
                  case 0x01:
                            for(uint8_t i=5,j=0;i<=14;i++){
                              serno[j++]=sensor_resp_data[i];
                              }
                            sprintf(tempbuff,"+PRV:%s",serno);
                            Serial.println(tempbuff);
                            break;
                  case 0x05:
                            for(uint8_t i=5,j=0;i<=14;i++){
                              serno[j++]=sensor_resp_data[i];
                              }                            
                            sprintf(tempbuff,"+PRV:%s",serno);
                            Serial.println(tempbuff);
                            break;
                  default:  break;
               }
               
               break;
      case 'S'://Span Data Value 60
               j=0;
               cal_flag=2;               
               calibration_zero[j++]=0x10;
               calibration_zero[j++]=0x1A;
               calibration_zero[j++]=0x04;
               calibration_zero[j++]=0x00;
               calibration_zero[j++]=0x00;
               calibration_zero[j++]=0x70;
               calibration_zero[j++]=0x42;
               calibration_zero[j++]=0x10;
               calibration_zero[j++]=0x1F;
               crcVal=crcorchecksum(calibration_zero,j);
               calibration_zero[j++]=((crcVal)>>8);
               calibration_zero[j++]=((crcVal&0x00FF));
               for(uint8_t i=0;i<j;i++) {
                 Serial1.write(calibration_zero[i]);
                 //Serial.write(calibration_zero[i]);
               }         
               break;
              
      case 'f'://FirmWare Version
               j=0;
               memset(date,'\0',sizeof(date));
               memset(time,'\0',sizeof(time));
               memset(version_fw,'\0',sizeof(version_fw));
               //Reading Date
               for(uint8_t i=3;i<=14;i++){
                 date[j++]=sensor_resp_data[i];
               }
               date[j]='\0';
               j=0;
               //Reading Time
               for(uint8_t i=15;i<=23;i++){
                 time[j++]=sensor_resp_data[i];
               }
               time[j]='\0';
               j=0;
               //Reading Version
               for(uint8_t i=25;i<=32;i++){
                 version_fw[j++]=sensor_resp_data[i];
               }
               version_fw[j]='\0';
               j=0;
               //Reading Variation
               variation= sensor_resp_data[34];              
               sprintf(tempbuff,"+FW:Date:%s,Time:%s,Version:%s,Variation:%c",date,time,version_fw,variation); 
               Serial.println(tempbuff);
               break;

      default :break;
    }
    memset(sensor_resp_data,'\0',sizeof(sensor_resp_data));
    memset(sensor_resp_full,'\0',sizeof(sensor_resp_full));
    sensor_resp_full_index=0;
    sensor_resp_data_index=0;
    sensor_resp_rcvd=0;
  }
}






// Serial Events run in each loop
void serialEvent() {
  if(Serial.available()) {
    // get the new byte:
    char ch = (char)Serial.read(); 
    // add it to the inputString:
    pc_cmd[pc_cmd_index++]=ch;
    if(ch=='?'){
      cal_flag=0;
      pc_cmd_rcvd=1;
      memset(sensor_resp_data,'\0',sizeof(sensor_resp_data));
      memset(sensor_resp_full,'\0',sizeof(sensor_resp_full));
      sensor_resp_full_index=0;
      sensor_resp_data_index=0;
      sensor_resp_rcvd=0;
    }
  }
}
void serialEvent1(){
  if(Serial1.available()) {
    char ch,ch1=0x00;
    // get the new byte:
    if(cal_flag>0)
    {
       while(Serial1.available()){
         ch=Serial1.read();
         if(ch==0x16)
           { 
             ch1=ch;
           }
         delay(5);
        //Serial.write(ch);
       }
       if(ch1==0x16){
         if(cal_flag==1)
           sensor_resp_rcvd=1;
         else{
           identifier='Z';
           sensor_resp_rcvd=1;
         }
       }
       if(ch==0x02||ch==0x09){
         Serial.println("+CAL:Error");
         cal_flag=0;
       }
       /*={0x10, 0x1A, 0x00, 0x10, 0x1F, 0x00, 0x59};*/
    }
    
    else{
    //add it to the inputString:
    ch = (char)Serial1.read();
    //Serial.write(ch);
    sensor_resp_full[sensor_resp_full_index++]=ch;
    sensor_resp_data[sensor_resp_data_index++]=ch;
    if(ch==0x10){
      delay(10);
      ch=(char)Serial1.read();
      //Serial.write(ch);
      if(ch==0x10)
        sensor_resp_full[sensor_resp_full_index++]=ch;
      else{
        sensor_resp_full[sensor_resp_full_index++]=ch;
        sensor_resp_data[sensor_resp_data_index++]=ch;
      }
    }
    if(ch==0x1F){ 
      delay(5);      
      ch = (char)Serial1.read(); 
      //Serial.write(ch);
      sensor_resp_full[sensor_resp_full_index++]=ch;
      sensor_resp_data[sensor_resp_data_index++]=ch;
      delay(5);
      ch = (char)Serial1.read();
      //Serial.write(ch); 
      sensor_resp_full[sensor_resp_full_index++]=ch;
      sensor_resp_data[sensor_resp_data_index++]=ch;
      sensor_resp_rcvd=1;
    }
  }
 }
}
