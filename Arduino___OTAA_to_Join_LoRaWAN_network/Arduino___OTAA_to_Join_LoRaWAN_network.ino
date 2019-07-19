#include <SoftwareSerial.h>
SoftwareSerial Serial1(2, 3); 

const char *error=" \r\n";
const char *CRE=" AT+CRESTORE\r\n";
const char *join=" AT+CJOIN=1,1,10,8\r\n";
char dtrx[16]=" AT+DTRX=0,0,";
char flags=0;
//char flag1=0;

void setup() {
  // initialize both serial ports:
  Serial1.begin(115200);
  Serial.begin(115200);
  while (!Serial1) {
     ;
  };

}

 void data()
{
    char dtrx_data[40]={"\0"};
    char Length[4]="10";              //uplink character number
    char Payload[12]="0123456789";    //uplink character,you define for yourself
    strcat(dtrx_data,dtrx);
    strcat(dtrx_data,Length); 
    strcat(dtrx_data,",");  
    strcat(dtrx_data,Payload);   
    strcat(dtrx_data,"\r\n\0"); 
    Serial1.write((char *)dtrx_data,40); //40 is uplink character number
    delay(30000);                         //sampling interval
 }
  
void loop() {
    if(flags==0)
    {
    delay(1000);  
    Serial1.write(error);
    delay(1000); 
    Serial1.write(CRE); 
    delay(4000);         
    Serial1.write(join);
    delay(1000); 
    Serial1.write(join);        
    delay(15000); 
    flags=1;
    }
// Arduino serial no use AT command
    if(flags==1)
    {
      data();
     }
//  Arduino serial use AT command 
//   char flags1=0;              
//   while (Serial.available()>0) {
//    if(flags1==0)
//    {
//   Serial1.write(' ');
//   flags1=1;
//    }
//    else
//    {
//    char inByte = Serial.read();
//    Serial1.write(inByte);
//      }
//  }
//  if (Serial.available()) {
//    int inByte = Serial.read();
//    Serial1.write(inByte);
//  }
}
