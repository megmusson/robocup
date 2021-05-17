/*
Filename:       MyNFC.ino
Author:         Julian Murphy
Version:        1.00
Creation Date:  29-05-2017 
Board:          Arduino Mega ADK
IDE Version:    1.81
 
Description:
Read from DFrobot NFC reader, and compare the result to a array of Unique ID's
 
TechnicalInfo:

Revision History: 
29-05-2017 1.00 Initilal release


*/

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#define print1Byte(args) Serial3.write(args)
#define print1lnByte(args)  Serial3.write(args),Serial1.println()
#else
#include "WProgram.h"
#define print1Byte(args) Serial3.print(args,BYTE)
#define print1lnByte(args)  Serial3.println(args,BYTE)
#endif

#define IDArraySize     80

const unsigned long long int IDArean1[]=
{
0x9537105A,0x755A115A,0xC56C115A,0x05850F5A,0xF566115A,
0x856D105A,0xB509115A,0x353C115A,0x155D115A,0x353D115A,
0xB50C105A,0xB56A115A,0xB5F7105A,0x4575115A,0x8574105A,
0x856B115A,0xA5EB105A,0xB551115A,0x5578115A,0x7568105A,
0xD5C4105A,0x857A105A,0x855A115A,0xC550115A,0x9517105A,

0xE550115A,0xC551115A,0x2557115A,0x9568115A,0xE578115A,
0xD5D60F5A,0x756B115A,0xD5C9105A,0xA50C105A,0x7561115A,
0xB5E9105A,0x054C115A,0x8550105A,0x0543115A,0xE5BF0F5A,
0x153E115A,0x7556105A,0xA510105A,0x9555115A,0x7541105A,
0x04900BCAFD4A84,0x04C7F8CAFD4A80,0x04CDF8CAFD4A80,0x00000001,0x00000001, 

0x048804CAFD4A84,0x049406CAFD4A84,0x049E06CAFD4A84,0x04EFF6CAFD4A80,0x042201CAFD4A85,
0x04A304CAFD4A84,0x041101CAFD4A85,0x041D01CAFD4A85,0x04880FCAFD4A84,0x04E205CAFD4A84,
0x042607CAFD4A85,0x041508CAFD4A85,0x04950ACAFD4A84,0x048002CAFD4A84,0x047BF4CAFD4A80,
0x04D206CAFD4A84,0x047F0CCAFD4A84,0x042DFECAFD4A81,0x041500CAFD4A85,0x04C400CAFD4A84,
0x04D9F5CAFD4A80,0x049906CAFD4A84,0x04A406CAFD4A84,0x04890ECAFD4A84,0x04FC01CAFD4A84,

0x04C806CAFD4A84,0x04B90BCAFD4A84,0x0405FFCAFD4A81,0x041FF6CAFD4A81,0x043CF8CAFD4A81

};

const unsigned long long int IDArean2[]=
{
0x00000001,0x00000001,0x00000001,0x00000001,0x00000001,
0x00000001,0x00000001,0x00000001,0x00000001,0x00000001,
0x00000001,0x00000001,0x00000001,0x00000001,0x00000001,
0x00000001,0x00000001,0x00000001,0x00000001,0x00000001,
0x00000001,0x00000001,0x00000001,0x00000001,0x00000001,

0x00000001,0x00000001,0x00000001,0x00000001,0x00000001,
0x00000001,0x00000001,0x00000001,0x00000001,0x00000001,
0x00000001,0x00000001,0x00000001,0x00000001,0x00000001,
0x00000001,0x00000001,0x00000001,0x00000001,0x00000001,
0x00000001,0x00000001,0x00000001,0x00000001,0x00000001, 

0x00000001,0x00000001,0x00000001,0x00000001,0x00000001,
0x00000001,0x00000001,0x00000001,0x00000001,0x00000001,
0x00000001,0x00000001,0x00000001,0x00000001,0x00000001,
0x00000001,0x00000001,0x00000001,0x00000001,0x00000001,
0x00000001,0x00000001,0x00000001,0x00000001,0x00000001,

0x00000001,0x00000001,0x00000001,0x00000001,0x00000001
};

const byte LocationX[]=
{
1,2,3,4,5, 
1,2,3,4,5, 
1,2,3,4,5, 
1,2,3,4,5, 
1,2,3,4,5,
 
1,2,3,4,5, 
1,2,3,4,5, 
1,2,3,4,5, 
1,2,3,4,5,  
1,2,3,4,5,

1,2,3,4,5,
1,2,3,4,5, 
1,2,3,4,5,
1,2,3,4,5,
1,2,3,4,5,

1,2,3,4,5
};

const byte LocationY[]=
{
1,1,1,1,1, 
2,2,2,2,2, 
3,3,3,3,3, 
4,4,4,4,4, 
5,5,5,5,5, 

6,6,6,6,6, 
7,7,7,7,7, 
8,8,8,8,8, 
9,9,9,9,9,        //My test ID's square
10,10,10,10,10,   //My test ID's round

11,11,11,11,11,   //Green Front
12,12,12,12,12,   //Green Side
13,13,13,13,13,   //Blue Front
14,14,14,14,14,   //Blue side 
15,15,15,15,15,   //Ramp up  

16,16,16,16,16    //Ramp top
};

union 
{
  byte IDasByte[8];
  unsigned long long int IDasInt;
} IDDecode;

/* 
NFC command list:

#wake up reader
send: 55 55 00 00 00 00 00 00 00 00 00 00 00 00 00 00 ff 03 fd d4 14 01 17 00
return: 00 00 FF 00 FF 00 00 00 FF 02 FE D5 15 16 00

#read the tag
send: 00 00 FF 04 FC D4 4A 01 00 E1 00
return: 00 00 FF 00 FF 00 00 00 FF 0C F4 D5 4B 01 01 00 04 08 04 XX XX XX XX 5A 00
XX is tag.
*/
const unsigned char wake[24]=
    {0x55,0x55,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
     0x00,0x00,0x00,0x00,0x00,0x00,0xff,0x03,0xfd,0xd4,
     0x14,0x01,0x17,0x00};//wake up NFC module
const unsigned char tag[11]=
    {0x00,0x00,0xFF,0x04,0xFC,0xD4,0x4A,0x01,0x00,0xE1,0x00};//detecting tag command
byte receive_ACK[50];//Command receiving buffer


void setup()
{
  Serial.begin(9600);     //Open terminal serial set Baund rate 9600 bps
  Serial3.begin(115200);  //Open NFC serial port
  SendWakeCard();         //Wake up NFC reader
  delay(50);
  ReceiveNFCData();       //Get back answer from NFC reader
}//setup()


void loop()
{
  byte inByte;                  //Incoming serial byte buffer
  byte x,y;                     //Cordinates of tag
  
   if (Serial.available() > 0)  //Check for serial terminal data
   {
    inByte = Serial.read();     //Get incoming byte from terminal
    if (inByte=='r')            //If r pressed then read a tag
    {                           //Read tag twice to get arround problem
                                // caused by no tag present
      SendReadTag();            //Send read tag command
      delay(50);
      ReceiveNFCData();         //Read tag data
      delay(50);
      ZeroArray();              //Zero incomming array,needed if no tag readed
      SendReadTag();            //Send read tag command
      delay(50);
      ReceiveNFCData();         //Read tag data
      DisplayID();              //Display tag ID
      if(FindMatch(1,x,y))      //Check for match in array
      {
        Serial.print(x);        //If match then print X,Y cordinates
        Serial.print(" ");
        Serial.print(y);
        Serial.println("");
      }
      else
      {
        Serial.println("No Match");  
      }//if
    }//if   
   }//if
}//loop()


void ZeroArray(void)
{
  byte i;
  
  for(i=0;i<50;i++)
  {
    receive_ACK[i]=0;
  }
}//ZeroArray


bool FindMatch(byte Arena,byte &Myx, byte &Myy)
{
  byte j,IDSize;
  unsigned long long int IDLocal;

  IDSize=receive_ACK[18];
  if(IDSize==4)             //Square tags
  {
    IDDecode.IDasByte[0]=receive_ACK[22];
    IDDecode.IDasByte[1]=receive_ACK[21];
    IDDecode.IDasByte[2]=receive_ACK[20];
    IDDecode.IDasByte[3]=receive_ACK[19];
    IDDecode.IDasByte[4]=0;
    IDDecode.IDasByte[5]=0;
    IDDecode.IDasByte[6]=0;
    IDDecode.IDasByte[7]=0;
  }
  else                      //IDSize==7 Round tags larger unique ID
  {
    IDDecode.IDasByte[0]=receive_ACK[25];
    IDDecode.IDasByte[1]=receive_ACK[24];
    IDDecode.IDasByte[2]=receive_ACK[23];
    IDDecode.IDasByte[3]=receive_ACK[22];
    IDDecode.IDasByte[4]=receive_ACK[21];
    IDDecode.IDasByte[5]=receive_ACK[20];
    IDDecode.IDasByte[6]=receive_ACK[19];
    IDDecode.IDasByte[7]=0;
  }
  for(j=0;j<IDArraySize;j++)  //Search through tag array for match
  {
    if(Arena==1)              //Arena array selection
      IDLocal=IDArean1[j];
    else
      IDLocal=IDArean2[j];
    if((long long)IDDecode.IDasInt==(long long)IDLocal)//If they match return xy cordinate
    {
      Myx=LocationX[j];
      Myy=LocationY[j];
      return(true);
    }
  }//for j
  Myx=0;                        //If no match return false and set X Y to zero
  Myy=0;
  return(false);
}//FindMatch()


void UART1_Send_Byte(byte command_data)
{
  print1Byte(command_data);// command send to device
  #if defined(ARDUINO) && ARDUINO >= 100
    Serial3.flush();// complete the transmission of outgoing serial data 
  #endif
}//UART1_Send_Byte()
 

void UART_Send_Byte(byte command_data)//, unsigned charBYTE
{
  if(command_data<16)Serial.print("0");
  Serial.print(command_data,HEX);
}//UART_Send_Byte() 


void ReceiveNFCData(void)
{
  byte i;

  i=0;                             //Set array index to zero
  while(Serial3.available()>0)     //Read until no data left to read
  {
   receive_ACK[i]= Serial3.read(); //Store result in array
   i++;                           
  }//while()
}//array_ACK()


void SendWakeCard(void)
{
  byte i;
  
  for(i=0;i<24;i++) //send command
    UART1_Send_Byte(wake[i]);
}//SendWakeCard()


void SendReadTag(void)
{
  byte i;
  
  for(i=0;i<11;i++) //send command
    UART1_Send_Byte(tag[i]);
}//SendReadTag()


void DisplayID(void)
{
  byte i,IDSize;

  IDSize=receive_ACK[18];
  
  if(IDSize==4)     //Square tags
  {
    for(i=19;i<23;i++) //send command
      UART_Send_Byte(receive_ACK[i]);  
  }
  else
  {
    for(i=19;i<26;i++) //send command
      UART_Send_Byte(receive_ACK[i]);  
  }
  Serial.println();
}//dsplay()

