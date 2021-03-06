// link between the computer and the SoftSerial Shield
//at 9600 bps 8-N-1
//Computer is connected to Hardware UART
//SoftSerial Shield is connected to the Software UART:D2&D3

#include <SoftwareSerial.h>

SoftwareSerial SoftSerial(2, 3);
unsigned char buffer[128];                   // buffer array for data recieve over serial port
int count=0;                                // counter for buffer array

void setup()
{
    SoftSerial.begin(9600);                 // the SoftSerial baud rate
    Serial.begin(9600);                     // the Serial port of Arduino baud rate.
}

void loop()
{
  char gpsc;
    if (SoftSerial.available())                     // if date is comming from softwareserial port ==> data is comming from SoftSerial shield
    {
        while(SoftSerial.available())               // reading data into char array
        {
           gpsc=SoftSerial.read();
            buffer[count++] =gpsc;     // writing data into array
            if(count == 128)break;
            if(gpsc == 0x0c)break;
        }
        Serial.write(buffer,count);                 // if no data transmission ends, write buffer to hardware serial port
        clearBufferArray();                         // call clearBufferArray function to clear the storaged data from the array
        count = 0;                                  // set counter of while loop to zero


    }
    if (Serial.available())                 // if data is available on hardwareserial port ==> data is comming from PC or notebook
    SoftSerial.write(Serial.read());        // write it to the SoftSerial shield
}

void clearBufferArray()                     // function to clear buffer array
{
    for (int i=0; i<count;i++)
    { buffer[i]=NULL;}                      // clear all index of array with command NULL
}
