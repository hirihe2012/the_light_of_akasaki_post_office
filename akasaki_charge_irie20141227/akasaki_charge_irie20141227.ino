#include<stdlib.h>

#define limit_high_cut 14.8
#define limit_high_reconnect 13.0
#define limit_low_cut 9.0
#define limit_low_reconnect 11.0 

#define sw_in_pin 13   // original 1
#define sw_out_pin 12



double vol_b,vol_p;
bool sw_in,sw_out;
bool flag;

void setup() {
  pinMode(sw_in_pin,OUTPUT);
   digitalWrite(sw_in_pin,LOW);
  pinMode(sw_out_pin,OUTPUT);
   digitalWrite(sw_out_pin,LOW);

  Serial.begin(9600);
    sw_in = sw_out = true;//フラグ建設
//    flag = 1;
}

void loop() {
 
  analogRead(0); delay(10);
  analogRead(0); delay(10);
  vol_b =(4.98 * analogRead(0) ) / 1023.0  * (9.05 / 2.030)+0.746; //0.746　は　保護ダイオードの順方向電圧
 // 4.98は内部のレギュレータの電圧（実測）  9.05と2.03は、VinとA0の実測時
 
  //条件判断：
  //充電回路スイッチ
  //充電止め
  if(vol_b > limit_high_cut)
  
    sw_in = false; 
  //充電開始
  else if(vol_b < limit_high_reconnect)
    sw_in = true;

  //放電回路スイッチ
  //放電止め
  if(vol_b < limit_low_cut )
    sw_out = false;
  //放電開始
  else if(vol_b > limit_low_reconnect)
    sw_out = true;

    //回路のスイッチ制御
    if(sw_in == true)
      digitalWrite(sw_in_pin,LOW); //リレーはN.C. 通常は太陽電池とバッテリーは接続
    else
      digitalWrite(sw_in_pin,HIGH); //電圧が上がるとリレーをONする。

    if(sw_out == true)
      digitalWrite(sw_out_pin,HIGH);//リレーはN.O. 電圧が適正の時はON
    else
      digitalWrite(sw_out_pin,LOW); //電圧が下がると、リレーをOFFする。
      
      
      Serial.print("vol_b=");
      Serial.print(vol_b);
      
      Serial.print(" SW(Solar)  is  ");
     if(sw_in) Serial.print("ON");else Serial.print("OFF");
      
     Serial.print(", SW(Load)  is  ");
     if(sw_out) Serial.print("ON");else Serial.print("OFF");
      
     Serial.println("");   
     
      delay(5000);
}
