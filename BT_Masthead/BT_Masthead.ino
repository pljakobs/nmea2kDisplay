//This example code is in the Public Domain (or CC0 licensed, at your option.)
//By Evandro Copercini - 2018
//
//This example creates a bridge between Serial and Classical Bluetooth (SPP)
//and also demonstrate that SerialBT have the same functionalities of a normal Serial

#include "BluetoothSerial.h"

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

BluetoothSerial SerialBT;
char sentence[80];
typedef enum {OUT, IN, FIN1, FIN2 } state_t;
int i;

state_t state;
void setup() {
 state=OUT;
 Serial.begin(4800);
 SerialBT.begin("AntaresMasthead"); //Bluetooth device name // <------- set this to be the same as the name you chose above!!!!!
 //Serial.println("The device started, now you can pair it with bluetooth!");
}

void loop() {
  char c;
 
  if (Serial.available()) {
    c=Serial.read();
    switch(c){
      case '$':
        if(state==OUT){
            state=IN;
         }else{
            sentence[i++]=c;
         }
         break;
      case 0x0d:
        if(state==IN){
          state=FIN1;
        }
        break;
      case 0x0a:
        if(state==FIN1){
          state=FIN2;
          sentence[i++]=0x0;
          //SerialBT.printf("got %i bytes",i);
          SerialBT.printf("$%s\r\n",&sentence);
          i=0;
          state=OUT;
        }
        break;
      default:
        //SerialBT.printf("got char: %c",c);
        sentence[i++]=c;
    }
  }
}
