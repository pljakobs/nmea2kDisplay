#include <EasyNextionLibrary.h>
#include <trigger.h>

#define LED_BUILTIN 13
uint32_t v;
int8_t j_1,j_2,j_3,j_4;
EasyNex myNex(Serial1);
uint32_t WindDir,WindSPD, BoatSPD, BoatTRK;
void setup(){
  Serial.begin(115200);
  //Serial1.begin(115200);
  digitalWrite(LED_BUILTIN, LOW);
  delay(500);
  digitalWrite(LED_BUILTIN, HIGH);
  Serial.println("hello world");
  
  myNex.begin(115200);
  WindDir,WindSPD, BoatSPD, BoatTRK=0;
  j_1=j_2=j_3=j_4=1;
}

void loop() {
  //delay(10);
  WindDir+=j_1;
  if(WindDir==360|WindDir==0) j_1=-j_1;
  WindSPD+=j_2;
  if(WindSPD==20|WindSPD==0) j_2=-j_2;
  BoatSPD+=j_3;
  if(BoatSPD==10|BoatSPD==0) j_3=-j_3;
  BoatTRK+=j_4;
  if(BoatTRK==360|BoatTRK==0) j_4=-j_4;
  Serial.printf("WindDir: %i, WindSPD: %i, BoadTRK: %i, BoadSPD: %i\n",WindDir, WindSPD, BoatTRK, BoatSPD);
  myNex.writeNum("gWind.val",WindDir);
  myNex.writeNum("vDirW.val",WindDir);
  myNex.writeNum("vSpeedW.val",WindSPD);
  myNex.writeNum("vSpeedB.val",BoatSPD);
  myNex.writeNum("vTrkB.val",BoatTRK);
  myNex.writeNum("wWind.val",WindSPD);
  myNex.writeNum("wBoat.val",BoatSPD);
  
  digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  
}
