// Demo: NMEA2000 library.
// This demo reads messages from NMEA 2000 bus and
// sends them translated to clear text to Serial.

// Note! If you use this on Arduino Mega, I prefer to also connect interrupt line
// for MCP2515 and define N2k_CAN_INT_PIN to related line. E.g. MessageSender
// sends so many messages that Mega can not handle them. If you only use
// received messages internally without slow operations, then youmay survive
// without interrupt.

#include <Arduino.h>
#include "/home/pjakobs/Arduino/libraries/NMEAParser/src/NMEAParser.h"
#include <NMEA2000_CAN.h>
#include <N2kMessages.h>
#include <N2kMessagesEnumToStr.h>
#include <BluetoothSerial.h>
//#include <BTAddress.h>
//#include <BTAdvertisedDevice.h>
//#include <BTScan.h>

//#define N2k_SPI_CS_PIN 53    // Pin for SPI select for mcp_can
//#define N2k_CAN_INT_PIN 21   // Interrupt pin for mcp_can
//#define USE_MCP_CAN_CLOCK_SET 8  // Uncomment this, if your mcp_can shield has 8MHz chrystal
//#define ESP32_CAN_TX_PIN GPIO_NUM_16 // Uncomment this and set right CAN TX pin definition, if you use ESP32 and do not have TX on default IO 16
//#define ESP32_CAN_RX_PIN GPIO_NUM_17 // Uncomment this and set right CAN RX pin definition, if you use ESP32 and do not have RX on default IO 4
//#define NMEA2000_ARDUINO_DUE_CAN_BUS tNMEA2000_due::CANDevice1    // Uncomment this, if you want to use CAN bus 1 instead of 0 for Arduino DUE
#define ENGINE_RPM_INPUT 36
#define RPMUpdatePeriod 250
#define LED_BUILTIN 2

typedef struct {
  unsigned long PGN;
  void (*Handler)(const tN2kMsg &N2kMsg);
} tNMEA2000Handler;

const char crlf[] = "\n\r";

void SystemTime(const tN2kMsg &N2kMsg);
void Rudder(const tN2kMsg &N2kMsg);
void EngineRapid(const tN2kMsg &N2kMsg);
void EngineDynamicParameters(const tN2kMsg &N2kMsg);
void TransmissionParameters(const tN2kMsg &N2kMsg);
void TripFuelConsumption(const tN2kMsg &N2kMsg);
void Speed(const tN2kMsg &N2kMsg);
void WaterDepth(const tN2kMsg &N2kMsg);
void BinaryStatus(const tN2kMsg &N2kMsg);
void FluidLevel(const tN2kMsg &N2kMsg);
void OutsideEnvironmental(const tN2kMsg &N2kMsg);
void Temperature(const tN2kMsg &N2kMsg);
void TemperatureExt(const tN2kMsg &N2kMsg);
void DCStatus(const tN2kMsg &N2kMsg);
void BatteryConfigurationStatus(const tN2kMsg &N2kMsg);
void COGSOG(const tN2kMsg &N2kMsg);
void GNSS(const tN2kMsg &N2kMsg);
void LocalOffset(const tN2kMsg &N2kMsg);
void Attitude(const tN2kMsg &N2kMsg);
void Heading(const tN2kMsg &N2kMsg);
void Humidity(const tN2kMsg &N2kMsg);
void Pressure(const tN2kMsg &N2kMsg);
void UserDatumSettings(const tN2kMsg &N2kMsg);
void GNSSSatsInView(const tN2kMsg &N2kMsg);
void Wind(const tN2kMsg &N2kMsg);

NMEAParser<4> NMEAparser;
BluetoothSerial SerialBT;
Stream *OutputStream;

tNMEA2000Handler NMEA2000Handlers[] = {
  //{126992L, &SystemTime},
  //{127245L, &Rudder },
  //{127250L, &Heading},
  //{127257L, &Attitude},
  //{127488L, &EngineRapid},
  //{127489L, &EngineDynamicParameters},
  //{127493L, &TransmissionParameters},
  //{127497L, &TripFuelConsumption},
  //{127501L, &BinaryStatus},
  //{127505L, &FluidLevel},
  //{127506L, &DCStatus},
  //{127513L, &BatteryConfigurationStatus},
  //{128259L, &Speed},
  //{128267L, &WaterDepth},
  //{129026L, &COGSOG},
  //{129029L, &GNSS},
  //{129033L, &LocalOffset},
  //{129045L, &UserDatumSettings},
  //{129540L, &GNSSSatsInView},
  //{130310L, &OutsideEnvironmental},
  //{130312L, &Temperature},
  //{130314L, &Pressure},
  //{130316L, &TemperatureExt},
  //{130306L, &Wind},
  {0, 0}
};


const unsigned long TransmitMessages[] PROGMEM={127488L,0};
bool BTconnected;
uint8_t address[6]  = {0x4C, 0xEB, 0xD6, 0x61, 0x4C, 0x3A};
String BTname="AntaresMasthead";

void HandleNMEA2000Msg(const tN2kMsg &N2kMsg);

void setup() {
  pinMode(ENGINE_RPM_INPUT,INPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  
  Serial.begin(115200); 
  delay(500);
  OutputStream = &Serial;

  OutputStream->print("------->starting Bluetooth\n");
  SerialBT.register_callback(BTcallback);
  if(SerialBT.begin("AntaresBase",true)){
    OutputStream->print("------->initialized Bluetooth\n");
  }
  
  OutputStream->print("------->connecting Bluetooth\n"); 
  if(BTconnected=SerialBT.connect(BTname)){
 // if(BTconnected=SerialBT.connect(address)){
    OutputStream->print("------->starting bt connect\n");
  }
  delay(500);
  
  
  // Set Product information
  NMEA2000.SetProductInformation("00000002", // Manufacturer's Model serial code
                                 291, // Manufacturer's product code
                                 "Data Display and nmea Gateway",  // Manufacturer's Model ID
                                 "0.0.0.1 (2022-01-10)",  // Manufacturer's Software version code
                                 "0.0.0.1 (2022-01-03)" // Manufacturer's Model version
                                 );
  // Set device information
  NMEA2000.SetDeviceInformation(2, // Unique number. Use e.g. Serial number.
                                130, // Device function=Atmospheric. See codes on http://www.nmea.org/Assets/20120726%20nmea%202000%20class%20&%20function%20codes%20v%202.00.pdf
                                 85, // Device class=External Environment. See codes on  http://www.nmea.org/Assets/20120726%20nmea%202000%20class%20&%20function%20codes%20v%202.00.pdf
                                135, //nmea 0183 gateway
                                120, //display
                                2046 // Just choosen free from code list on http://www.nmea.org/Assets/20121020%20nmea%202000%20registration%20list.pdf                               
                               );

  // Wind Sentence
  // WIMWV,ddd,R,xx.x,N,A*cc 
  // ddd-> relative direction
  // xx.x-> speed in m/s
  // cc-> checksum
  NMEAparser.addHandler("WIMWV",handleWind);
  NMEAparser.setErrorHandler(errorHandler);
  NMEAparser.setDefaultHandler(defaultHandler);

  //  NMEA2000.SetN2kCANReceiveFrameBufSize(50);
  // Do not forward bus messages at all
  NMEA2000.SetForwardType(tNMEA2000::fwdt_Text);
  NMEA2000.SetForwardStream(OutputStream);
  NMEA2000.SetMode(tNMEA2000::N2km_ListenAndNode,23);

  // Set false below, if you do not want to see messages parsed to HEX withing library
  NMEA2000.EnableForward(false);
  NMEA2000.SetMsgHandler(HandleNMEA2000Msg);
  //  NMEA2000.SetN2kCANMsgBufSize(2)
  NMEA2000.ExtendTransmitMessages(TransmitMessages);

  NMEA2000.Open();
  OutputStream->print("Running...");
}

//*****************************************************************************
template<typename T> void PrintLabelValWithConversionCheckUnDef(const char* label, T val, double (*ConvFunc)(double val) = 0, bool AddLf = false, int8_t Desim = -1 ) {
  OutputStream->print(label);
  if (!N2kIsNA(val)) {
    if ( Desim < 0 ) {
      if (ConvFunc) {
        OutputStream->print(ConvFunc(val));
      } else {
        OutputStream->print(val);
      }
    } else {
      if (ConvFunc) {
        OutputStream->print(ConvFunc(val), Desim);
      } else {
        OutputStream->print(val, Desim);
      }
    }
  } else OutputStream->print("not available");
  if (AddLf) OutputStream->println();
}

void BTcallback(esp_spp_cb_event_t event, esp_spp_cb_param_t *param){
  char c;
  switch(event){
    case ESP_SPP_INIT_EVT:
      OutputStream->print("------->BT init");
    case ESP_SPP_DISCOVERY_COMP_EVT:
      OutputStream->print("------->BT discovery completed\n");
      break;
    case ESP_SPP_SRV_OPEN_EVT:
      OutputStream->print("------->BT Connected\n");
      break;
    case ESP_SPP_DATA_IND_EVT:
      while(SerialBT.available()){
        c=SerialBT.read();
        /*if (c=='$'){
          NMEAparser<<crlf[0];
          //OutputStream->print(crlf[0]);
          NMEAparser<<crlf[1];
          //OutputStream->print(crlf[1]);
        }
        */
        //OutputStream->print(c);
        NMEAparser << c;
        //OutputStream->printf(" state: %i\n",NMEAparser.getState());
      }
      break;
      case ESP_SPP_CLOSE_EVT:
        SerialBT.connect();
      break;
    default:
      OutputStream->printf("------->BT Event %i\n",event);
  }
}

void defaultHandler(){
  OutputStream->print("*** default Handler called ***\n");
}

void errorHandler(){
  char* buff;
  OutputStream->printf("*** Error : %i\n",NMEAparser.error());
  //buff=NMEAparser.getBuffer();
  //OutputStream->print("[");
  //for (int i=0;i<32;i++){
    //OutputStream->print(buff[i]);
 // }
  //OutputStream->println("]");
  //OutputStream->printf("in state %i\n",NMEAparser.getState());
}

void handleWind(void) {
  //OutputStream->println("------->NMEA0183 wind sentence");
  int windAngle;
  float windSpeed;
  tN2kMsg N2kMsg;
  NMEAparser.getArg(0,windAngle);
  NMEAparser.getArg(2,windSpeed);
  //OutputStream->printf("------> got Data: Speed:%fm/s, Angle: %i°",windSpeed,windAngle);
  SetN2kWindSpeed(N2kMsg, 1, msToKnots(windSpeed), DegToRad(windAngle),N2kWind_Apparent);
  NMEA2000.SendMsg(N2kMsg);
}

void SendN2kRPM() {
  static unsigned long RPMUpdated=millis();
  tN2kMsg N2kMsg;

  if ( RPMUpdated+RPMUpdatePeriod<millis() ) {
    SetN2kEngineParamRapid(N2kMsg, (unsigned char)0, getEngineRev(),(double)0, (int8_t)0 );
    RPMUpdated=millis();
    NMEA2000.SendMsg(N2kMsg);
  }
}


//*****************************************************************************
void EngineRapid(const tN2kMsg &N2kMsg) {
  unsigned char EngineInstance;
  double EngineSpeed;
  double EngineBoostPressure;
  int8_t EngineTiltTrim;

  if (ParseN2kEngineParamRapid(N2kMsg, EngineInstance, EngineSpeed, EngineBoostPressure, EngineTiltTrim) ) {
    PrintLabelValWithConversionCheckUnDef("Engine rapid params: ", EngineInstance, 0, true);
    PrintLabelValWithConversionCheckUnDef("  RPM: ", EngineSpeed, 0, true);
    PrintLabelValWithConversionCheckUnDef("  boost pressure (Pa): ", EngineBoostPressure, 0, true);
    PrintLabelValWithConversionCheckUnDef("  tilt trim: ", EngineTiltTrim, 0, true);
  } else {
    OutputStream->print("Failed to parse PGN: "); OutputStream->println(N2kMsg.PGN);
  }
}



//*****************************************************************************
void printLLNumber(Stream *OutputStream, unsigned long long n, uint8_t base = 10)
{
  unsigned char buf[16 * sizeof(long)]; // Assumes 8-bit chars.
  unsigned long long i = 0;

  if (n == 0) {
    OutputStream->print('0');
    return;
  }

  while (n > 0) {
    buf[i++] = n % base;
    n /= base;
  }

  for (; i > 0; i--)
    OutputStream->print((char) (buf[i - 1] < 10 ?
                                '0' + buf[i - 1] :
                                'A' + buf[i - 1] - 10));
}



//*****************************************************************************
//NMEA 2000 message handler
void HandleNMEA2000Msg(const tN2kMsg &N2kMsg) {
  int iHandler;

  // Find handler
  //OutputStream->print("In Main Handler: "); OutputStream->println(N2kMsg.PGN);
  for (iHandler = 0; NMEA2000Handlers[iHandler].PGN != 0 && !(N2kMsg.PGN == NMEA2000Handlers[iHandler].PGN); iHandler++);

  if (NMEA2000Handlers[iHandler].PGN != 0) {
    NMEA2000Handlers[iHandler].Handler(N2kMsg);
  }
}

//*****************************************************************************
//Engine rev measurement
double getEngineRev(){
  int revs;
  revs=(double)analogRead(ENGINE_RPM_INPUT); 
  //OutputStream->print(" ===RPM=== :"); OutputStream->println(revs);
  return revs;
}


//*****************************************************************************
void loop()
{

  NMEA2000.ParseMessages();
  SendN2kRPM();
  if(!SerialBT.connected()){
    OutputStream->println("BT not connected");
    SerialBT.connect();
  }
  
}
