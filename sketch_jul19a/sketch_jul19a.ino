#include <SPI.h>
#include <WiFi.h>
#include <Wire.h>
#include <esp_now.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

//-------------------------------------|DISPLAY|-------------------------------------
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
// The pins for I2C are defined by the Wire-library. 
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 Display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

//---------------------------------------|ESP|---------------------------------------
const uint8_t SLAVES_MAC_ADDRESS[][6] = {
  { 0xCC, 0xDB, 0xA7, 0x30, 0x46, 0x34 }  //Mac address do ESP32 do drone
};
const unsigned int CHANNEL = 1;  //Canal do slave
const unsigned int AMOUNT_OF_SLAVES = sizeof(SLAVES_MAC_ADDRESS) / 6; //Quantidade de ESP Escravos, os que vão receber dados

//-------------------------------------|CONTROL|-------------------------------------
//DOF = Degrees Of Freedom
//J1x, J1y, J1z, J2x, J2y, J2z
const unsigned int PIN_YAW = 35;
const unsigned int PIN_THROTTLE = 34;
const unsigned int PIN_J1Z = 25;
const unsigned int PIN_ROLL = 33;
const unsigned int PIN_PITCH = 32;
const unsigned int PIN_J2Z = 26;

const unsigned int *PT_PINS_XY_JS[] = {&PIN_YAW, &PIN_THROTTLE, &PIN_ROLL, &PIN_PITCH};
const int MIN_SETPOINTS[] = {-30, 950, -30, -30};
const int MID_SETPOINTS[] = {0, 1525, 0, 0};
const int MAX_SETPOINTS[] = {30, 2000, 30, 30};
unsigned int maxRangeOfJoystickAxes[] = {0, 0, 0, 0};
unsigned int midRangeOfJoystickAxes[] = {0, 0, 0, 0};
unsigned int minRangeOfJoystickAxes[] = {UINT_MAX, UINT_MAX, UINT_MAX, UINT_MAX};
const unsigned int JOYSTICK_AXIS_DEAD_ZONE_RATES[] = {10, 10, 10, 10};
unsigned int joystickAxisDeadZones[] = {0, 0, 0, 0};
int SetPoints[] = {0, 0, 0, 0};

//----------------------------------|COMUNICATION|-----------------------------------
struct Package {
    unsigned int dof[3];
    unsigned int throttle;
    volatile bool flightMode;
};

Package package = {{4, 7, 5}, 1240, true};

void startSerial(){
  Serial.begin(115200);
}

void configDisplay(){
  Display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS);
  Display.setCursor(0,0);
  Display.setTextSize(1);
  Display.setTextColor(SSD1306_WHITE);
  Display.clearDisplay();
}

void printOnDisplay(String text, bool resetCursor = NULL, int delayTime = 0, bool clearDisplay = NULL){
  if (clearDisplay) Display.clearDisplay();
  if (resetCursor) Display.setCursor(0,0);
  Display.println(text);
  Display.display();
  if (delayTime) delay(delayTime);
}

template<typename T, int N>
int getArraySize(T (&array)[N]) {
  return N;
}

void invertFlightMode(){
  package.flightMode = !package.flightMode;
}

void configJoystick(){
  printOnDisplay("Configurando joysticks", true, 1000, true);
  for (int i = 0; i < getArraySize(PT_PINS_XY_JS); i++) {
    pinMode(*PT_PINS_XY_JS[i], INPUT);
  }

  pinMode(PIN_J1Z, INPUT_PULLUP);
  pinMode(PIN_J2Z, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(PIN_J1Z), invertFlightMode, FALLING);
}

void stopUntilPressToContinue(String text){
  printOnDisplay(text);
  
  while(digitalRead(PIN_J2Z));
  while(!digitalRead(PIN_J2Z));
}

void getRangeOfJoystickAxes(){
  printOnDisplay("Mova os joysticks em todas as direções", true, 0, true);

  stopUntilPressToContinue("Aperte R3 para prosseguir");

  int value = 0;

  do{
    Display.clearDisplay();
    Display.setCursor(0,0);

    for (int i = 0; i < getArraySize(PT_PINS_XY_JS); i++) {
      value = analogRead(*PT_PINS_XY_JS[i]);
      if(value > maxRangeOfJoystickAxes[i]){
        maxRangeOfJoystickAxes[i] = value;
      } else if (value < minRangeOfJoystickAxes[i]) {
        minRangeOfJoystickAxes[i] = value;
      }

      printOnDisplay(String(*PT_PINS_XY_JS[i]) + ": " + String(minRangeOfJoystickAxes[i]) + " - " + String(maxRangeOfJoystickAxes[i]));
    }
  }while(digitalRead(PIN_J2Z));
}

void getAverageOfJoystickAxes(){
  printOnDisplay("Calculando o meio dos eixos dos joysticks", true, 0, true);

  int max = 3000;
  for (int i = 0; i < max; i++) {
    for (int j = 0; j < getArraySize(PT_PINS_XY_JS); j++) {
      midRangeOfJoystickAxes[j] = midRangeOfJoystickAxes[j] + analogRead(*PT_PINS_XY_JS[j]);
    }
    // delayMicroseconds(20);
  }

  for (int i = 0; i < 4; i++) {
    midRangeOfJoystickAxes[i] = midRangeOfJoystickAxes[i]/max;
    printOnDisplay(String(*PT_PINS_XY_JS[i]) + ": " + String(midRangeOfJoystickAxes[i]));
  }
}

void getDeadZonesOfJoystickAxes(){
  printOnDisplay("Deadzone dos eixos dos joysticks", true, 0, true);

  for (int i = 0; i < 4; i++) {
    joystickAxisDeadZones[i] = JOYSTICK_AXIS_DEAD_ZONE_RATES[i] * midRangeOfJoystickAxes[i] / 100;

    printOnDisplay(String(*PT_PINS_XY_JS[i]) + ": " + String(midRangeOfJoystickAxes[i] - joystickAxisDeadZones[i]) + " - " + String(midRangeOfJoystickAxes[i] + joystickAxisDeadZones[i]));
  }
}

void StartStationMode() {
  WiFi.mode(WIFI_STA);

  printOnDisplay("Inicializando o modo station", true, 0, true);
  printOnDisplay("Mac Address desse ESP em Station: " + WiFi.macAddress());
}

void StartEspNow() {
  bool Result = esp_now_init() == ESP_OK;

  String message = Result ? "ESPNow iniciado com sucesso." : "Não foi possível iniciar o ESPNow.";
  
  printOnDisplay(message, true, 0, true);

  if (!Result) {ESP.restart();}
}

void SendDataEspDrone() {
  esp_now_send(SLAVES_MAC_ADDRESS[0], (uint8_t *) &package, sizeof(package));
}

void CalculatePackageValues() {
  for (int i = 0; i < getArraySize(PT_PINS_XY_JS); i++) {  
    SetPoints[i] = analogRead(*PT_PINS_XY_JS[i]);
    SetPoints[i] = constrain(SetPoints[i], minRangeOfJoystickAxes[i], maxRangeOfJoystickAxes[i]);
    if (SetPoints[i] <= midRangeOfJoystickAxes[i] - joystickAxisDeadZones[i]) {
      SetPoints[i] = map(SetPoints[i], minRangeOfJoystickAxes[i], midRangeOfJoystickAxes[i] - joystickAxisDeadZones[i], MIN_SETPOINTS[i], MID_SETPOINTS[i]);
    } else if (SetPoints[i] >= midRangeOfJoystickAxes[i] + joystickAxisDeadZones[i]) {
      SetPoints[i] = map(SetPoints[i], midRangeOfJoystickAxes[i] + joystickAxisDeadZones[i], maxRangeOfJoystickAxes[i], MID_SETPOINTS[i], MAX_SETPOINTS[i]);
    } else {
      SetPoints[i] = MID_SETPOINTS[i];
    }
  }
}

void WhenReceivingResponseDo(const uint8_t *mac_addr,  esp_now_send_status_t response) {
  SendDataEspDrone();
}

void RegisterFunctionThatExecutesWhenReceivingData(){
  esp_now_register_send_cb(WhenReceivingResponseDo);
}

void EstablishConnectionBetweenESPs(){
  printOnDisplay("Configurando Master (Mestre) e Escravos (Slaves)...", true, 500, true);

  for (int i = 0; i < AMOUNT_OF_SLAVES; i++) {
    esp_now_peer_info_t Slave = {};
    Slave.channel = CHANNEL;
    Slave.encrypt = 0;
    memcpy(Slave.peer_addr, SLAVES_MAC_ADDRESS[i], sizeof(SLAVES_MAC_ADDRESS[i]));
    esp_now_add_peer(&Slave);
  }
}

void setup() {
  startSerial();
  configDisplay();
  configJoystick();
  getRangeOfJoystickAxes();
  getAverageOfJoystickAxes();
  getDeadZonesOfJoystickAxes();
  StartStationMode();
  StartEspNow();
  RegisterFunctionThatExecutesWhenReceivingData();
  EstablishConnectionBetweenESPs();
  SendDataEspDrone();
}

void loop() {
  // for (int i = 0; i < getArraySize(PT_PINS_XY_JS); i++) {
  //   Serial.print(analogRead(*PT_PINS_XY_JS[i]));
  //   Serial.print("\t");
  // }

  // Serial.print(digitalRead(PIN_J1Z));
  // Serial.print("\t");

  // Serial.print(digitalRead(PIN_J2Z));
  // Serial.print("\t");

  // Serial.println();
}