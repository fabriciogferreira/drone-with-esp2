#include <SPI.h>
#include <Wire.h>
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

//-------------------------------------|CONTROL|-------------------------------------
//DOF = Degrees Of Freedom
//J1x, J1y, J1z, J2x, J2y, J2z
const int PIN_YAW = 35;
const int PIN_THROTTLE = 34;
const int PIN_J1Z = 25;
const int PIN_ROLL = 33;
const int PIN_PITCH = 32;
const int PIN_J2Z = 26;

const int *PT_PINS_XY_JS[] = {&PIN_YAW, &PIN_THROTTLE, &PIN_ROLL, &PIN_PITCH};
int maxRangeOfJoystickAxes[] = {0, 0, 0, 0};
int midRangeOfJoystickAxes[] = {0, 0, 0, 0};
unsigned int minRangeOfJoystickAxes[] = {UINT_MAX, UINT_MAX, UINT_MAX, UINT_MAX};
const int JOYSTICK_AXIS_DEAD_ZONE_RATES[] = {10, 10, 10, 10};
int joystickAxisDeadZones[] = {0, 0, 0, 0};
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

void configJoystick(){
  printOnDisplay("Configurando joysticks", true, 1000, true);
  for (int i = 0; i < getArraySize(PT_PINS_XY_JS); i++) {
    pinMode(*PT_PINS_XY_JS[i], INPUT);
  }

  pinMode(PIN_J1Z, INPUT_PULLUP);
  pinMode(PIN_J2Z, INPUT_PULLUP);
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

void setup() {
  startSerial();
  configDisplay();
  configJoystick();
  getRangeOfJoystickAxes();
  getAverageOfJoystickAxes();
  getDeadZonesOfJoystickAxes();
}

void loop() {
  for (int i = 0; i < getArraySize(PT_PINS_XY_JS); i++) {
    Serial.print(analogRead(*PT_PINS_XY_JS[i]));
    Serial.print("\t");
  }

  Serial.print(digitalRead(PIN_J1Z));
  Serial.print("\t");

  Serial.print(digitalRead(PIN_J2Z));
  Serial.print("\t");

  Serial.println();
}