#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

template<typename T, int N>
int getArraySize(T (&array)[N]) {
  return N;
}

//-------------------------------------|DISPLAY|-------------------------------------
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
// The pins for I2C are defined by the Wire-library. 
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 Display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
//-------------------------------------|CONTROL|-------------------------------------
//---|??(J1s), ??(J2s)
const int JOYSTICK_BUTTON_PINS[] = {25, 26};
//---|Yaw(J1x), Roll(J2x), Pitch(J2y)
//DOF = Degrees Of Freedom
const int DOF_ROTATIONS_PINS[] = {35, 33, 32};
//---|Throttle(J1y)
const int THROTTLE_PIN = {34};

void startSerial(){
  Serial.begin(115200);  
}

void configDisplay(){
  Display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS);
  Display.setTextSize(1);
  Display.setTextColor(SSD1306_WHITE);
}

void printOnDisplay(String text){
  Display.clearDisplay();
  Display.setCursor(0,0);
  Display.println(text);
  Display.display();
  delay(500);
}

void configJoystick(){
  printOnDisplay("Configurando joysticks");
  for (int i = 0; i < getArraySize(DOF_ROTATIONS_PINS); i++) {
    pinMode(DOF_ROTATIONS_PINS[i], INPUT);
  }

  for (int i = 0; i < getArraySize(JOYSTICK_BUTTON_PINS); i++) {
    pinMode(JOYSTICK_BUTTON_PINS[i], INPUT_PULLUP);
  }

  pinMode(THROTTLE_PIN, INPUT);
}

void setup() {
  startSerial();
  configDisplay();
  configJoystick();
}

void loop() {
  for (int i = 0; i < getArraySize(DOF_ROTATIONS_PINS); i++) {
    Serial.print(analogRead(DOF_ROTATIONS_PINS[i]));
    Serial.print("\t");
  }

  Serial.print(analogRead(THROTTLE_PIN));
  Serial.print("\t");

  for (int i = 0; i < getArraySize(JOYSTICK_BUTTON_PINS); i++) {
    Serial.print(!digitalRead(JOYSTICK_BUTTON_PINS[i]));
    Serial.print("\t");
  }

  Serial.println();
}