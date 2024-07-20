template<typename T, int N>
int getArraySize(T (&array)[N]) {
  return N;
}

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

void configJoystick(){
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