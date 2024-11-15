//------------------------------------|LIBRARIES|------------------------------------
#include <WiFi.h>
#include <Wire.h>
#include <esp_now.h>
#include <MPU6050_tockn.h>

//--------------------------------------|TIMES|--------------------------------------
const int SOFTWARE_CYCLE_TIME_IN_US = 6000;
int cycleStartTime = 0;

//----------------------------------|COMUNICATION|-----------------------------------
struct DataReceived {
  unsigned int dof[3];
  unsigned int throttle;
  bool flightMode;
};

DataReceived dataReceived;



bool isDisconnected = true;

const uint8_t MASTER_MAC_ADDRESS[] = {0xC4, 0xD8, 0xD5, 0x95, 0x97, 0xF4};
//-------------------------------------|ENGINES|-------------------------------------
int rcYawAngle = 0;
int rcThrotle = 0;
int rcRollAngle = 0;
int rcPitchAngle = 0;
int *PT_RC_ANGLES[] = {&rcYawAngle, &rcRollAngle, &rcPitchAngle}; 
int *PT_RC_ROLL_AND_PITCH_ANGLES[] = {&rcRollAngle, &rcPitchAngle};

float engineStartTime = 0;
bool anyEngineOn;
//-------------------------------------|ENGINES|-------------------------------------
// Motor 1, 2, 3, 4
const int MOTOR_PINS[4] = {26, 14, 18, 19}; //não usar do 6 ao 11
const int CHANNELS[4] = { 0, 1, 2, 3 }; //Utilizando 4 canais de 16 do PWM do ESP32  

const int RESOLUTION = 8; //0-4095 == 4096
const int FREQUENCY = 50000;

const unsigned int MIN_SPEED = 0;
const unsigned int MAX_SPEED = 255;
const unsigned int SPEED_PIN = 34;
unsigned int speed = MIN_SPEED;

float pwmSignalInUs[] = {0, 0, 0, 0};

struct DataSend {
  enum Errors {
    NOT_ERROR = 0,
    MPU6050_ERROR = 1,
  };

  Errors error;
  unsigned int speed;
  float angles[3];
  float pwmSignalInUs[4];
  float angularVelocities[3];
};

DataSend dataSend = {DataSend::NOT_ERROR, speed, {0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0}};
//-------------------------------------|MPU6050|-------------------------------------
#define MPU6050Address 0x68

MPU6050 mpu6050(Wire);

float MPU6050AccX = 0;
float MPU6050AccY = 0;
float MPU6050AccZ = 0;
float Temperature = 0;
float MPU6050GyrX = 0;
float MPU6050GyrY = 0;
float MPU6050GyrZ = 0;
float *PT_MPU6050_ACC_DATA[3] = {&MPU6050AccX, &MPU6050AccY, &MPU6050AccZ};
float *PT_MPU6050_GYR_DATA[3] = {&MPU6050GyrX, &MPU6050GyrY, &MPU6050GyrZ};
float *PT_MPU6050_DATA[] = {&MPU6050AccX, &MPU6050AccY, &MPU6050AccZ, &Temperature, &MPU6050GyrX, &MPU6050GyrY, &MPU6050GyrZ};
float *PT_MPU6050_ACC_AND_GYR_DATA[] = {&MPU6050AccX, &MPU6050AccY, &MPU6050AccZ, &MPU6050GyrX, &MPU6050GyrY, &MPU6050GyrZ};


float MPU6050AccCalX = 0;
float MPU6050AccCalY = 0;
float MPU6050AccCalZ = 0;
float MPU6050GyrCalX = 0;
float MPU6050GyrCalY = 0;
float MPU6050GyrCalZ = 0;
float *PT_MPU6050_ACC_CAL_DATA[3] = {&MPU6050AccCalX, &MPU6050AccCalY, &MPU6050AccCalZ};
float *PT_MPU6050_GYR_CAL_DATA[3] = {&MPU6050GyrCalX, &MPU6050GyrCalY, &MPU6050GyrCalZ};
float *PT_MPU6050_CAL_DATA[] = {&MPU6050AccCalX, &MPU6050AccCalY, &MPU6050AccCalZ, &MPU6050GyrCalX, &MPU6050GyrCalY, &MPU6050GyrCalZ};


float angularVelocityGyrX = 0;
float angularVelocityGyrY = 0;
float angularVelocityGyrZ = 0;
float *PT_GYRO_VELOCITY_ANGULAR[] = {&angularVelocityGyrX, &angularVelocityGyrY, &angularVelocityGyrZ};
float PT_GYRO_VELOCITY_ANGULAR_ANT[] = {0, 0, 0};
float vectorAcc = 0;
float pitchAngleAcceleration = 0;
float rollAngleAcceleration = 0;


float yawAngle = 0;
float rollAngle = 0;
float pitchAngle = 0;
float *PT_ANGLES[] = {&yawAngle, &rollAngle, &pitchAngle};
float *PT_ROLL_AND_PITCH_ANGLES[] = {&rollAngle, &pitchAngle};


float lastYawAngle = 0;
float lastRollAngle = 0;
float lastPitchAngle = 0;
float *lastAngles[] = {&lastYawAngle, &lastRollAngle, &lastPitchAngle};
float *PT_LAST_ROLL_AND_PITCH_ANGLE[] = {&lastRollAngle, &lastPitchAngle};


bool setGyroAngles = false;

bool isCalibrated = false;
//-----------------------------------|PREFERENCES|-----------------------------------
bool flightMode = true;

//---------------------------------------|PID|---------------------------------------
float rollAndPitchPIDAngleAdjustment[2][3] = {
  {0.5, 0.71, 10}, //ROLL
  {0.5, 0.71, 10}, //Pitch
};

float pidAngularVelocityAdjustments[3][3] ={
  {1, 0.92, 0},//yaw
  {2, 0.69, 0},//roll
  {2, 0.69, 0},//pitch
};

float pidVelocityAngularKi[] = {0, 0, 0};//yaw, roll, pitch
float rollAndPitchPidAngleKi[2] = {0, 0}; //roll, pitch

float yawPidAngleOutput = 0;
float rollPidAngleOutput = 0;
float pitchPidAngleOutput = 0;
float *PT_PID_ANGLE_OUTPUT[] = {&yawPidAngleOutput, &rollPidAngleOutput, &pitchPidAngleOutput};
float *PT_ROLL_AND_PITCH_PID_ANGLE_OUTPUT[] = {&rollPidAngleOutput, &pitchPidAngleOutput};

float PT_PID_VELOCITY_ANGULAR_OUTPUT[] = {0,0,0};

int pidAngleIntegralError = 130;
int pidVelocityAngularIntegralError = 380;

template<typename T, int N>
int getArraySize(T (&array)[N]) {
  return N;
}

void pidVelocityAngular(){
  float angularVelocityPidAngle = 0;
  float error = 0, kp = 0, kd;

  for (int i = 0; i < getArraySize(PT_PID_ANGLE_OUTPUT); i++) {
    if (i == 0) {
      angularVelocityPidAngle = rcYawAngle;
    }else{
      angularVelocityPidAngle = flightMode ? *PT_ROLL_AND_PITCH_PID_ANGLE_OUTPUT[i - 1] : *PT_RC_ROLL_AND_PITCH_ANGLES[i - 1];
    }

    error = angularVelocityPidAngle - *PT_GYRO_VELOCITY_ANGULAR[i];

    kp = pidAngularVelocityAdjustments[i][0] * error;


    pidVelocityAngularKi[i] = pidVelocityAngularKi[i] + error;
    pidVelocityAngularKi[i] = pidAngularVelocityAdjustments[i][1] * pidVelocityAngularKi[i];
    pidVelocityAngularKi[i] = constrain(pidVelocityAngularKi[i], -pidVelocityAngularIntegralError, pidVelocityAngularIntegralError);
    
    kd = pidAngularVelocityAdjustments[i][2] * (*PT_GYRO_VELOCITY_ANGULAR[i] - PT_GYRO_VELOCITY_ANGULAR_ANT[i]);

    PT_PID_VELOCITY_ANGULAR_OUTPUT[i] = kp + pidVelocityAngularKi[i] + kd;
    PT_PID_VELOCITY_ANGULAR_OUTPUT[i] = constrain(PT_PID_VELOCITY_ANGULAR_OUTPUT[i], -pidVelocityAngularIntegralError, pidVelocityAngularIntegralError);
  }
}

void pidAngle(){
  float error = 0;
  float kp = 0;
  float kd = 0;

  for (int i = 0; i < getArraySize(PT_RC_ROLL_AND_PITCH_ANGLES); i++) {
    error = *PT_RC_ROLL_AND_PITCH_ANGLES[i] - *PT_ROLL_AND_PITCH_ANGLES[i];

    kp = rollAndPitchPIDAngleAdjustment[i][0] * error;
    
    rollAndPitchPidAngleKi[i] = rollAndPitchPidAngleKi[i] + error;
    rollAndPitchPidAngleKi[i] = rollAndPitchPidAngleKi[i] * rollAndPitchPIDAngleAdjustment[i][1];
    rollAndPitchPidAngleKi[i] = constrain(rollAndPitchPidAngleKi[i], -pidAngleIntegralError, pidAngleIntegralError);
    
    kd = rollAndPitchPIDAngleAdjustment[i][2] * (*PT_ROLL_AND_PITCH_ANGLES[i] - *PT_LAST_ROLL_AND_PITCH_ANGLE[i]);

    *PT_ROLL_AND_PITCH_PID_ANGLE_OUTPUT[i] = kp + rollAndPitchPidAngleKi[i] + kd;

    *PT_ROLL_AND_PITCH_PID_ANGLE_OUTPUT[i] = constrain(*PT_ROLL_AND_PITCH_PID_ANGLE_OUTPUT[i], -pidAngleIntegralError, pidAngleIntegralError);
  }
}

void startSerial() {
  Serial.begin(115200);
}

void startStationMode() {
  WiFi.begin();
  WiFi.mode(WIFI_STA);
}

void startEspNow() {
  if (esp_now_init() != ESP_OK) {
    ESP.restart();
  }
}

void settings(){
  for (int i = 0; i < getArraySize(MOTOR_PINS); i++) {
    pinMode(MOTOR_PINS[i], OUTPUT);
    ledcAttachChannel(MOTOR_PINS[i], FREQUENCY, RESOLUTION, CHANNELS[i]);
    ledcWrite(MOTOR_PINS[i], 0);    
  }
}

void registerFunctionThatExecutesWhenReceivingData(){
  esp_now_register_recv_cb(whenReceivingDataDo);
  while (isDisconnected) {delayMicroseconds(1);};
}

void addMasterAsPeerOnEspNow(){
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, MASTER_MAC_ADDRESS, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    ESP.restart();
  }
}

void sendData(){
  esp_now_send(MASTER_MAC_ADDRESS, (uint8_t *) &dataSend, sizeof(dataSend));
}

void setupMPU6050() {
  Wire.begin();

  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);
  
  /*
  Wire.beginTransmission(MPU6050Address);
  Wire.write(0x6B);                                                          //Registro 6B hex)
  Wire.write(0x00);                                                          //00000000 para activar giroscopio
  Wire.endTransmission();

  Wire.beginTransmission(MPU6050Address);
  Wire.write(0x1B);                                                          //Register 1B hex
  Wire.write(0x08);                                                          //Girscopio a 500dps (full scale)
  Wire.endTransmission();
  
  Wire.beginTransmission(MPU6050Address);
  Wire.write(0x1C);                                                          //Register (1A hex)
  Wire.write(0x10);                                                          //Acelerometro a  +/- 8g (full scale range)
  Wire.endTransmission();

  Wire.beginTransmission(MPU6050Address);
  Wire.write(0x1B);
  Wire.endTransmission();
  Wire.requestFrom(MPU6050Address, 1);
  while (Wire.available() < 1);
  
  if (Wire.read() != 0x08) {
    dataSend.error = DataSend::MPU6050_ERROR;
    sendData();
    while (true) {delayMicroseconds(1);};
  }
  
  // Activar y configurar filtro pasa bajos LPF que incorpora el sensor
  Wire.beginTransmission(MPU6050Address);
  Wire.write(0x1A);
  Wire.write(0x04);
  Wire.endTransmission();

    Frecuencia de corte del filtro pasa bajos:
    256Hz(0ms):0x00
    188Hz(2ms):0x01
    98Hz(3ms):0x02
    42Hz(4.9ms):0x03
    20Hz(8.5ms):0x04
    10Hz(13.8ms):0x05
    5Hz(19ms):0x06
  */
}
//CORRETO
void readMPU6050() {
  Wire.beginTransmission(MPU6050Address);
  Wire.write(0x3B);
  Wire.endTransmission();
  Wire.requestFrom(MPU6050Address, 14);
  while (Wire.available() < 14);

  for (int i = 0; i < getArraySize(PT_MPU6050_DATA); i++) {
    *PT_MPU6050_DATA[i] = Wire.read() << 8 | Wire.read();
  }

  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  // 0x41 (TEMP_OUT_H)   & 0x42 (TEMP_OUT_L)
  // 0x43 (GYRO_XOUT_H)  & 0x44 (GYRO_XOUT_L)
  // 0x45 (GYRO_YOUT_H)  & 0x46 (GYRO_YOUT_L)
  // 0x47 (GYRO_ZOUT_H)  & 0x48 (GYRO_ZOUT_L)

  // Restar valores de calibracion del acelerómetro
  if (isCalibrated) {
    for (int i = 0; i< getArraySize(PT_MPU6050_ACC_DATA); i++) {
      *PT_MPU6050_ACC_DATA[i] = *PT_MPU6050_ACC_DATA[i] - *PT_MPU6050_ACC_CAL_DATA[i];
    }

    MPU6050AccZ = MPU6050AccZ + 4096;
  }
}

void averageAccAndGyr(){
  int n = 3000;
  for (int i = 0; i < n; i++) {
    readMPU6050();

    for (int j = 0; j < getArraySize(PT_MPU6050_ACC_AND_GYR_DATA); j++) {
      *PT_MPU6050_CAL_DATA[j] = *PT_MPU6050_CAL_DATA[j] + *PT_MPU6050_ACC_AND_GYR_DATA[j];
    }

    delayMicroseconds(20);
  }

  for (int i = 0; i < getArraySize(PT_MPU6050_CAL_DATA); i++) {
    *PT_MPU6050_CAL_DATA[i] = *PT_MPU6050_CAL_DATA[i] / n;
  }

  isCalibrated = true;
}

float mpu6050RuntimeInMs = 0;
float angleCalculationTimeInUs = 0;
void processMPU6050Data(){
  // for (int i = 0; i < getArraySize(PT_GYRO_VELOCITY_ANGULAR); i++) {
  //   *PT_GYRO_VELOCITY_ANGULAR[i] = (*PT_MPU6050_GYR_DATA[i] - *PT_MPU6050_GYR_CAL_DATA[i]) / 65.5;
  // }

  *PT_GYRO_VELOCITY_ANGULAR[0] = mpu6050.getGyroX();
  *PT_GYRO_VELOCITY_ANGULAR[1] = mpu6050.getGyroZ();
  *PT_GYRO_VELOCITY_ANGULAR[2] = mpu6050.getGyroY();

  mpu6050RuntimeInMs = (micros() - angleCalculationTimeInUs) / 1000;

  pitchAngle = pitchAngle + angularVelocityGyrX * mpu6050RuntimeInMs / 1000;
  rollAngle = rollAngle + angularVelocityGyrY * mpu6050RuntimeInMs / 1000;

  pitchAngle = pitchAngle + rollAngle * sin((MPU6050GyrZ - MPU6050GyrCalZ) * mpu6050RuntimeInMs * 0.000000266);
  rollAngle = rollAngle - pitchAngle * sin((MPU6050GyrZ - MPU6050GyrCalZ) * mpu6050RuntimeInMs * 0.000000266);

  angleCalculationTimeInUs = micros();

  vectorAcc = sqrt(pow(MPU6050AccX, 2) + pow(MPU6050AccY, 2) + pow(MPU6050AccZ, 2));
  pitchAngleAcceleration = asin(MPU6050AccY / vectorAcc) * 57.2958;
  rollAngleAcceleration = asin(MPU6050AccX / vectorAcc) * -57.2958;

  mpu6050.update();

  if (setGyroAngles) {
    pitchAngle = mpu6050.getAngleX() * 0.995 + pitchAngleAcceleration * 0.005;
    rollAngle = mpu6050.getAngleY() * 0.995 + rollAngleAcceleration * 0.005;
  }else{
    pitchAngle = pitchAngleAcceleration;
    rollAngle = rollAngleAcceleration;

    setGyroAngles = true;
  }
}

void modulator(){
  if (rcThrotle <= 1300) {
    for(int i = 0; i < getArraySize(pidVelocityAngularKi); i++) pidVelocityAngularKi[i] = 0;

    for(int i = 0; i < getArraySize(rollAndPitchPidAngleKi); i++) rollAndPitchPidAngleKi[i] = 0;
    
    for(int i = 0; i < getArraySize(pwmSignalInUs); i++){
      pwmSignalInUs[i] = rcThrotle;
      if(pwmSignalInUs[i] < 1000) pwmSignalInUs[i] = 950;
    }
  } else {
    if(rcThrotle > 1800) rcThrotle = 1800;
    
    pwmSignalInUs[0] = rcThrotle + PT_PID_VELOCITY_ANGULAR_OUTPUT[2] - PT_PID_VELOCITY_ANGULAR_OUTPUT[1] - PT_PID_VELOCITY_ANGULAR_OUTPUT[0];
    pwmSignalInUs[1] = rcThrotle + PT_PID_VELOCITY_ANGULAR_OUTPUT[2] + PT_PID_VELOCITY_ANGULAR_OUTPUT[1] + PT_PID_VELOCITY_ANGULAR_OUTPUT[0];
    pwmSignalInUs[2] = rcThrotle - PT_PID_VELOCITY_ANGULAR_OUTPUT[2] + PT_PID_VELOCITY_ANGULAR_OUTPUT[1] - PT_PID_VELOCITY_ANGULAR_OUTPUT[0];
    pwmSignalInUs[3] = rcThrotle - PT_PID_VELOCITY_ANGULAR_OUTPUT[2] - PT_PID_VELOCITY_ANGULAR_OUTPUT[1] + PT_PID_VELOCITY_ANGULAR_OUTPUT[0];

    for(int i = 0; i < getArraySize(pwmSignalInUs); i++){
      if(pwmSignalInUs[i] < 1100) pwmSignalInUs[i] = 1100;
      if(pwmSignalInUs[i] > 2000) pwmSignalInUs[i] = 2000;
    }
  }
}

void manageSoftwareCycle(){
  if (micros() - cycleStartTime > SOFTWARE_CYCLE_TIME_IN_US + 50) {
    Serial.println("Passou");
  }

  while (micros() - cycleStartTime < SOFTWARE_CYCLE_TIME_IN_US) {
    delayMicroseconds(1);
  }

  cycleStartTime = micros();
} 

void WhenReceivingResponseDo(const uint8_t *mac_addr,  esp_now_send_status_t response) {

  dataSend.speed = speed;
  for (int i = 0; i < getArraySize(PT_ANGLES); i++) {
    dataSend.angles[i] = *PT_ANGLES[i];
  }


  for (int i = 0; i < 4; i++) {
    dataSend.pwmSignalInUs[i] = pwmSignalInUs[i];
  }

  for (int i = 0; i < 3; i++) {
    dataSend.angularVelocities[i] = PT_PID_VELOCITY_ANGULAR_OUTPUT[i];
  }

  sendData();
}

void registerFunctionThatExecutesWhenReceivingResponse(){
  esp_now_register_send_cb(WhenReceivingResponseDo);
}

void setup() {
  startSerial();
  startStationMode();
  startEspNow();
  settings();
  registerFunctionThatExecutesWhenReceivingResponse();
  registerFunctionThatExecutesWhenReceivingData();
  addMasterAsPeerOnEspNow();
  setupMPU6050();
  averageAccAndGyr();
  sendData();

  cycleStartTime = micros();
}

void processRCData(){
  rcThrotle = dataReceived.throttle > 1800 ? 1800 : dataReceived.throttle;

  flightMode = dataReceived.flightMode;
  
  for (int i = 0; i < getArraySize(dataReceived.dof); i++) {
    *PT_RC_ANGLES[i] = dataReceived.dof[i];
  }
}

void whenReceivingDataDo(const esp_now_recv_info_t * MAC_ADDRESS, const uint8_t* PACKAGE, const int PACKAGE_SIZE){
  isDisconnected = false;
  memcpy(&dataReceived, PACKAGE, sizeof(dataReceived));
  processRCData();
}

void prepareForNewCycle(){
  for (int i = 0; i < getArraySize(PT_GYRO_VELOCITY_ANGULAR_ANT); i++) {
    *lastAngles[i] = *PT_ANGLES[i];
    PT_GYRO_VELOCITY_ANGULAR_ANT[i] = *PT_GYRO_VELOCITY_ANGULAR[i];
  }
}

void emitPWMSignal(){
  for (int i = 0; i < getArraySize(MOTOR_PINS); i++) {
    digitalWrite(MOTOR_PINS[i], HIGH);
  }

  engineStartTime = micros();

  do {
    anyEngineOn = false;

    for (int i = 0; i < getArraySize(MOTOR_PINS); i++) {
      if (engineStartTime + pwmSignalInUs[i] <= micros()) digitalWrite(MOTOR_PINS[i], LOW);
      
      if (digitalRead(MOTOR_PINS[i]) == HIGH) anyEngineOn = true;
    }
  } while (anyEngineOn);
}

void readSpeed(){
  speed = map(analogRead(SPEED_PIN), 0, 4095, MIN_SPEED, MAX_SPEED);
}

void loop() {
  manageSoftwareCycle();
  emitPWMSignal();
  readMPU6050();
  processMPU6050Data();
  if (flightMode) pidAngle();
  pidVelocityAngular();
  modulator();
  prepareForNewCycle();
  readSpeed();
}