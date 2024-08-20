//------------------------------------|LIBRARIES|------------------------------------
#include <WiFi.h>
#include <Wire.h>
#include <esp_now.h>

//----------------------------------|COMUNICATION|-----------------------------------
struct DataReceived {
  unsigned int dof[3];
  unsigned int throttle;
  volatile bool flightMode;
};

DataReceived dataReceived;

struct DataSend {
  enum Errors {
    NOT_ERROR = 0,
    MPU6050_ERROR = 1,
  };

  Errors error;
};

DataSend dataSend = {DataSend::NOT_ERROR};

bool isDisconnected = true;

const uint8_t MASTER_MAC_ADDRESS[] = {0xC4, 0xD8, 0xD5, 0x95, 0x97, 0xF4};

//-------------------------------------|ENGINES|-------------------------------------
const int MOTOR_PINS[4] = {14, 18, 19, 26}; //não usar do 6 ao 11
const int CHANNELS[4] = { 0, 1, 2, 3 }; //Utilizando 4 canais de 16 do PWM do ESP32  

const int RESOLUTION = 9; //0-4095 == 4096
const int FREQUENCY = 50;
const int MIN_SPEED = 205;//pow(2, RESOLUTION)

//-------------------------------------|MPU6050|-------------------------------------
#define MPU6050Address 0x68

template<typename T, int N>
int getArraySize(T (&array)[N]) {
  return N;
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
  // pinMode(MPU6050_ERROR_LED_PIN, OUTPUT);
  // pinMode(CycleExceededErrorRedLEDPin, OUTPUT);
  // pinMode(FlashingBlueLEDPin, OUTPUT);
  // pinMode(LowBatteryYellowLEDPin, OUTPUT);

  for (int i = 0; i < getArraySize(MOTOR_PINS); i++) {
    pinMode(MOTOR_PINS[i], OUTPUT);
    ledcAttachChannel(MOTOR_PINS[i], FREQUENCY, RESOLUTION, CHANNELS[i]);
    ledcWrite(MOTOR_PINS[i], MIN_SPEED);    
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

  /*
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

void setup() {
  startSerial();
  startStationMode();
  startEspNow();
  settings();
  registerFunctionThatExecutesWhenReceivingData();
  addMasterAsPeerOnEspNow();
  setupMPU6050();
}

void whenReceivingDataDo(const esp_now_recv_info_t * MAC_ADDRESS, const uint8_t* PACKAGE, const int PACKAGE_SIZE){
  isDisconnected = false;
  memcpy(&dataReceived, PACKAGE, sizeof(dataReceived));
}

void loop() {
  // Serial.println(dataReceived.throttle);
}