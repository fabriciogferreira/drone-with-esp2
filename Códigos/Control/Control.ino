//------------------------------------|LIBRARIES|------------------------------------
#include <WiFi.h>
#include <esp_now.h>

//--------------------------------------|DEBUG|--------------------------------------
bool ShowInformation = true;

//--------------------------------------|TIMER|--------------------------------------
long TemporizadorLoop, TempoExecucao;

//---------------------------------------|ESP|---------------------------------------
uint8_t MacAndressOfSlaves[][6] = {
  { 0xA8, 0x42, 0xE3, 0x45, 0x81, 0xF8 }  //Mac address do ESP32 do drone
};
const int Canal = 1;  //Canal do slave
int QtdSlaves = sizeof(MacAndressOfSlaves) / 6; //Quantidade de ESP Escravos, os que vão receber dados

//-------------------------------------|CONTROL|-------------------------------------
//---|Yaw(J1x), Throttle(J1y), Roll(J2x), Pitch(J2y);
const int JoystickPins[4] = {35, 34, 33, 32};
int SetPoints[4] = {0, 0, 0, 0};
int MinSetPointsAdjustedUs[4] = {-30, 950, -30, -30};
int MidSetPointsAdjustedUs[4] = {0, 1500, 0, 0};
int MaxSetPointsAdjustedUs[4] = {30, 2000, 30, 30};
int MinRangeJoystick[4] = {2048, 2048, 2048, 2048};
long MidRangeJoystick[4] = {0, 0, 0, 0};
int MaxRangeJoystick[4] = {2048, 2048, 2048, 2048};
long DeadZonePercentage[4] = {10, 10, 10, 10};
long Start = 0;
#define CycleTimeMicroseconds 10000 //10000000 para teste

void StartSerial() {
  if (!ShowInformation) {return;}
  Serial.begin(115200);
}

void Settings(){
  if (ShowInformation) {
    Serial.println("=====================================================================");
    Serial.println("Configurando...");
  }

  for (int i = 0; i < 4; i++) {
    pinMode(JoystickPins[i], INPUT);
  }
}

void CalculateRangeJoystickAxes(){
  if (ShowInformation) {
    Serial.println("=====================================================================");
    Serial.println("Mova os dois joysticks para todas as direções");
    delay(2000);
  }

  Start = micros();
  int Read = 0;
  while (micros() - Start < 10000000) {
    for (int i = 0 ; i < 4; i++) {
      Read = analogRead(JoystickPins[i]);
      if(Read > MaxRangeJoystick[i]){
        MaxRangeJoystick[i] = Read;
      } else if (Read < MinRangeJoystick[i]) {
        MinRangeJoystick[i] = Read;
      }
    }
  }
}

void CalculateJoystickMiddle(){
  if (ShowInformation) {
    Serial.println("Tire as mãos do Joystick imediatamente");
    delay(5000);
    Serial.println("=====================================================================");
    Serial.println("Calculando o meio do joystick");
  }

  int max = 3000;
  for (int i = 0; i < max; i++) {
    for (int j = 0; j < 4; j++) {
      SetPoints[j] = analogRead(JoystickPins[j]);
      SetPoints[j] = constrain(SetPoints[j], MinRangeJoystick[j], MaxRangeJoystick[j]);
      MidRangeJoystick[j] = MidRangeJoystick[j] + SetPoints[j];
    }
    delayMicroseconds(20);
  }

  for (int i = 0; i < 4; i++) {
    MidRangeJoystick[i] = MidRangeJoystick[i]/max;
  }
}

void CalculateJoystickMiddleDeadZone(){
  if(ShowInformation){
    Serial.println("=====================================================================");
    Serial.println("Calculando a zona morta");
  }

  long Diferrence = 0;
  for (int i = 0; i < 4; i++) {
    Diferrence = MaxSetPointsAdjustedUs[i] - MinSetPointsAdjustedUs[i];
    DeadZonePercentage[i] = Diferrence * DeadZonePercentage[i] / 100; 
  }
}

void StartsStationMode() {
  WiFi.mode(WIFI_STA);

  if (ShowInformation) {
    Serial.println("=====================================================================");
    Serial.println("Inicializando o modo station");
    Serial.print("Mac Address desse ESP em Station: ");
    Serial.println(WiFi.macAddress());
  }
}

void StartEspNow() {
  bool Result = esp_now_init() == ESP_OK;

  if (ShowInformation && Result) {
    Serial.println("=====================================================================");
    Serial.println("ESPNow iniciado com sucesso.");
  } else if (ShowInformation) {
    Serial.println("=====================================================================");
    Serial.println("Não foi possível iniciar o ESPNow.");
  }

  if (!Result) {ESP.restart();}
}

void AoReceberRespostaExecute(const uint8_t *mac_addr,  esp_now_send_status_t response) {
  if(ShowInformation){
      Serial.print("RESPOSTA: ");
    (response == ESP_NOW_SEND_SUCCESS) ? Serial.println("Recebido") : Serial.println("Não recebido");
  }
  EnviarDados();
}

void RegisterMainFuncion(){
  if (ShowInformation) {
    Serial.println("=====================================================================");
    Serial.println("Registrando a função que executa ao receber resposta");
  }
  esp_now_register_send_cb(AoReceberRespostaExecute);
}

void StartCommunicationESPs(){
  if(ShowInformation){
    Serial.println("=====================================================================");
    Serial.println("Configurando Master (Mestre) e Escravos (Slaves)...");
  }

  for (int i = 0; i < QtdSlaves; i++) {
    esp_now_peer_info_t Slave = {};
    Slave.channel = Canal;
    Slave.encrypt = 0;
    memcpy(Slave.peer_addr, MacAndressOfSlaves[i], sizeof(MacAndressOfSlaves[i]));
    esp_now_add_peer(&Slave);
  }
}

template<typename Array>
void ShowArrayOneDimensionalValues(Array array[], String Title = ""){
  int max = sizeof(array);

  if(Title != ""){
    Serial.print(Title);
    Serial.print(": ");
  }

  for (int i = 0; i < max; i++) {
    if (i != max - 1) {
      Serial.print(array[i]);
      Serial.print("\t");
    }else{
      Serial.println(array[i]);
    }
  }
}

void Temporizador() {
  while (micros() - TemporizadorLoop < CycleTimeMicroseconds);
  TempoExecucao = (micros() - TemporizadorLoop) / 1000;
  TemporizadorLoop = micros();
}

void ShowConfigurationInformation(){
  if(ShowInformation){
    ShowArrayOneDimensionalValues(SetPoints, "SetPoints");
    ShowArrayOneDimensionalValues(MinSetPointsAdjustedUs, "MinSetPointsAdjustedUs");
    ShowArrayOneDimensionalValues(MidSetPointsAdjustedUs, "MidSetPointsAdjustedUs");
    ShowArrayOneDimensionalValues(MaxSetPointsAdjustedUs, "MaxSetPointsAdjustedUs");
    ShowArrayOneDimensionalValues(MinRangeJoystick, "MinRangeJoystick");
    ShowArrayOneDimensionalValues(MidRangeJoystick, "MidRangeJoystick");
    ShowArrayOneDimensionalValues(MaxRangeJoystick, "MaxRangeJoystick");
    ShowArrayOneDimensionalValues(DeadZonePercentage, "DeadZonePercentage");
  }
}

void CalcularSetPoints() {
  for (int i = 0; i < 4; i++) {
    SetPoints[i] = analogRead(JoystickPins[i]);
    SetPoints[i] = constrain(SetPoints[i], MinRangeJoystick[i], MaxRangeJoystick[i]);
    if (SetPoints[i] <= MidRangeJoystick[i]) {
      SetPoints[i] = map(SetPoints[i], MinRangeJoystick[i], MidRangeJoystick[i], MinSetPointsAdjustedUs[i], MidSetPointsAdjustedUs[i]);
    }else{
      SetPoints[i] = map(SetPoints[i], MidRangeJoystick[i], MaxRangeJoystick[i], MidSetPointsAdjustedUs[i], MaxSetPointsAdjustedUs[i]);
    }

    if (MidSetPointsAdjustedUs[i] - DeadZonePercentage[i] <= SetPoints[i] && SetPoints[i] <= MidSetPointsAdjustedUs[i] + DeadZonePercentage[i]) {
      SetPoints[i] = MidSetPointsAdjustedUs[i];
    }
  }

  if (ShowInformation) {
    ShowArrayOneDimensionalValues(SetPoints, "Enviar dados");
  }
}

void EnviarDados() {
  Temporizador();
  CalcularSetPoints();

  esp_err_t response = esp_now_send(MacAndressOfSlaves[0], (uint8_t *) &SetPoints, sizeof(SetPoints));

  if(ShowInformation){
    Serial.println("==================================|COMUNICAÇÃO|==================================");
    Serial.print("REQUISIÇÃO: ");
    (response == ESP_OK) ? Serial.print("Feita para ") : Serial.print("Não feita para ");

    for (int i = 0; i < 6; i++) {
      Serial.print(MacAndressOfSlaves[0][i], HEX);
      if(i != 5){
        Serial.print(":");
      }else{
        Serial.println();
      }
    }
  }
}

void setup() {
  StartSerial();
  Settings();
  CalculateRangeJoystickAxes();
  CalculateJoystickMiddle();
  CalculateJoystickMiddleDeadZone();
  StartsStationMode();
  StartEspNow();
  RegisterMainFuncion();
  StartCommunicationESPs();
  ShowConfigurationInformation();
  EnviarDados();
}

void loop() {}