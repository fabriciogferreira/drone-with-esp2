//------------------------------------|LIBRARIES|------------------------------------
#include <esp_now.h>
#include <Wire.h>
#include <WiFi.h>

//-----------------------------------|INFORMATION|-----------------------------------
/*
-|SENTIDO DO DRONE (Drone no formato X)
--|Para localizar os motores, basta tomar como direção a 'frente' onde está o enforca gato

-|CONCEITOS
--|Throttle: Acelerar os motores

-|GRAUS DE LIBERDADE
--|REFERÊNCIA
---| (https://pt.wikipedia.org/wiki/Graus_de_liberdade_(mec%C3%A2nica)#:~:text=Na%20f%C3%ADsica%2C%20os%20graus%20de,definem%20sua%20configura%C3%A7%C3%A3o%20ou%20estado.)
--|DESLOCAMENTO
---|X - Frente/Trâs | Avançar/Recuar
---|Y - Direita/Esquerda | Deslocamento Esquerda/Direita
---|Z - Cima/Baixo | Subir/Descer
----|Subir:aumentar a velocidade dos motores de forma igual
----|Descer: diminuir a velocidade dos motores de forma igual
--|ROTAÇÕES
---|Pitch (X) - Rotação/Giro ou Deslocamento no eixo X
----|Avançar: é preciso diminuir a velocidade de dois motores da frente e aumentar a velocidade dos dois opostos
----|Recuar: é exatamente o oposto, aumenta os dois da frente e diminui os outros dois
---|Roll (Y) - Rotação/Giro ou Deslocamento no eixo Y
----|Direita: é preciso diminuir a velocidade dos dois motores da direita e aumentar a velocidade dos dois opostos
----|Esquerda: é preciso diminuir a velocidade dos dois motores da esquerda e aumentar a velocidade dos dois opostos
---|Yaw (Z) - Rotação/Giro no eixo Z
----|Sentido Horário: É preciso aumentar a velocidade dos motores dos braços brancos e diminuir os outros dois, ou seja, aumentar a velocidade dos motores que giram no sentido anti-horário
----|Sentido Anti-Horário: Aumentar a velocidade dos motores dos braços vermelhos e diminiuir os outros dois, ou seja, aumentar a velocidade dos motores que giram no sentido horário

-|VISUALIZAÇÃO
--|Usar o modo de visualização no serial apenas quando não estiver em voo, utilizar em modo voo, isso pode fazer com que o ciclo exceda o tempo  

-|CICLO
--|Não deixar o valor do ciclo do drone passar de 1800µs
--|Os PIDs também deverão ser limitados a um valor de aproximadamente 300μs.
--|0,0018s | Segundos
--|1,8ms   | Millissegundos
--|1800µs  | Microssegundos 

-|ESSE ESP32
--|Possui 16 canais
--|1-20 RESOLUÇÃO: Qtd = 2^Resolução

-|MATERIAIS
--|1 Bateria URUAV 11.1V Lipo
--|4 Motores Brushless (Corrente Continua, Sem Escovas ) 2300 kV
--|4 Placas ESC – 30 A

-|PID
--|Kp = ao P do PID = Kp * Erroº = 10*10º = 100μs => 1,5ms + 0,1ms = 1,6ms
--|Ki = ao I do PID = Erroº + Somatório(ErroAnterior) => 10º + 1 = 11
--|Kd = ao D do PID = ErroAnterior - ErroAtual => 10º - 5º = 5
--|A saída do PID será em microssegundos, será a quantidade de tempo que o motor ficará em estado high
--|Há um PID para eixo de rotação do drone, ou seja, um pro Pitch, Roll e Yaw
*/

//-----------------------------------|PREFERENCES|-----------------------------------
bool FlightMode = 1;   // 0: Modo acrobatico, 1: Modo estable (por defecto FlightMode = 1)

//---------------------------------------|PID|---------------------------------------
int LimitIntegralPartPIDAngularVelocity = 380;  // Limitar parte integral PID velocidad
int LimitPIDAngularVelocityOutput = 380;  // Limitar salida del PID velocidad
int LimitIntegralPartPIDAngle = 130;  // Limitar parte integral PID ángulo
int LimitPIDAngleOutput = 130;  // Limitar salida del PID ángulo

//Gyro Pitch (X), Roll (Y), Yaw (Z)
float PIDAngleError[3] = {0, 0, 0};
float ProportionalPartPIDAngle[3] = {0, 0, 0};
float IntegralPartPIDAngle[3] = {0, 0, 0};
float DerivativePartPIDAngle[3] = {0, 0, 0};
float PIDAngleOutput[3] = {0, 0, 0};
float PIDAngularVelocityError[3] = {0, 0, 0};
float ProportionalPartPIDAngularVelocity[3] = {0, 0, 0};
float IntegralPartPIDAngularVelocity[3] = {0, 0, 0};
float DerivativePartPIDAngularVelocity[3] = {0, 0, 0};
float PIDAngularVelocityOutput[3] = {0, 0, 0};

//--------------------------------------|DEBUG|--------------------------------------
bool ShowInformation = false;
float Start = 0;

//---------------------------------|PID ADJUSTMENTS|---------------------------------
float PidOutputAdjustment[2][3] = {
  {0.5, 0.05, 10},//Pitch{P, I, D}
  {0.5, 0.05, 10},//Roll{P, I, D}
};

float PidAngularVelocityAdjustment[3][3] = {
  {2, 0.02, 0},//Pitch{P, I, D}
  {2, 0.02, 0},//Roll{P, I, D}
  {1, 0.05, 0},//Yaw{P, I, D}
};

//-------------------------------------|ENGINES|-------------------------------------
const int EnginePins[4] = {14, 18, 19, 26};
//não usar do 6 ao 11
const int BaseSpeed[4] = {0, 0, 0, 0};
float ESCUs[4] = {0, 0, 0, 0};
const int Channels[4] = { 0, 1, 2, 3 }; //Utilizando 4 canais de 16 do PWM do ESP32  
const int Resolution = 12; //0-4095 == 4096
//(Está em Hz) 50 já estava, ir subindo até acha um equilibrio entre perfomance e vibração do drone, até 32.000Hz, manter a frequência a baixo de Freq * 2 < 80000 MHZ
const int Frequency = 50;
const int MaxSpeed = 350;//pow(2, Resolution)
const int MinSpeed = 205;//pow(2, Resolution)

//---------------------------------------|LED|---------------------------------------
#define MPU6050RedErrorLEDPin 25
#define CycleExceededErrorRedLEDPin 14
#define FlashingBlueLEDPin 12
const int LowBatteryYellowLEDPin = 13;
const int BatteryPin = 6;
int LedCounter;

//-------------------------------------|CONTROL|-------------------------------------
float ControlThrottleSetPoint = 1000;
float ControlThrottleSetPointFilter = 0;
//Gyro Pitch (X), Roll (Y), Yaw (Z)
float ControlSetPoint[3] = {0, 0, 0};
//Yaw, Throttle, Roll, Pitch;
int ControlData[4] = {0, 0, 0, 0};
uint8_t ControlMacAddress[6] = {0, 0, 0, 0, 0, 0};

//-------------------------------------|MPU6050|-------------------------------------
float MPU6050ExecutionTime, MPU6050Time;
#define MPU6050Address 0x68
bool SetGyroscopeAngles, AccelerometerCalibrateOK = false;
float PitchAngleAcceleration, RollAngleAcceleration, Temperature;
float AccelerationTotalVector;
//accx, accy, accz, gyrox, gyroy, gyroz,
float MPU6050Calibration[6] = {0, 0, 0, 0, 0, 0};
float MPU6050Data[6] = {0, 0, 0, 0, 0, 0};
//Pitch (X), Roll (Y), Yaw (Z)
float CurrentAngle[3] = {0, 0, 0};
float PreviousAngle[3] = {0, 0, 0};
int GyroscopePreviousPoint[3] = {0, 0, 0};
int GyroscopeCurrentPoint[3] = {0, 0, 0};
float Error = 0;

//--------------------------------------|TIMES|--------------------------------------
long LoopTimer, EngineStartingMoment, RunningTime, StartTime, EndTime;
#define CycleTimeMicroseconds 6000   // Ciclo de ejecucion del software en microsegundos

//-------------------------------------|BATTERY|-------------------------------------
bool LowBatteryWarning = false;
float BatteryVoltage;
int LowBatteryWarningCounter;

//------------------------------------|FUNCTIONS|------------------------------------
void StartSerial() {
  if (!ShowInformation) {return;}
  Serial.begin(115200);
  // analogWrite(LowBatteryYellowLEDPin, 255);
}

void StartsStationMode() {
  WiFi.mode(WIFI_STA);

  if (!ShowInformation) {return;}

  Serial.println("=====================================================================");
  Serial.println("Inicializando o modo station");
  Serial.print("Mac Address desse ESP em Station: ");
  Serial.println(WiFi.macAddress());
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

void Settings(){
  // pinMode(MPU6050RedErrorLEDPin, OUTPUT);
  // pinMode(CycleExceededErrorRedLEDPin, OUTPUT);
  // pinMode(FlashingBlueLEDPin, OUTPUT);
  // pinMode(LowBatteryYellowLEDPin, OUTPUT);

  for (int i = 0; i < 4; i++) {
    pinMode(EnginePins[i], OUTPUT);
    ledcAttachChannel(EnginePins[i], Frequency, Resolution, i);
    ledcWrite(EnginePins[i], MinSpeed);    
  }

  if (!ShowInformation) {return;}

  Serial.println("=====================================================================");
  Serial.println("Configurando...");
}

void SetupMPU6050() {
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
    if (ShowInformation) {
      Serial.println("=====================================================================");
      while (1) {
        // digitalWrite(MPU6050RedErrorLEDPin, LOW);
        // delay(500);
        // digitalWrite(MPU6050RedErrorLEDPin, HIGH);
        // delay(500);
      }
    }
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

void ShowFlightMode(){
  if (!ShowInformation) {return;}
  Serial.println("=====================================================================");
  FlightMode ? Serial.println("Modo Estável") : Serial.println("Modo Acrobático");
}

void WaitUntilMinimumSpeed (){
  // digitalWrite(CycleExceededErrorRedLEDPin, HIGH);
  // while (ControlThrottleSetPoint < 950 || ControlThrottleSetPoint > 1050) ProcessControlData();
  // digitalWrite(CycleExceededErrorRedLEDPin, LOW);
}

template<typename AnyArray, size_t NumberPositions>
void ShowArrayOneDimensionalValues(AnyArray (&Array)[NumberPositions], String Title = ""){
  if(Title != ""){
    Serial.print(Title);
    Serial.print(": ");
  }

  for (size_t Position = 0; Position < NumberPositions; ++Position) {
    if (Position != NumberPositions - 1) {
      Serial.print(Array[Position]);
      Serial.print("\t");
    }else{
      Serial.println(Array[Position]);
    }
  }
}

void ProcessControlData() {
  int decrease = 2;
  for (int increase = 0; increase < 4; increase++) {
    if (increase == 1){
      ControlThrottleSetPoint = float(ControlData[increase]);
    } else {
      ControlSetPoint[decrease] = float(ControlData[increase]);
      decrease--;
    }
  }
}

// Función para hacer parpadear el LED al entrar en el loop() principal
void BlinkLED() {
  // if (LedCounter % 20 == 0) {
  //   digitalRead(FlashingBlueLEDPin) == LOW ? digitalWrite(FlashingBlueLEDPin, HIGH) : digitalWrite(FlashingBlueLEDPin, LOW);
  //   digitalRead(FlashingBlueLEDPin) == HIGH && LowBatteryWarning == 1 ? analogWrite(LowBatteryYellowLEDPin, 0) : analogWrite(LowBatteryYellowLEDPin, 255);
  // }
  // LedCounter++;
}

void ReadBatteryVoltage() {
  // if (ControlThrottleSetPoint < 1100) {
  //   // Leer entrada analógica
  //   BatteryVoltage = 2.95 * (analogRead(BatteryPin) * 4.92  / 1023);

  //   // Si la tension de batería es inferior a 10.5V durante un número de ciclos consecutivos
  //   // se activa la señal de batería baja
  //   if (BatteryVoltage < 10.5 && LowBatteryWarning == false) {
  //     LowBatteryWarningCounter++;
  //     if (LowBatteryWarningCounter > 30)LowBatteryWarning = true;
  //   } else {
  //     LowBatteryWarningCounter = 0;
  //   }
  // }
}

void WriteSpeed(int Value) {
  for (int i = 0; i < 4; i++) {
    ledcWrite(EnginePins[i], Value);
  }
}

void IncreaseSpeed() {
  for (int i = 0; i < MaxSpeed; i++) {
    WriteSpeed(i);
  }
}

void ResetEngineSpeed(int Address){
  ledcWrite(Address, 0);
}

void DecreaseSpeed() {
  for (int i = MaxSpeed; i >= 1; --i) {
    WriteSpeed(i);
  }
}

void PWM() {
  // Para generar las 4 señales PWM, el primer paso es poner estas señales a 1 (HIGH).
  IncreaseSpeed();

  EngineStartingMoment = micros();

  // ------------------ ¡¡1ms max!! ------------------
  StartTime = micros();
  ProcessControlData();// Leer mando RC
  BlinkLED();          // LED parpadeo
  ReadBatteryVoltage();// Leer Vbat

  // Si la duracion entre StartTime y EndTime ha sido mayor de 900us, encender LED de aviso.
  // Nunca hay que sobrepasar 1ms de tiempo en estado HIGH.
  EndTime = micros();
  RunningTime = EndTime - StartTime;
  // if (RunningTime > 900) digitalWrite(CycleExceededErrorRedLEDPin, HIGH);
  // ------------------ ¡¡1ms max!! ------------------

  // Pasamos las señales PWM a estado LOW cuando haya transcurrido el tiempo definido en las variables ESCx_us
  while (ledcRead(0) == MaxSpeed || ledcRead(1) == MaxSpeed || ledcRead(2) == MaxSpeed || ledcRead(3) == MaxSpeed) {
    for (int i = 0; i < 4; i++) {
      if (EngineStartingMoment + ESCUs[i] <= micros()) {
        // ledcWrite(i, 0);
        ResetEngineSpeed(i);
      }
    }
  }
}

void ReadDataMPU6050() {
  // Los datos del giroscopio y el acelerómetro se encuentran de la dirección 3B a la 14
  Wire.beginTransmission(MPU6050Address);       // Empezamos comunicación
  Wire.write(0x3B);                             // Pedir el registro 0x3B (AcX)
  Wire.endTransmission();
  Wire.requestFrom(MPU6050Address, 14);         // Solicitar un total de 14 registros
  while (Wire.available() < 14);                // Esperamos hasta recibir los 14 bytes

  for (int i = 0; i < 7; i++) {
    i == 3 ? Temperature = Wire.read() << 8 | Wire.read() : MPU6050Data[i] = Wire.read() << 8 | Wire.read();
  }
  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  // 0x41 (TEMP_OUT_H)   & 0x42 (TEMP_OUT_L)
  // 0x43 (GYRO_XOUT_H)  & 0x44 (GYRO_XOUT_L)
  // 0x45 (GYRO_YOUT_H)  & 0x46 (GYRO_YOUT_L)
  // 0x47 (GYRO_ZOUT_H)  & 0x48 (GYRO_ZOUT_L)

  // Restar valores de calibracion del acelerómetro
  if (AccelerometerCalibrateOK) {
    MPU6050Data[0] = MPU6050Data[0] - MPU6050Calibration[0];
    MPU6050Data[1] = MPU6050Data[1] - MPU6050Calibration[1];
    MPU6050Data[2] = MPU6050Data[2] - MPU6050Calibration[2];
    MPU6050Data[2] = MPU6050Data[2] + 4096;
  }
}

// Cálculo de velocidad angular (º/s) y ángulo (º)
void ProcessDataMPU6050() {
  // Restar valores de calibración del acelerómetro y calcular
  // velocidad angular en º/s. Leer 65.5 en bruto equivale a 1º/s
  GyroscopeCurrentPoint[0] = (MPU6050Data[3] - MPU6050Calibration[3]) / 65.5;
  GyroscopeCurrentPoint[1] = (MPU6050Data[4] - MPU6050Calibration[4]) / 65.5;
  GyroscopeCurrentPoint[2] = (MPU6050Data[5] - MPU6050Calibration[5]) / 65.5;

  // Calculamos exactamente cuánto tiempo ha pasado desde que se ha ejecutado el cálculo del ángulo.
  // Al tener señales PWM variables entre 1 y 2ms, este cálculo del ángulo no se ejecuta siempre
  // con un periodo constante.
  MPU6050ExecutionTime = (micros() - MPU6050Time) / 1000;

  // Calcular ángulo de inclinación con datos de giroscopio:
  // velocidad (º/s) * tiempo (s) = grados de inclinación (º)
  CurrentAngle[0] = CurrentAngle[0] + GyroscopeCurrentPoint[0] * MPU6050ExecutionTime / 1000;
  CurrentAngle[1] = CurrentAngle[1] + GyroscopeCurrentPoint[1] * MPU6050ExecutionTime / 1000;
  // 0.000000266 = t // 0.000000266 iempo_ejecucion / 1000 / 65.5 * PI / 180
  CurrentAngle[0] = CurrentAngle[0] + CurrentAngle[1] * sin((MPU6050Data[5] - MPU6050Calibration[5]) * MPU6050ExecutionTime * 0.000000266);
  CurrentAngle[1] = CurrentAngle[1] - CurrentAngle[0] * sin((MPU6050Data[5] - MPU6050Calibration[5]) * MPU6050ExecutionTime * 0.000000266);
  MPU6050Time = micros();

  // Calcular vector de aceleración
  // 57.2958 = Conversion de radianes a grados 180/PI
  AccelerationTotalVector = sqrt(pow(MPU6050Data[1], 2) + pow(MPU6050Data[0], 2) + pow(MPU6050Data[2], 2));
  PitchAngleAcceleration = asin(MPU6050Data[1] / AccelerationTotalVector) * 57.2958;
  RollAngleAcceleration  = asin(MPU6050Data[0] / AccelerationTotalVector) * -57.2958;

  if (SetGyroscopeAngles) {
    // Filtro complementario
    CurrentAngle[0] = CurrentAngle[0] * 0.995 + PitchAngleAcceleration * 0.005;   // Angulo Pitch de inclinacion
    CurrentAngle[1] = CurrentAngle[1] * 0.995 + RollAngleAcceleration  * 0.005;   // Angulo Roll de inclinacion
  } else {
    CurrentAngle[0] = PitchAngleAcceleration;
    CurrentAngle[1] = RollAngleAcceleration;
    SetGyroscopeAngles = true;
  }
}

void CalculateAngleWithPID() {
  for(int i = 0; i < 2; i++){
    PIDAngleError[i] = ControlSetPoint[i] - CurrentAngle[i];                                                              // Error entre lectura y consigna
    
    ProportionalPartPIDAngle[i] = PidOutputAdjustment[i][0] * PIDAngleError[i];                                           // Parte proporcional
    
    IntegralPartPIDAngle[i] = IntegralPartPIDAngle[i] + (PidOutputAdjustment[i][1] * PIDAngleError[i]);                   // Parte integral (sumatorio del error en el tiempo)
    IntegralPartPIDAngle[i] = constrain(IntegralPartPIDAngle[i], -LimitIntegralPartPIDAngle, LimitIntegralPartPIDAngle);  // Limitar parte integral
    
    DerivativePartPIDAngle[i] = PidOutputAdjustment[i][2] * (CurrentAngle[i] - PreviousAngle[i]);                         // Parte derivativa (diferencia entre el error actual y el anterior)

    PIDAngleOutput[i] = ProportionalPartPIDAngle[i] + IntegralPartPIDAngle[i] + DerivativePartPIDAngle[i];                // Salida PID  
    PIDAngleOutput[i] = constrain(PIDAngleOutput[i], -LimitPIDAngleOutput, LimitPIDAngleOutput);                          // Limitar salida del PID
  }
}

void CalculateAngularVelocityWithPID() {
  // En funcion del modo de vuelo que hayamos seleccionado, las consignas de los PID serán diferentes
  for(int i = 0; i < 3; i++){
    if (i == 2){
      Error = ControlSetPoint[i];
    }else{
      if (FlightMode == 0) {
        // En modo acrobático solo controlamos la velocidad de cada eje (un PID por eje). La consigna del PID se da en º/s
        // y viene directamente del mando RC
        Error = ControlSetPoint[i];
      }else {
        // En modo estable las consignas de los PID de velocidad vienen de las salidas de los PID de ángulo
        Error = PIDAngleOutput[i];
      }
    }

    PIDAngularVelocityError[i] = Error - GyroscopeCurrentPoint[i];                                                                    //Error entre lectura y consigna
    ProportionalPartPIDAngularVelocity[i] = PidAngularVelocityAdjustment[i][0]  * PIDAngularVelocityError[i];                                                 // Parte proporcional
    IntegralPartPIDAngularVelocity[i] = IntegralPartPIDAngularVelocity[i] + (PidAngularVelocityAdjustment[i][1] * PIDAngularVelocityError[i]);                // Parte integral (sumatorio del error en el tiempo)
    IntegralPartPIDAngularVelocity[i] = constrain(IntegralPartPIDAngularVelocity[i], -LimitIntegralPartPIDAngularVelocity, LimitIntegralPartPIDAngularVelocity);// Limitar parte integral
    DerivativePartPIDAngularVelocity[i] = PidAngularVelocityAdjustment[i][2] * (GyroscopeCurrentPoint[i] - GyroscopePreviousPoint[i]);                        // Parte derivativa (diferencia entre el error actual y el anterior)

    PIDAngularVelocityOutput[i] = ProportionalPartPIDAngularVelocity[i] + IntegralPartPIDAngularVelocity[i] + DerivativePartPIDAngularVelocity[i];              // Salida PID
    PIDAngularVelocityOutput[i] = constrain(PIDAngularVelocityOutput[i], -LimitPIDAngularVelocityOutput, LimitPIDAngularVelocityOutput);                        // Limitar salida del PID
  }
}

void Stability() {
  // Si el Throttle es menos a 1300us, el control de estabilidad se desactiva. La parte integral
  // de los controladores PID se fuerza a 0.
  if (ControlThrottleSetPoint <= 1300) {
    IntegralPartPIDAngularVelocity[0] = 0;
    IntegralPartPIDAngularVelocity[1] = 0;
    IntegralPartPIDAngularVelocity[2] = 0;
    IntegralPartPIDAngle[0] = 0;
    IntegralPartPIDAngle[1] = 0;

    for (int i = 0; i < 4; i++) {
      ESCUs[i] = ControlThrottleSetPoint;
      if (ESCUs[i] < 1000) ESCUs[i] = 950; // Si lo motores giran con el stick de Throttle al mínimo, recudir el valor de 950us
    }
  } else { // Si el throttle es mayor a 1300us, el control de estabilidad se activa.
    // Limitar throttle a 1800 para dejar margen a los PID
    if (ControlThrottleSetPoint > 1800) ControlThrottleSetPoint = 1800;

    // Stability
    ESCUs[0] = ControlThrottleSetPoint + PIDAngularVelocityOutput[0] - PIDAngularVelocityOutput[1] - PIDAngularVelocityOutput[2]; // Motor 1
    ESCUs[1] = ControlThrottleSetPoint + PIDAngularVelocityOutput[0] + PIDAngularVelocityOutput[1] + PIDAngularVelocityOutput[2]; // Motor 2
    ESCUs[2] = ControlThrottleSetPoint - PIDAngularVelocityOutput[0] + PIDAngularVelocityOutput[1] - PIDAngularVelocityOutput[2]; // Motor 3
    ESCUs[3] = ControlThrottleSetPoint - PIDAngularVelocityOutput[0] - PIDAngularVelocityOutput[1] + PIDAngularVelocityOutput[2]; // Motor 4
    // for (int i = 0; i < 4; i++) {
    //   ESCUs[i] = ControlThrottleSetPointFilter; // Solo para testeos
    // }

    for (int i = 0; i < 4; i++) {
      if (ESCUs[i] < 1100) ESCUs[i] = 1100; // Evitamos que alguno de los motores de detenga completamente en pleno vuelo
      if (ESCUs[i] > 2000) ESCUs[i] = 2000; // Evitamos mandar consignas mayores a 2000us a los motores
    }
  }
}

void Views() {
  if (!ShowInformation) {return;}
  Serial.println("===============================|COMUNICAÇÃO|===============================");

  Serial.print("REMETENTE: ");
  for (int i = 0; i < 6; i++) {
    Serial.printf("%02X", ControlMacAddress[i]);
    if(i != 5){
      Serial.print(":");
    }else{
      Serial.println();
    }
  }

  Serial.print("MPU6050: ");
  Serial.println(MPU6050ExecutionTime);
  
  Serial.print("MP6050 (ACCE): ");
  Serial.print(MPU6050Data[0] / 4096);
  Serial.print("\t");
  Serial.print(MPU6050Data[1] / 4096);
  Serial.print("\t");
  Serial.println(MPU6050Data[2] / 4096);
  
  Serial.print("MP6050 (GYRO): ");
  Serial.print((MPU6050Data[3] - MPU6050Calibration[3]) / 16.4);
  Serial.print("\t");
  Serial.print((MPU6050Data[4] - MPU6050Calibration[4]) / 16.4);
  Serial.print("\t");
  Serial.println((MPU6050Data[5] - MPU6050Calibration[5]) / 16.4);

  ShowArrayOneDimensionalValues(CurrentAngle, "CurrentAngle");

  Serial.print("ControlThrottle: ");
  Serial.println(ControlThrottleSetPoint);

  ShowArrayOneDimensionalValues(ControlSetPoint, "ControlSetPoint");

  ShowArrayOneDimensionalValues(ESCUs, "ESCUs");

  PidAngularVelocityAdjustment[0][1] = 0;
  PidAngularVelocityAdjustment[1][1] = 0;
  PidAngularVelocityAdjustment[2][1] = 0;
  PidOutputAdjustment[0][1] = 0;
  PidOutputAdjustment[1][1] = 0;

  Serial.print("Temperature: ");
  Serial.println(Temperature);

  Serial.print("Tempo Execução (s): ");
  Serial.println((micros() - Start) / 1000000, 6);
  delay(2000);
}

void WhenReceivingDataExecute(const esp_now_recv_info_t * MacAddr, const uint8_t* IncomingData, int QtdValoresJoysticks){
  if(ShowInformation) {memcpy(&ControlMacAddress, MacAddr, sizeof(ControlMacAddress));}
  memcpy(&ControlData, IncomingData, sizeof(ControlData));
}

void CalibrateMPU6050Data() {
  if (ShowInformation) {
    Serial.println("=====================================================================");
    Serial.println("Calibrar MPU6050");
  }

  int Max = 3000;
  for (int i = 0; i < Max; i++) {
    ReadDataMPU6050();
    for (int i = 0; i < 6; i++) {
      MPU6050Calibration[i] = MPU6050Calibration[i] + MPU6050Data[i];
    }
    delayMicroseconds(20);
  }
  
  for (int i = 0; i < 6; i++) {
    MPU6050Calibration[i] = MPU6050Calibration[i] / Max;
  }

  AccelerometerCalibrateOK = true;
}

void RegisterMainFuncion(){
  if (ShowInformation) {
    Serial.println("=====================================================================");
    Serial.println("Registrando a função principal");
  }
  esp_now_register_recv_cb(WhenReceivingDataExecute);
  LoopTimer = micros();
}

void ReleaseExecution() {
  if (ShowInformation) {
    Serial.println("=====================================================================");
    Serial.println("Mova a velocidade até ficar em 0");
  }
  while (ControlThrottleSetPoint != 950) {
    ProcessControlData();
    if (ShowInformation) {
      Serial.println(ControlThrottleSetPoint);
    }
  }
}

void setup() {
  StartSerial();
  StartsStationMode();
  StartEspNow();
  Settings();
  SetupMPU6050();
  CalibrateMPU6050Data();
  ReadBatteryVoltage();
  RegisterMainFuncion();
  ReleaseExecution();
}

void loop() {
  if (ShowInformation) {
    Start = micros();
  }
  // if (micros() - LoopTimer > CycleTimeMicroseconds + 50) digitalWrite(CycleExceededErrorRedLEDPin, HIGH);

  while (micros() - LoopTimer < CycleTimeMicroseconds);
  LoopTimer = micros();

  PWM();                                         // Generar señales PWM para los motores
  ReadDataMPU6050();                             // Leer sensor MPU6050
  ProcessDataMPU6050();                          // Procesar datos del sensor MPU6050
  if (FlightMode == 1) {CalculateAngleWithPID();}// Obtener salida de los PID de inclinación
  CalculateAngularVelocityWithPID();             // Obtener salida de los PID de velocidad
  Stability();                                   // Stability o generador de señales para PWM

  // // Guardamos las lecturas del sensor MPU6050 para el siguiente ciclo (necesario para los PID)
  for (int i = 0; i < 3; i++){
    PreviousAngle[i] = CurrentAngle[i];
    GyroscopePreviousPoint[i] = GyroscopeCurrentPoint[i];
  }

  Views();
}