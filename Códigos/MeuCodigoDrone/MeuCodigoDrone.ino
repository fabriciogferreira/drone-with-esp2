#include <esp_now.h>
#include <WiFi.h>
#include <Wire.h>

//MPU 3 = 22 = SCL
//MPU 4 = 21 = SDA

//VARIÁVEIS E CONSTANTES
//Para o PWM
const int Canais[4] = { 0, 1, 2, 3 }; //Utilizando 4 canais de 16 do PWM do ESP32  
const int Resolucao = 12; //0-4095 == 4096
const int Frequencia = 50; //50 já estava, ir subindo até acha um equilibrio entre perfomance e vibração do drone
int AntigaVelocidade = 210;
int NovaVelocidade;
const int PinosMotores[4] = { 14, 18, 19, 26 };   //Pino x Motor: {(2) TrásEsquerda, (3) TrásDireita, (4) FrenteDireita, (1) FrenteEsquerda} respectivamente
int VelocidadeBase[4] = { 0, 0, 0, 0 };  //3,3,4,4 - A velocidade base é necessária e individual para cada motor, pois mesmo que os motores sejam iguais, eles reagem de forma diferente para a mesma quantidade de energia
int QtdPinosMotores = sizeof(PinosMotores) / 4;
//Váriaveis para subir a velocidade do drone pelo Serial
bool isNumber;
char CaracterSerial;
char Escolha;
String TextoSerial = "";
///9- 10000
///8- 20000
//MPU6050
const char EnderecoMPU6050 = 0X68;

void EscreverVelocidadoPeloSerial() {
    if (Serial.available()) {
    TextoSerial = "";
    // delay(1000);
    isNumber = true;
    while (Serial.available()) {
      CaracterSerial = Serial.read();
      if (CaracterSerial != '\n') {
        TextoSerial.concat(CaracterSerial);
        if (!isDigit(CaracterSerial)) {
          isNumber = false;
        }
      }
    }

    // Serial.println("=====================================================================");
    // Serial.print("Texto digitado: ");
    // Serial.println(TextoSerial);
    // Serial.println("=====================================================================");
    if (isNumber) {
      NovaVelocidade = TextoSerial.toInt();
      // Serial.print("Antiga velocidade: ");
      // Serial.println(AntigaVelocidade);
      // Serial.print("Nova velocidade: ");
      // Serial.println(NovaVelocidade);
      // Serial.println("=====================================================================");
      // MudarVelocidade();
      AntigaVelocidade = NovaVelocidade;
      // MostrarVelocidadesDasHelices();
    } else {
      switch (TextoSerial[0]) {
        case 'a':
          Serial.print("A: ");
          VelocidadeBase[0] = NovaVelocidade;
          break;
        case 'b':
          Serial.print("B: ");
          VelocidadeBase[1] = NovaVelocidade;
          break;
        case 'c':
          Serial.print("B: ");
          VelocidadeBase[2] = NovaVelocidade;
          break;
        case 'd':
          Serial.print("B: ");
          VelocidadeBase[3] = NovaVelocidade;
          break;
      }
      Serial.println(NovaVelocidade);
      // EscreverVelocidade();
      for(int i = 0; i < QtdPinosMotores; i++){
        Serial.print("Velocidade base do Motor ");
        Serial.print(PinosMotores[i]);
        Serial.print(": ");
        Serial.println(VelocidadeBase[i]);
      }
    }
  }
}

void EscreverVelocidade() {
  // Serial.println(AntigaVelocidade);
  // Serial.println(AntigaVelocidade);
    // ledcWrite(0, AntigaVelocidade);
  // for (int i = 0; i < QtdPinosMotores; i++) {
  // }
}

void AumentarVelocidade() {
  for (int i = 0; i < QtdPinosMotores; i++) {
    Serial.println(ledcRead(i));
    Serial.println(ledcRead(PinosMotores[i]));
    ledcWrite(PinosMotores[i], VelocidadeBase[i] + AntigaVelocidade);
  }
  // for (; AntigaVelocidade <= NovaVelocidade; ++AntigaVelocidade) {
  //   EscreverVelocidade();
  // }
}

void DiminuirVelocidade() {
  for (; AntigaVelocidade >= NovaVelocidade; --AntigaVelocidade) {
    EscreverVelocidade();
  }
}

void MudarVelocidade(){
  if (NovaVelocidade < AntigaVelocidade) {
    Serial.println("Diminuindo a velocidade das hélices...");
    DiminuirVelocidade();
  } else if (NovaVelocidade == AntigaVelocidade) {
    Serial.println("A velocidade digitada é a mesma da atual, digite outra velocidade!!!");
  } else {
    Serial.println("Aumentando a velocidade das hélices... ");
    AumentarVelocidade();
  }
}

void MostrarVelocidadesDasHelices(){
  for (int i = 0; i < QtdPinosMotores; i++) {
    Serial.print("Velocidade do motor ");
    Serial.print(PinosMotores[i]);
    Serial.print(" é ");
    Serial.print(VelocidadeBase[i] + AntigaVelocidade);
    Serial.print(" = ");
    Serial.print(VelocidadeBase[i]);
    Serial.print(" + ");
    Serial.println(AntigaVelocidade);
  }
}


void setup() {
  Serial.begin(115200);

  for (int i = 0; i < sizeof(PinosMotores) / 4; i++) {
    pinMode(PinosMotores[i], OUTPUT);
    ledcAttachChannel(PinosMotores[i], Frequencia, Resolucao, i);
    ledcWrite(PinosMotores[i], 210);
  }
}

void loop() {
  if (Serial.available()) {
    TextoSerial = "";
    while (Serial.available()) {
      CaracterSerial = Serial.read();
      if (CaracterSerial != '\n') {
        TextoSerial.concat(CaracterSerial);
      }
    }
    AntigaVelocidade = TextoSerial.toInt();
    Serial.println(AntigaVelocidade);
  }
  for (int j = 0; j < AntigaVelocidade; j++) {
    for (int i = 0; i < sizeof(PinosMotores) / 4; i++) {
      ledcWrite(PinosMotores[i], j);
    }
    delayMicroseconds(1000);
  }
}