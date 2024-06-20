const int PinosMotores[4] = { 14, 18, 19, 26 };   //Pino x Motor: {TrásEsquerda, TrásDireita, FrenteDireita, FrenteEsquerda} respectivamente
const int Resolucao = 12; //0-4095 == 4096
const int Frequencia = 50; //50 já estava, ir subindo até acha um equilibrio entre perfomance e vibração do drone

int Speed = 0;
void WriteSpeed() {
  for (int i = 0; i < 4; i++) {
    ledcWrite(EnginePins[i], Speed);
  }
}

void IncreaseSpeed() {
  for (; Speed <= 254; ++Speed) {
    WriteSpeed();
  }
}

void DecreaseSpeed() {
  for (; Speed >= 1; --Speed) {
    WriteSpeed();
  }
}
void setup() {
  Serial.begin(115200);
  for (int i = 0; i < 4; i++) {
    pinMode(PinosMotores[i], OUTPUT);
    ledcAttachChannel(PinosMotores[i], Frequencia, Resolucao, i);
    ledcWrite(PinosMotores[i], 300);
  }
  Serial.println("----");
  delay(5000);
    IncreaseSpeed();
    Serial.println("----");
  delay(5000);
    ledcWrite(0, 300);
    Serial.println("----");
  delay(5000);
    ledcWrite(0, 0);
    Serial.println("----");
  delay(5000);
    analogWrite(0, 300);
  delay(5000);
    analogWrite(0, 0);
    Serial.println("----");
      delay(5000);
    analogWrite(PinosMotores[0], 300);
    Serial.println("----");
  delay(5000);
    analogWrite(PinosMotores[0], 0);
}



int Speed = 2000;
int Delay = 2000;
void loop() {
  DecreaseSpeed();
  delay(Delay);
  Serial.println("1");
  for (int i = 0; i < 4; i++) {
    ledcWrite(PinosMotores[i], 300);
  }
  delay(Delay);
  Serial.println("2");
  for (int i = 0; i < 4; i++) {
    ledcWrite(PinosMotores[i], 0);
  }
  delay(Delay);
  Serial.println("3");
  for(int j = 0; j < Speed; j++){
    for (int i = 0; i < 4; i++) {
      ledcWrite(PinosMotores[i], j);
    }
  }
  delay(Delay);
  Serial.println("4");
  for(int j = Speed; j > 1; j--){
    for (int i = 0; i < 4; i++) {
      ledcWrite(PinosMotores[i], j);
    }
  }
  delay(Delay);
    Serial.println("5");
  for (int i = 0; i < 4; i++) {
    ledcWrite(i, Speed);
  }
  delay(Delay);
  Serial.println("6");
  for (int i = 0; i < 4; i++) {
    ledcWrite(i, 300);
  }
  delay(Delay);
  Serial.println("7");
  for(int j = 0; j < Speed; j++){
    for (int i = 0; i < 4; i++) {
      ledcWrite(i, j);
    }
  }
  delay(Delay);
  Serial.println("8");
  for(int j = Speed; j > 1; j--){
    for (int i = 0; i < 4; i++) {
      ledcWrite(i, j);
    }
  }
  delay(Delay);
      Serial.println("9");
  for (int i = 0; i < 4; i++) {
    analogWrite(PinosMotores[i], Speed);
  }
  delay(Delay);
  Serial.println("10");
  for (int i = 0; i < 4; i++) {
    analogWrite(PinosMotores[i], 0);
  }
  delay(Delay);
  Serial.println("11");
  for(int j = 0; j < Speed; j++){
    for (int i = 0; i < 4; i++) {
      analogWrite(PinosMotores[i], j);
    }
  }
  delay(Delay);
  Serial.println("12");
  for(int j = Speed; j > 1; j--){
    for (int i = 0; i < 4; i++) {
      analogWrite(PinosMotores[i], j);
    }
  }
  delay(Delay);
}