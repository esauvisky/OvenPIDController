// Controlador PID para o forninho
// Autor: Emiliano Sauvisky <esauvisky@gmail.com>

/**************************
        Bibliotecas
**************************/
#include <max6675.h>
#include <LiquidCrystal.h>
#include <Wire.h>
#include <PID_v1.h>

/**************************
  Definição de constantes
**************************/
// Pinos do Módulo MAX6675
#define THERMO_GND 6
#define THERMO_VCC 5
#define THERMO_DO  4
#define THERMO_CS  3
#define THERMO_CLK 2

// Pinos do LCD
#define LCD_RS 8
#define LCD_EN 9
#define LCD_D4 10
#define LCD_D5 11
#define LCD_D6 12
#define LCD_D7 13

// Símbolo ºC bonitinho para o LCD
uint8_t degree[8] = {140, 146, 146, 140, 128, 128, 128, 128};

// Definições PID
double temperature, lastTemperature, setPoint, output;
unsigned long lastTempUpdate, windowStartTime;
#define WINDOW_SIZE        5000
#define MINIMUM_RELAY_TIME 500
#define RELAY_PIN          A0
#define TEMP_READ_DELAY    250
#define KP                 45
#define KI                 0.05
#define KD                 20

/***************************
  Inicialização de Objetos
***************************/
// Cria o objeto do MAX6675
MAX6675 thermocouple(THERMO_CLK, THERMO_CS, THERMO_DO);

// Cria o objeto do LCD
LiquidCrystal lcd(LCD_RS, LCD_EN, LCD_D4, LCD_D5, LCD_D6, LCD_D7);

// Cria o objeto do PID
//PID myPID(&temperature, &output, &setPoint, KP, KI, KD, P_ON_M, DIRECT);
PID myPID(&temperature, &output, &setPoint, KP, KI, KD, DIRECT);


// Atualiza a temperatura no menor tempo possível de acordo com o chip
bool updateTemperature() {
  if ((millis() - lastTempUpdate) > TEMP_READ_DELAY) {
    // Obtém a média do termopar em celsius
    //temperature = (lastTemperature + thermocouple.readCelsius()) / 2;
    //lastTemperature = temperature;
    temperature = thermocouple.readCelsius();
    lastTempUpdate = millis();
    return true;
  }
  return false;
}

/***************************
     Pre-Configurações
***************************/
void setup() {
  Serial.begin(9600);

  // Define setPoint hardcoded
  setPoint = 100;

  // Define o pino do relay como um output
  pinMode(RELAY_PIN, OUTPUT);

  // Ativa os pinos GND e VCC para o módulo MAX6675
  pinMode(THERMO_VCC, OUTPUT); digitalWrite(THERMO_VCC, HIGH);
  pinMode(THERMO_GND, OUTPUT); digitalWrite(THERMO_GND, LOW);

  // Inicia o módulo do LCD (2 linhas, 16 colunas por linha)
  lcd.begin(16, 2);
  // Adiciona o símbolo de graus ao LCD
  lcd.createChar(0, degree);
  lcd.clear();

  // Inicializa o PID
  // Seta o output do myPID entre 0 e WINDOW_SIZE.
  myPID.SetOutputLimits(0, WINDOW_SIZE);
  myPID.SetSampleTime(125);
  myPID.SetMode(AUTOMATIC);

  // Aguarda a inicialização do MAX6675
  delay(250);

  // Obtém a primeira medida de temperatura
  //lastTemperature = thermocouple.readCelsius();
}


/***************************
       Loop Principal
***************************/
void loop() {
  // Atualiza a temperatura
  updateTemperature();
  // Roda o cálculo do PID
  myPID.Compute();
  
  // Imprime a primeira linha do LCD (a temperatura atual)
  lcd.setCursor(0, 0);
  lcd.print("CurTemp:"); lcd.print(temperature); 
  lcd.setCursor(14, 0); 
  lcd.write((byte)0); lcd.print("C");

  // Imprime a segunda linha (o output)
  lcd.setCursor(0, 1);
  lcd.print("Output: "); lcd.print(int(output));
  
  // Liga o relê baseado no output do pid
  unsigned long now = millis();
  if (now - windowStartTime > WINDOW_SIZE) {
    lcd.clear();
    windowStartTime += WINDOW_SIZE;
  }
  if (output > now - windowStartTime) {
    // Do not waste relay clicks if the time is too little
    //if (output > MINIMUM_RELAY_TIME) {
      lcd.setCursor(13, 1); lcd.print(" ON");
      digitalWrite(RELAY_PIN, HIGH);
    //}
  } else {
    // Do not waste relay clicks if the time is too little
    //if (WINDOW_SIZE - output < MINIMUM_RELAY_TIME) {
      lcd.setCursor(13, 1); lcd.print("OFF");
      digitalWrite(RELAY_PIN, LOW);
    //}
  }
  
  delay(50);
}
