// Controlador PID para o forninho
// Autor: Emiliano Sauvisky <esauvisky@gmail.com>

// TODO: Adicionar controles para mudar setPoint em tempo real
// TODO: Testar ativar P_ON_M quando o gap entre input e setPoint for pequeno

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
double temperature, setPoint, output;
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


/* updateTemperature()
    Atualiza a temperatura no menor tempo possível de acordo com o IC
    Retorna true  se a temperatura foi atualizada
    Retorna false se ainda não passou o tempo para realizar nova leitura */
bool updateTemperature() {
    if ((millis() - lastTempUpdate) > TEMP_READ_DELAY) {
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
    //Serial.begin(9600);

    // Define a temperatura desejada
    setPoint = 100;

    // Define o pino do relay como um output
    pinMode(RELAY_PIN, OUTPUT);

    // Ativa os pinos GND e VCC para o módulo MAX6675
    pinMode(THERMO_VCC, OUTPUT); digitalWrite(THERMO_VCC, HIGH);
    pinMode(THERMO_GND, OUTPUT); digitalWrite(THERMO_GND, LOW);

    // Inicia o módulo do LCD (2 linhas, 16 colunas por linha)
    lcd.begin(16, 2);
    // Cria o símbolo de graus ao LCD
    lcd.createChar(0, degree);
    lcd.clear();

    // Seta o output do myPID entre 0 e WINDOW_SIZE.
    myPID.SetOutputLimits(0, WINDOW_SIZE);
    // Seta o sampling time para 125ms
    myPID.SetSampleTime(125);
    // Inicializa o PID
    myPID.SetMode(AUTOMATIC);

    // Aguarda a inicialização do MAX6675
    while (!updateTemperature) {};
}


/***************************
       Loop Principal
***************************/
void loop() {
    // Faz controle de tempo proporcional para determinar se o relê deve ser ligado ou não
    unsigned long now = millis();
    if (now - windowStartTime > WINDOW_SIZE) {
        // Limpa o LCD a cada ciclo
        lcd.clear();
        windowStartTime += WINDOW_SIZE;
    }

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
    if (output > now - windowStartTime) {
        // TODO: Não gastar clicks do relê se a janela for muito pequena
        //if (output > MINIMUM_RELAY_TIME) {
        lcd.setCursor(13, 1); lcd.print(" ON");
        digitalWrite(RELAY_PIN, HIGH);
        //}
    } else {
        //if (WINDOW_SIZE - output < MINIMUM_RELAY_TIME) {
        lcd.setCursor(13, 1); lcd.print("OFF");
        digitalWrite(RELAY_PIN, LOW);
        //}
    }

    delay(50);
}
