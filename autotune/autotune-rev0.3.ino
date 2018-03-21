#include <PID_v1.h>
#include <PID_AutoTune_v0.h>
#include <max6675.h>
#include <LiquidCrystal.h>
#include <EEPROM.h>

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


#define WINDOW_SIZE     2550
#define RELAY_PIN       A0
#define TEMP_READ_DELAY 250
byte ATuneModeRemember=2;
double input=30, output=50, setpoint=80;
double kp=2,ki=0.5,kd=2;

double kpmodel=1.5, taup=100, theta[50];
double outputStart=5;
//double aTuneStep=50, aTuneNoise=1, aTuneStartValue=100;
double aTuneStep=127, aTuneNoise=1, aTuneStartValue=127;
unsigned int aTuneLookBack=30;

boolean tuning = true;
unsigned long  modelTime, serialTime;

PID myPID(&input, &output, &setpoint,kp,ki,kd, DIRECT);
PID_ATune aTune(&input, &output);
MAX6675 thermocouple(THERMO_CLK, THERMO_CS, THERMO_DO);
LiquidCrystal lcd(LCD_RS, LCD_EN, LCD_D4, LCD_D5, LCD_D6, LCD_D7);


unsigned long lastTempUpdate, windowStartTime;
double temperature, lastTemperature;

//set to false to connect to the real world
boolean useSimulation = false;

struct Parameters {
  double kp;
  double ki;
  double kd;
};

void AutoTuneHelper(boolean start)
{
  if(start)
    ATuneModeRemember = myPID.GetMode();
  else
    myPID.SetMode(ATuneModeRemember);
}

void changeAutoTune()
{
 if(!tuning)
  {
    //Set the output to the desired starting frequency.
    output=aTuneStartValue;
    aTune.SetNoiseBand(aTuneNoise);
    aTune.SetOutputStep(aTuneStep);
    aTune.SetLookbackSec((int)aTuneLookBack);
    AutoTuneHelper(true);
    tuning = true;
  }
  else
  { //cancel autotune
    aTune.Cancel();
    tuning = false;
    AutoTuneHelper(false);
  }
}


void SerialSend()
{
  Serial.print("setpoint: ");Serial.print(setpoint); Serial.print(" ");
  Serial.print("input: ");Serial.print(input); Serial.print(" ");
  Serial.print("output: ");Serial.print(output); Serial.print(" ");
  if(tuning){
    lcd.clear();
    lcd.setCursor(0, 0);
    Serial.println("tuning mode");
    lcd.print("i:"); lcd.print(input);
    lcd.setCursor(0, 1);
    lcd.print("o:"); lcd.print(output);
    lcd.print(" s:"); lcd.print(int(setpoint));
  } else {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(myPID.GetKp());
    lcd.print("|"); lcd.print(myPID.GetKi());
    lcd.setCursor(0, 1);
    lcd.print(myPID.GetKd());
    Serial.print("kp: ");Serial.print(myPID.GetKp());Serial.print(" ");
    Serial.print("ki: ");Serial.print(myPID.GetKi());Serial.print(" ");
    Serial.print("kd: ");Serial.print(myPID.GetKd());Serial.println();
  }
  
}

void SerialReceive()
{
  if(Serial.available())
  {
   char b = Serial.read(); 
   Serial.flush(); 
   if((b=='1' && !tuning) || (b!='1' && tuning))changeAutoTune();
  }
}

void DoModel()
{
  //cycle the dead time
  for(byte i=0;i<49;i++)
  {
    theta[i] = theta[i+1];
  }
  //compute the input
  input = (kpmodel / taup) *(theta[0]-outputStart) + input*(1-1/taup) + ((float)random(-10,10))/100;

}

// Atualiza a temperatura no menor tempo possível de acordo com o chip
bool updateTemperature() {
  if ((millis() - lastTempUpdate) > TEMP_READ_DELAY) {
    // Obtém a média termopar em celsius
    temperature = (lastTemperature + thermocouple.readCelsius()) / 2;
    lastTemperature = temperature;
    lastTempUpdate = millis();
    return true;
  }
  return false;
}

void setup()
{
  // Ativa os pinos GND e VCC para o módulo MAX6675
  pinMode(THERMO_VCC, OUTPUT); digitalWrite(THERMO_VCC, HIGH);
  pinMode(THERMO_GND, OUTPUT); digitalWrite(THERMO_GND, LOW);
  lcd.begin(16, 2);
  delay(1000);
  
  // Obtém a primeira medida de temperatura
  lastTemperature = thermocouple.readCelsius();
  // Define o pino do relay como um output
  pinMode(RELAY_PIN, OUTPUT);
  
  if(useSimulation)
  {
    for(byte i=0;i<50;i++)
    {
      theta[i]=outputStart;
    }
    modelTime = 0;
  }
  //Setup the pid 
  myPID.SetMode(AUTOMATIC);

  if(tuning)
  {
    tuning=false;
    changeAutoTune();
    tuning=true;
  }
  
  serialTime = 0;
  Serial.begin(9600);
  Parameters customVar; 
  EEPROM.get(0, customVar);

  Serial.println("Read Parameters from EEPROM: ");
  Serial.println(customVar.kp, 10);
  Serial.println(customVar.ki, 10);
  Serial.println(customVar.kd, 10);
}

void loop()
{

  unsigned long now = millis();

  if(!useSimulation)
  { //pull the input in from the real world
    // Atualiza a temperatura
    updateTemperature();
    input = temperature;
  }
  
  if(tuning)
  {
    byte val = (aTune.Runtime());
    if (val!=0)
    {
      tuning = false;
    }
    if(!tuning)
    { //we're done, set the tuning parameters
      kp = aTune.GetKp();
      ki = aTune.GetKi();
      kd = aTune.GetKd();
      Parameters storeVar = {
        kp,
        ki,
        kd
      };
      EEPROM.put(0, storeVar);
      myPID.SetTunings(kp,ki,kd);
      AutoTuneHelper(false);
    }
  }
  else myPID.Compute();
  
  if(useSimulation)
  {
    theta[30]=output;
    if(now>=modelTime)
    {
      modelTime +=100; 
      DoModel();
    }
  }
  else
  {
    // Liga o relê baseado no output do pid
    unsigned long now = millis();
    if (now - windowStartTime > WINDOW_SIZE) {
      windowStartTime += WINDOW_SIZE;
    }
    if ((output * 10) > now - windowStartTime) {
      lcd.setCursor(13, 0); lcd.print(" ON");
      digitalWrite(RELAY_PIN, HIGH);
    } else {
      lcd.setCursor(13, 0); lcd.print("OFF");
      digitalWrite(RELAY_PIN, LOW);
    }
  }
  
  //send-receive with processing if it's time
  if(millis()>serialTime)
  {
    SerialReceive();
    SerialSend();
    serialTime+=500;
  }
}

