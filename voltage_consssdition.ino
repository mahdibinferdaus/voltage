#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <ZMPT101B.h>
#include <SoftwareSerial.h>

#define volt 500.0f
#define load1 2
#define load2 3
#define supplyrelay 4
#define voltalert 9
#define shotcircuitalert 10
#define loadcondition1 11 // over current hole ekta load2 off kore load1 high kore
#define loadcondition2 12 // ekta load off kore over current thakle
#define loadcondition3 8  // over current test for 20s
#define rstbtn 5
#define warning 13
#define zmpt101b A0
#define shortcircuitcurrent A2
#define loadcurrent A1

SoftwareSerial bt(7, 6);
ZMPT101B voltageSensor(zmpt101b, 50.0);
LiquidCrystal_I2C lcd(0x27, 16, 2);

const int sensorIn = shortcircuitcurrent;
const int sensor2 = loadcurrent;
int mVperAmp = 185;                       
bool pressed = false;
double Voltage = 0;
double VRMS = 0;
double AmpsRMS = 0;
double Voltage2 = 0; //Voltage
double VRMS2 = 0; //VRMS
double Amps = 0; //AmpsRMS

void setup()
{
  Serial.begin(9600);
  lcd.init();
  lcd.backlight();
  voltageSensor.setSensitivity(volt);
  pinMode(rstbtn, INPUT_PULLUP);
  pinMode(warning, OUTPUT);
  pinMode(supplyrelay, OUTPUT);
  pinMode(voltalert, OUTPUT);
  pinMode(load1, OUTPUT);
  pinMode(load2, OUTPUT);
}
void voltage()
{
  Voltage2 = getVPP();
  VRMS2 = (Voltage2 / 2.0) * 0.707; // root 2 is 0.707
  Amps = (VRMS2 * 1000) / mVperAmp;
  float voltage = voltageSensor.getRmsVoltage();
  const float overvolt = 230.0;
  const float lowvolt = 170.0;

  if (voltage > overvolt)
  {
    digitalWrite(supplyrelay, 0);
    digitalWrite(voltalert, 1);
    digitalWrite(warning, 1);
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Voltage: ");
    lcd.print(voltage);
    lcd.setCursor(1, 1);
    lcd.print("OVER VOLTAGE");
    Serial.print("Voltage: ");
    Serial.print(voltage);
    Serial.println("Over volt");
    delay(1000);
  }
  else if (voltage < lowvolt)
  {
    digitalWrite(supplyrelay, 0);
    digitalWrite(voltalert, 1);
    digitalWrite(warning, 1);
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Voltage: ");
    lcd.print(voltage);
    lcd.setCursor(1, 1);
    lcd.print("LOW VOLTAGE");
    Serial.print("Voltage: ");
    Serial.print(voltage);
    Serial.println("Low volt");
    delay(1000);
  } else {
    digitalWrite(supplyrelay, 1);
    digitalWrite(voltalert, 0);
    digitalWrite(warning, 0);
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Voltage: ");
    lcd.print(voltage);
    lcd.setCursor(1,1);
    lcd.print("Current: ");
    lcd.print(Amps);
    delay(1000);
  }
  }

void shortcircuit() {
  Voltage = getVPP2();
  VRMS = (Voltage / 2.0) * 0.707; // root 2 is 0.707
  AmpsRMS = (VRMS * 1000) / mVperAmp;

  if (AmpsRMS >= 0.3) {
    digitalWrite(supplyrelay, 0);
    digitalWrite(shotcircuitalert, 1);
    digitalWrite(warning, 1);
    lcd.clear();
    lcd.print("SHORT CIRCUIT");
    delay(1000);

  while (AmpsRMS >= 0.3) {
      bool Redbtn = digitalRead(rstbtn);
      if (Redbtn == pressed) {
        digitalWrite(supplyrelay, 1);
        digitalWrite(shotcircuitalert, 0);
        digitalWrite(warning, 0);

        // Wait until the reset button is released
        while (digitalRead(rstbtn) == pressed) {
          // Do nothing while button is pressed
        }

        // Break out of the infinite loop
        break;
      }
    }
  }
}
void load() {
  Voltage2 = getVPP();
  VRMS2 = (Voltage2 / 2.0) * 0.707; // root 2 is 0.707
  Amps = (VRMS2 * 1000) / mVperAmp;
  int Watt = (Voltage2 * Amps);

  const float currentThreshold = 0.5; // Threshold for overcurrent detection
  const unsigned long delayTime = 10000; // 10 seconds delay

  if (Amps >= currentThreshold) {
    digitalWrite(load1, HIGH);
    digitalWrite(load2, LOW);
    digitalWrite(warning, HIGH);
  }

  while (Amps >= currentThreshold) {
    bool redBtn = digitalRead(rstbtn);
    
    if (redBtn == pressed) {
      digitalWrite(load1, LOW);
      digitalWrite(load2, LOW);
      digitalWrite(warning, LOW);
      
      // Wait until the reset button is released
      while (digitalRead(rstbtn) == pressed) {
        // Do nothing while button is pressed
      }

      delay(delayTime);

      Voltage2 = getVPP();
      VRMS2 = (Voltage2 / 2.0) * 0.707;
      Amps = (VRMS2 * 1000) / mVperAmp;

      if (Amps >= currentThreshold) {
        digitalWrite(load1, HIGH);
        digitalWrite(load2, HIGH);
        delay(delayTime);
      } else {
        digitalWrite(load1, LOW);
        digitalWrite(load2, LOW);
        digitalWrite(warning, LOW);
        break;
      }
    }
  }
}

void loop()
{
  voltage();
  shortcircuit();
  load ();
}
float getVPP()
{
  float result;
  int readValue;       // value read from the sensor
  int maxValue = 0;    // store max value here
  int minValue = 1024; // store min value here

  uint32_t start_time = millis();
  while ((millis() - start_time) < 1000) // sample for 1 Sec
  {
    readValue = analogRead(sensor2);
    // see if you have a new maxValue
    if (readValue > maxValue)
    {
      /*record the maximum sensor value*/
      maxValue = readValue;
    }
    if (readValue < minValue)
    {
      /*record the minimum sensor value*/
      minValue = readValue;
    }
  }

  // Subtract min from max
  result = ((maxValue - minValue) * 5.0) / 1024.0;

  return result;
}
float getVPP2()
{
  float result;
  int readValue;       // value read from the sensor
  int maxValue = 0;    // store max value here
  int minValue = 1024; // store min value here

  uint32_t start_time = millis();
  while ((millis() - start_time) < 1000) // sample for 1 Sec
  {
    readValue = analogRead(sensorIn);
    // see if you have a new maxValue
    if (readValue > maxValue)
    {
      /*record the maximum sensor value*/
      maxValue = readValue;
    }
    if (readValue < minValue)
    {
      /*record the minimum sensor value*/
      minValue = readValue;
    }
  }

  // Subtract min from max
  result = ((maxValue - minValue) * 5.0) / 1024.0;

  return result;
}
