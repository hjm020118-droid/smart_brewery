#include <PID_v1.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include "HX711.h"
#include <math.h> 

// 1. 하드웨어 핀 매핑
#define ONE_WIRE_LIQUID 13   
#define ONE_WIRE_CHAMBER A3  

#define LOADCELL_SCK_PIN A0  
#define LOADCELL_DOUT_PIN A1 

#define LEN_PIN 7            
#define REN_PIN 4            
#define p_heatPin 5          
#define p_coolPin 6          

#define STIR_EN 3            
#define STIR_IN1 8           
#define STIR_IN2 9           

#define FAN_EN 10            
#define FAN_IN3 12           
#define FAN_IN4 11           

// 2. 객체 및 전역 변수 선언
OneWire oneWireLiquid(ONE_WIRE_LIQUID);
OneWire oneWireChamber(ONE_WIRE_CHAMBER);
DallasTemperature sensorLiquid(&oneWireLiquid);
DallasTemperature sensorChamber(&oneWireChamber);
HX711 scale;

double targetTemp = 25.0;
double liquidTemp = 0;
double chamberTemp = 0;
double targetChamberTemp = 0;
double peltierPower = 0;

double mKp = 2.0, mKi = 0.5, mKd = 0.1;
double sKp = 10.0, sKi = 1.0, sKd = 1.0;

PID masterPID(&liquidTemp, &targetChamberTemp, &targetTemp, mKp, mKi, mKd, DIRECT);
PID slavePID(&chamberTemp, &peltierPower, &targetChamberTemp, sKp, sKi, sKd, DIRECT);

int safeHeatingLimit = 100; 

enum PeltierState { COOLING, IDLE, HEATING };
PeltierState currentPeltierState = IDLE;
unsigned long lastPeltierSwitchTime = 0;
const unsigned long PELTIER_DEAD_TIME = 5000; 

float initialWeight = 0;
float currentWeight = 0;
float currentABV = 0.0;

float CALIBRATION_FACTOR = 219.32;  
float EMPTY_BUCKET_WEIGHT = 1200.0;
float MASH_DENSITY = 1.05;
float EVAPORATION_FACTOR = 1.0; 
float mashVolume_mL = 0;

// 정규 교반 스케줄 변수
const unsigned long stirInterval = 12UL * 60UL * 60UL * 1000UL; 
const unsigned long stirDuration = 3UL * 60UL * 1000UL;        
const unsigned long rampDuration = 10UL * 1000UL;
unsigned long lastStirStartTime = 0;
bool isStirring = false;

// 간헐적 펄스 보조 교반 변수
bool isAssistStirring = false; 
unsigned long lastAssistStirTime = 0;
const unsigned long assistStirInterval = 3UL * 60UL * 1000UL; 
const unsigned long assistStirDuration = 15UL * 1000UL;       
const double ASSIST_STIR_THRESHOLD = 50.0; 

const int MIN_STIR_SPEED = 150;    
const int MAX_STIR_SPEED = 255;    

unsigned long previousMillis = 0;
const int MAX_MSG_LENGTH = 32;
char serialBuffer[MAX_MSG_LENGTH];
int bufferIndex = 0;

bool isSystemReady = false; 


// 3. Setup 함수
void setup() {
  Serial.begin(9600); 

  pinMode(LEN_PIN, OUTPUT); pinMode(REN_PIN, OUTPUT); 
  pinMode(p_coolPin, OUTPUT); pinMode(p_heatPin, OUTPUT);
  pinMode(FAN_EN, OUTPUT); pinMode(FAN_IN3, OUTPUT); pinMode(FAN_IN4, OUTPUT);
  pinMode(STIR_EN, OUTPUT); pinMode(STIR_IN1, OUTPUT); pinMode(STIR_IN2, OUTPUT);

  digitalWrite(LEN_PIN, HIGH); digitalWrite(REN_PIN, HIGH);
  
  // 대기 상태일 때는 모터 방향(IN) 핀도 모두 LOW로 차단
  digitalWrite(FAN_IN3, LOW); digitalWrite(FAN_IN4, LOW);
  digitalWrite(STIR_IN1, LOW); digitalWrite(STIR_IN2, LOW);
  
  // 출력(EN/PWM)도 무조건 안전하게 0으로 차단
  digitalWrite(FAN_EN, LOW); analogWrite(STIR_EN, 0);
  analogWrite(p_heatPin, 0); analogWrite(p_coolPin, 0);

  sensorLiquid.begin(); 
  sensorChamber.begin();
  sensorLiquid.setWaitForConversion(false); 
  sensorChamber.setWaitForConversion(false); 
  sensorLiquid.requestTemperatures();       
  sensorChamber.requestTemperatures();

  scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
  scale.set_scale(CALIBRATION_FACTOR);
  delay(1000); 
  scale.tare(); // 전원 켰을 때 초기 무게를 0g으로 셋팅

  masterPID.SetMode(AUTOMATIC);
  masterPID.SetOutputLimits(10.0, 45.0); 
  masterPID.SetSampleTime(1000); 

  slavePID.SetMode(AUTOMATIC);
  slavePID.SetOutputLimits(-255, (double)safeHeatingLimit); 
  slavePID.SetSampleTime(1000); 
}

// 4. Loop 함수

void loop() {
  unsigned long currentMillis = millis();

  // 시리얼 명령 수신 버퍼
  while (Serial.available() > 0) {
    char inChar = Serial.read();
    if (inChar == '\n') {
      serialBuffer[bufferIndex] = '\0';
      
      // 웹에서 [설정 적용하기] 명령이 오면
      if (strncmp(serialBuffer, "SET_TEMP:", 9) == 0) {
        targetTemp = atof(serialBuffer + 9);
        
        if (!isSystemReady) {
          initialWeight = scale.get_units(10); 
          float pureMashWeight = initialWeight - EMPTY_BUCKET_WEIGHT;
          if(pureMashWeight < 0) pureMashWeight = 0; 
          mashVolume_mL = pureMashWeight / MASH_DENSITY; 
          
          isSystemReady = true; 
        }
      }
      else if (strcmp(serialBuffer, "STOP_SYSTEM") == 0) {
        isSystemReady = false;
        peltierPower = 0;
        analogWrite(p_heatPin, 0);
        analogWrite(p_coolPin, 0);
        digitalWrite(FAN_IN3, LOW); 
        digitalWrite(FAN_EN, LOW);  
        digitalWrite(STIR_IN1, LOW); 
        analogWrite(STIR_EN, 0); 
      }

      bufferIndex = 0;
    } else if (bufferIndex < MAX_MSG_LENGTH - 1) {
      serialBuffer[bufferIndex++] = inChar;
    } else {
      bufferIndex = 0;
    }
  }

  if (currentMillis - previousMillis >= 1000) {
    previousMillis = currentMillis;

    readTemperature(); 

    if (!isSystemReady) {
      peltierPower = 0;
      analogWrite(p_heatPin, 0);
      analogWrite(p_coolPin, 0);
      digitalWrite(FAN_IN3, LOW); 
      digitalWrite(FAN_EN, LOW);  
      digitalWrite(STIR_IN1, LOW); 
      analogWrite(STIR_EN, 0);    
      
      sendDataToPi();
      return; 
    }

    calculatePID();    

    if (fabs(targetTemp - liquidTemp) > 0.5 && fabs(peltierPower) >= ASSIST_STIR_THRESHOLD) {
      isAssistStirring = true;
    } else {
      isAssistStirring = false;
    }

    if (!isStirring && !isAssistStirring) { 
      measureWeight();    
      calculateABV();     
    }
    
    controlPeltier(peltierPower, currentMillis); 
    controlFan(peltierPower); 
    sendDataToPi();
  }

  if (isSystemReady) {
    controlStirrer(currentMillis);
  }
}

// 5. 모듈화된 서브 함수
void readTemperature() {
  float t1 = sensorLiquid.getTempCByIndex(0);
  float t2 = sensorChamber.getTempCByIndex(0);
  
  if (t1 > -50.0 && t1 < 125.0) liquidTemp = (double)t1;
  if (t2 > -50.0 && t2 < 125.0) chamberTemp = (double)t2;
  
  sensorLiquid.requestTemperatures(); 
  sensorChamber.requestTemperatures();
}

void measureWeight() {
  if (scale.is_ready()) {
    currentWeight = scale.get_units(3); 
  }
}

void calculateABV() {
  float weightLoss = initialWeight - currentWeight;
  if (weightLoss < 0) weightLoss = 0; 
  
  float ethanolMass = weightLoss * EVAPORATION_FACTOR * 1.047; 
  float ethanolVolume = ethanolMass / 0.789; 
  
  if (mashVolume_mL > 0) {
    currentABV = (ethanolVolume / mashVolume_mL) * 100.0;
  }
}

void calculatePID() {
  if (fabs(targetTemp - liquidTemp) <= 0.2) {
    if (masterPID.GetMode() == AUTOMATIC) {
      masterPID.SetMode(MANUAL); 
      slavePID.SetMode(MANUAL);
      peltierPower = 0;
      targetChamberTemp = chamberTemp; 
    }
  } else {
    if (masterPID.GetMode() == MANUAL) {
      masterPID.SetMode(AUTOMATIC); 
      slavePID.SetMode(AUTOMATIC);
    }
  }
  masterPID.Compute();
  slavePID.Compute();
}

void controlPeltier(double power, unsigned long currentMillis) {
  int finalPower = (int)power;
  PeltierState nextState = IDLE;

  if (finalPower > 0) nextState = HEATING;
  else if (finalPower < 0) nextState = COOLING;

  if ((currentPeltierState == HEATING && nextState == COOLING) || 
      (currentPeltierState == COOLING && nextState == HEATING)) {
    currentPeltierState = IDLE; 
    lastPeltierSwitchTime = currentMillis;
  }

  if (currentMillis - lastPeltierSwitchTime < PELTIER_DEAD_TIME) {
    finalPower = 0;
  } else {
    currentPeltierState = nextState; 
  }

  if (finalPower > 0) { 
    analogWrite(p_coolPin, 0);                 
    analogWrite(p_heatPin, finalPower);      
  } 
  else if (finalPower < 0) { 
    analogWrite(p_heatPin, 0);                 
    analogWrite(p_coolPin, abs(finalPower)); 
  } 
  else {
    analogWrite(p_heatPin, 0);                 
    analogWrite(p_coolPin, 0); 
  }
}

void controlFan(double power) {
  if (power != 0) {
    digitalWrite(FAN_IN3, HIGH); 
    digitalWrite(FAN_EN, HIGH); 
  } else {
    digitalWrite(FAN_IN3, LOW);  
    digitalWrite(FAN_EN, LOW);  
  }
}

void controlStirrer(unsigned long currentMillis) {
  if (!isStirring) {
    if (currentMillis - lastStirStartTime >= stirInterval) {
      isStirring = true;
      lastStirStartTime = currentMillis; 
    } 
  } 
  
  if (isStirring) {
    unsigned long elapsed = currentMillis - lastStirStartTime;
    if (elapsed < rampDuration) {
      digitalWrite(STIR_IN1, HIGH); 
      int currentSpeed = map(elapsed, 0, rampDuration, MIN_STIR_SPEED, MAX_STIR_SPEED);
      analogWrite(STIR_EN, currentSpeed);
    }
    else if (elapsed < stirDuration - rampDuration) {
      digitalWrite(STIR_IN1, HIGH);
      analogWrite(STIR_EN, MAX_STIR_SPEED);
    }
    else if (elapsed < stirDuration) {
      digitalWrite(STIR_IN1, HIGH);
      int currentSpeed = map(elapsed, stirDuration - rampDuration, stirDuration, MAX_STIR_SPEED, MIN_STIR_SPEED);
      analogWrite(STIR_EN, currentSpeed);
    }
    else {
      isStirring = false;
      lastStirStartTime = currentMillis; 
      digitalWrite(STIR_IN1, LOW); 
      analogWrite(STIR_EN, 0); 
    }
  } 
  else if (isAssistStirring) {
    unsigned long elapsedAssist = currentMillis - lastAssistStirTime;
    if (elapsedAssist >= assistStirInterval) {
      lastAssistStirTime = currentMillis;
    }
    if (elapsedAssist < assistStirDuration) {
      digitalWrite(STIR_IN1, HIGH);
      analogWrite(STIR_EN, 200); 
    } else {
      digitalWrite(STIR_IN1, LOW);
      analogWrite(STIR_EN, 0); 
    }
  } 
  else {
    digitalWrite(STIR_IN1, LOW);
    analogWrite(STIR_EN, 0);
    lastAssistStirTime = currentMillis; 
  }
}

void sendDataToPi() {
  int stirStatus = (isStirring || isAssistStirring) ? 1 : 0;
  
  Serial.print("{\"L_TEMP\":"); Serial.print(liquidTemp, 1);
  Serial.print(",\"C_TEMP\":"); Serial.print(chamberTemp, 1);
  Serial.print(",\"T_TEMP\":"); Serial.print(targetTemp, 1);
  Serial.print(",\"ABV\":"); Serial.print(currentABV, 2);
  Serial.print(",\"PWR\":"); Serial.print(peltierPower, 0);
  Serial.print(",\"WGT\":"); Serial.print(currentWeight, 1); 
  Serial.print(",\"STIR\":"); Serial.print(stirStatus); 
  Serial.println("}");
}