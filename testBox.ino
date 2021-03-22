#include <movingAvg.h>

#define SNS_POT A1
#define TIME_POT A2
#define CUR_SENSE A3
#define LED_RED digitalWrite(9,HIGH);analogWrite(10,128);
#define LED_BLUE analogWrite(9,128);digitalWrite(10,HIGH);digitalWrite(11,HIGH);
#define LED_GREEN digitalWrite(9,HIGH);digitalWrite(10,HIGH);analogWrite(11,128);
#define LED_OFF digitalWrite(9,HIGH);digitalWrite(10,HIGH);digitalWrite(11,HIGH);
#define RELAY 13
#define SWITCH 12

#define TIME_OFFSET 1000 // 1 second
#define TIME_MULT 30 * 60 // about 30 min

#define CUR_OFFSET 0
#define CUR_MULT 0.5

movingAvg aveNoCurrentCalc(32);
int16_t aveNoCurrent;

void setup() {
  // SETUP
  Serial.begin(115200);
  for (uint8_t i=9; i<= 11; i++) {
    pinMode(i, OUTPUT);
    digitalWrite(i, HIGH);
  }
  pinMode(SWITCH, INPUT);
  digitalWrite(SWITCH, HIGH);
  pinMode(RELAY, OUTPUT);
  digitalWrite(RELAY, HIGH); // turn off power to outlet
  aveNoCurrentCalc.begin();
  
  // CALIBRATION
  for (uint16_t i=0; i< 1024; i++) {
    aveNoCurrent = aveNoCurrentCalc.reading(analogRead(CUR_SENSE));
  }
  Serial.println(aveNoCurrent);
  //delay(1000);
}

void loop() {
  int16_t current = 0, tmp;
  static uint32_t timeTurnedOn;
  static boolean switchIsOff = true;
  static boolean testTime = true;
  
  if (digitalRead(SWITCH)) {
    digitalWrite(RELAY, HIGH); // shut off
    LED_RED;
    switchIsOff = true;
    return;
  }

  if (switchIsOff) {
    switchIsOff = false;
    testTime = true;
    timeTurnedOn = millis();
    LED_BLUE;
  } else {
    uint32_t timeOff = analogRead(TIME_POT);
    timeOff = timeOff * TIME_MULT + TIME_OFFSET;
    if (not(testTime) || (millis() > (timeTurnedOn + timeOff))) {
      testTime = false;
      digitalWrite(RELAY, HIGH); // shut off
      LED_RED;
      return;
    }
  }
  digitalWrite(RELAY,LOW);

  uint32_t start = millis();
  while (millis() < (start + 20)) {
    tmp = analogRead(CUR_SENSE);
    current = max(current, tmp);
  }

  current = abs(current - aveNoCurrent);

  float sensitivity = analogRead(SNS_POT);
  sensitivity = sensitivity * CUR_MULT + CUR_OFFSET;
  if (current > sensitivity) {
    LED_GREEN;
    timeTurnedOn = millis();
  } else {
    LED_BLUE;
  }
  
  Serial.println(current);
  delay(100);
}
