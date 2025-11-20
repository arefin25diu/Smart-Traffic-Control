// ESP32 Traffic Control

#define BLYNK_TEMPLATE_ID "TMPL6yoO29M1V"
#define BLYNK_TEMPLATE_NAME "ESP32 Traffic Control "
#define BLYNK_AUTH_TOKEN "bKZ2tFFeHVOITK8axuj9ciQExaCxaXQD"

#include <WiFi.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>


char ssid[] = "Tanjid";
char pass[] = "Tanjid@5";

#define IR1 32
#define IR2 13

#define IR3 14
#define IR4 33

#define TRIG 23
#define ECHO 34

#define L1_RED 16
#define L1_YELLOW 17
#define L1_GREEN 18

#define L2_RED 19
#define L2_YELLOW 21
#define L2_GREEN 22

#define L3_RED 25
#define L3_YELLOW 26
#define L3_GREEN 27

const unsigned long GREEN_TIME = 8000;   
const unsigned long YELLOW_TIME = 3000;   
const unsigned long RED_TIME = 5000;     

// State Control
unsigned long phaseStart = 0;
int activeLane = 0;
int phase = 0;       
bool sequentialMode = false;

// Functions
long readUltrasonic() {
  digitalWrite(TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);

  long duration = pulseIn(ECHO, HIGH, 30000); 
  long distance = duration * 0.034 / 2;      
  return distance;
}

int getLane1Density() {
  int count = 0;
  if (digitalRead(IR1) == LOW) count++;
  if (digitalRead(IR2) == LOW) count++;
  return count;
}

int getLane2Density() {
  int count = 0;
  if (digitalRead(IR3) == LOW) count++;
  if (digitalRead(IR4) == LOW) count++;
  return count;
}

int getLane3Density() {
  long d = readUltrasonic();
  if (d > 0 && d < 40) return 1; // Vehicle within 40cm
  return 0;
}

void setLights(int lane, int state) {
  // state: 0=RED, 1=YELLOW, 2=GREEN
  digitalWrite(L1_RED, LOW); digitalWrite(L1_YELLOW, LOW); digitalWrite(L1_GREEN, LOW);
  digitalWrite(L2_RED, LOW); digitalWrite(L2_YELLOW, LOW); digitalWrite(L2_GREEN, LOW);
  digitalWrite(L3_RED, LOW); digitalWrite(L3_YELLOW, LOW); digitalWrite(L3_GREEN, LOW);

  if (lane == 1) {
    if (state == 0) digitalWrite(L1_RED, HIGH);
    else if (state == 1) digitalWrite(L1_YELLOW, HIGH);
    else if (state == 2) digitalWrite(L1_GREEN, HIGH);
  }
  else if (lane == 2) {
    if (state == 0) digitalWrite(L2_RED, HIGH);
    else if (state == 1) digitalWrite(L2_YELLOW, HIGH);
    else if (state == 2) digitalWrite(L2_GREEN, HIGH);
  }
  else if (lane == 3) {
    if (state == 0) digitalWrite(L3_RED, HIGH);
    else if (state == 1) digitalWrite(L3_YELLOW, HIGH);
    else if (state == 2) digitalWrite(L3_GREEN, HIGH);
  }
}

// Sequential Cycle 
void runSequentialCycle() {
  if (phase == 0) {
    activeLane = 1;
    setLights(activeLane, 2); // Green
    phaseStart = millis();
    phase = 1;
    sequentialMode = true;
  }
  else if (phase == 1 && millis() - phaseStart >= GREEN_TIME) {
    setLights(activeLane, 1); // Yellow
    phaseStart = millis();
    phase = 2;
  }
  else if (phase == 2 && millis() - phaseStart >= YELLOW_TIME) {
    setLights(activeLane, 0); // Red
    phaseStart = millis();
    phase = 3;
  }
  else if (phase == 3 && millis() - phaseStart >= RED_TIME) {
    activeLane++;
    if (activeLane > 3) activeLane = 1;
    setLights(activeLane, 2); // Next lane Green
    phaseStart = millis();
    phase = 1;
  }
}

// Max Density 
void runMaxDensity(int d1, int d2, int d3) {
  int maxLane = 1;
  int maxVal = d1;

  if (d2 > maxVal) { maxLane = 2; maxVal = d2; }
  if (d3 > maxVal) { maxLane = 3; maxVal = d3; }

  activeLane = maxLane;
  sequentialMode = false;

  digitalWrite(L1_RED, LOW); digitalWrite(L1_YELLOW, LOW); digitalWrite(L1_GREEN, LOW);
  digitalWrite(L2_RED, LOW); digitalWrite(L2_YELLOW, LOW); digitalWrite(L2_GREEN, LOW);
  digitalWrite(L3_RED, LOW); digitalWrite(L3_YELLOW, LOW); digitalWrite(L3_GREEN, LOW);

  if (activeLane == 1) {
    digitalWrite(L1_GREEN, HIGH);
    digitalWrite(L2_RED, HIGH);
    digitalWrite(L3_RED, HIGH);
  }
  else if (activeLane == 2) {
    digitalWrite(L2_GREEN, HIGH);
    digitalWrite(L1_RED, HIGH);
    digitalWrite(L3_RED, HIGH);
  }
  else if (activeLane == 3) {
    digitalWrite(L3_GREEN, HIGH);
    digitalWrite(L1_RED, HIGH);
    digitalWrite(L2_RED, HIGH);
  }

  Serial.printf("Lane %d GREEN, others RED\n", activeLane);
}


void setup() {
  Serial.begin(115200);
  Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass);

  pinMode(IR1, INPUT);
  pinMode(IR2, INPUT);
  pinMode(IR3, INPUT);
  pinMode(IR4, INPUT);

  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT);

  pinMode(L1_RED, OUTPUT);
  pinMode(L1_YELLOW, OUTPUT);
  pinMode(L1_GREEN, OUTPUT);

  pinMode(L2_RED, OUTPUT);
  pinMode(L2_YELLOW, OUTPUT);
  pinMode(L2_GREEN, OUTPUT);

  pinMode(L3_RED, OUTPUT);
  pinMode(L3_YELLOW, OUTPUT);
  pinMode(L3_GREEN, OUTPUT);

  setLights(1, 0);
  setLights(2, 0);
  setLights(3, 0);
}


void loop() {
  Blynk.run();

  int d1 = getLane1Density();
  int d2 = getLane2Density();
  int d3 = getLane3Density();
  long dist = readUltrasonic();

  Serial.printf("Lane1=%d Lane2=%d Lane3=%d\n", d1, d2, d3);

  Blynk.virtualWrite(V0, d1);
  Blynk.virtualWrite(V1, d2);
  Blynk.virtualWrite(V6, d3);
  Blynk.virtualWrite(V7, dist);

  int irBits = 0;
  if (digitalRead(IR1) == LOW) irBits |= 1 << 0;
  if (digitalRead(IR2) == LOW) irBits |= 1 << 1;
  if (digitalRead(IR3) == LOW) irBits |= 1 << 2;
  if (digitalRead(IR4) == LOW) irBits |= 1 << 3;
  Blynk.virtualWrite(V5, irBits);

  bool tieMode = (d1 == d2 && d2 == d3);
  Blynk.virtualWrite(V3, tieMode ? 1 : 0);

  if (tieMode) {
    runSequentialCycle();
  } else {
    runMaxDensity(d1, d2, d3);
  }

  String phaseText;
  if (phase == 1) phaseText = "GREEN";
  else if (phase == 2) phaseText = "YELLOW";
  else if (phase == 3) phaseText = "RED";
  else phaseText = "IDLE";

  String lanePhase = "LANE" + String(activeLane) + " " + phaseText;
  Blynk.virtualWrite(V2, lanePhase);
  Blynk.virtualWrite(V4, activeLane);
}