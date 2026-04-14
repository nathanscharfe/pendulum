#define dirPin 11
#define stepPin 10
#define enaPin 12 
#define steps 4000
#define runs 19

long diff = 0; // variable to store the time delta
long start = 0;
long end = 0;
int dir = HIGH;
//int del[runs] = {2400, 2300, 2200, 2100, 2000, 1900, 1800, 1700, 1600, 1500, 1400, 1300, 1200, 1100, 1000, 900, 800, 700, 600, 500, 400, 300, 200}; 
int del[runs] = {190, 180, 170, 160, 150, 140, 130, 120, 110, 100, 90, 80, 70, 60, 50, 40, 30, 20, 10};

void setup() {
  Serial.begin(115200);
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(enaPin, OUTPUT);
  digitalWrite(dirPin, dir);
  digitalWrite(enaPin, HIGH);
  Serial.println("calibrate linear actuator");
  delay(2000);
}

void loop() {
  for (int i = 0; i < runs; i++) {
    digitalWrite(dirPin, dir);
    digitalWrite(enaPin, LOW);
    start = millis();
    move(steps, del[i]);
    end = millis();
    diff = end - start;
    Serial.println(diff);
    digitalWrite(enaPin, HIGH);
    if (dir==HIGH)
      dir = LOW;
    else
      dir = HIGH;
  }
  delay(1000000);
}

void move(int stps, int d) {
  for (int i = 0; i < stps; i++) {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(d);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(d);
  }
}
