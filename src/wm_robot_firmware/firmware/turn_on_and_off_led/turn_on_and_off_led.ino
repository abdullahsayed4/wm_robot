#define Led_PIN 13

void setup() {
  Serial.begin(115200);
  pinMode(Led_PIN, OUTPUT);
  digitalWrite(Led_PIN, LOW);
}

void loop() {
  if (Serial.available()) {
    int x = Serial.readString().toInt();

    if (x == 0) {
      digitalWrite(Led_PIN, LOW);
    } else {
      digitalWrite(Led_PIN, HIGH);
    }
  }
}