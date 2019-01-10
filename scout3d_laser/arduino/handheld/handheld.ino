const int lowPower = 80;
const int highPower = 255;

void setup() {
  analogWrite(DAC0, 0); // blue
  analogWrite(DAC1, 0); // green

  pinMode(2, OUTPUT);
  pinMode(3, OUTPUT);

  digitalWrite(2, 0);
  digitalWrite(3, 0);

  pinMode(8, INPUT);
  digitalWrite(8, 1);
}

void loop() {
  enum {OFF, LOW_POWER, HIGH_POWER};
  static int state = OFF;
  bool stateChanged = false;

  if (digitalRead(8) == 0) {
    long start = millis();

    while (digitalRead(8) == 0) ;

    long elapsed = millis() - start;

    if (elapsed > 100) {

      if (state != OFF) {
        state = OFF;
      } else {
        if (elapsed > 1000) {
          state = HIGH_POWER;
        } else {
          state = LOW_POWER;
        }
      }

      if (state == OFF) {
        analogWrite(DAC0, 0);
        analogWrite(DAC1, 0);

        digitalWrite(2, 0);
        digitalWrite(3, 0);
      } else {
        if (state == HIGH_POWER) {
          analogWrite(DAC0, highPower);
          analogWrite(DAC1, highPower);
        } else {
          analogWrite(DAC0, lowPower);
          analogWrite(DAC1, lowPower);
        }
        digitalWrite(2, 1);
        digitalWrite(3, 1);
      }
    }

    delay(200);
  }
}
