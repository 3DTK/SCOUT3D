int pwmGreen = 9;
int pwmBlue = 10;

const int triggerPin = 2;
const int strobePin = 3;

int led = 13;

volatile int valueGreen = 0;
volatile int valueBlue = 0;

void setup() {
  pinMode(pwmGreen, OUTPUT);
  pinMode(pwmBlue, OUTPUT);

  pinMode(led, OUTPUT);

  // 31372.55 Hz
  TCCR1B = TCCR1B & B11111000 | B00000001;

  analogWrite(pwmGreen, 0);
  analogWrite(pwmBlue, 0);

  pinMode(triggerPin, INPUT);
  pinMode(strobePin, INPUT);
  
  Serial.begin(115200);
}

void loop() {
  while (Serial.available() > 0) {
    int mode = Serial.parseInt();
    float green = Serial.parseFloat();
    float blue = Serial.parseFloat();

    if (Serial.read() == '\n') {
      valueGreen = constrain(255 * green, 0, 255);
      valueBlue = constrain(255 * blue, 0, 255);
      if (mode == 0) {
        attachInterrupt(digitalPinToInterrupt(strobePin), strobe, CHANGE);
      } else {
        detachInterrupt(digitalPinToInterrupt(strobePin));
        analogWrite(pwmGreen, valueGreen);
        analogWrite(pwmBlue, valueBlue);
      }
    }
  }
}

void strobe() {
  static volatile bool toggleLine = false;
  
  if (digitalRead(strobePin) != 0) {
      analogWrite(pwmGreen, 0);
      analogWrite(pwmBlue, 0);
      
      digitalWrite(led, HIGH);
  } else {
    if (toggleLine) {
        analogWrite(pwmGreen, 0);
        analogWrite(pwmBlue, valueBlue);
    } else {
        analogWrite(pwmGreen, valueGreen);
        analogWrite(pwmBlue, 0);
    }

    digitalWrite(led, LOW);
    
    toggleLine = !toggleLine;
  }
}
