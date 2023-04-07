#define CLOCK_PIN 6
#define STROBE_PIN 7
#define DATA_PIN 8

uint8_t bitArrays[] = {0b00000001, 0b00000010, 0b00000100, 0b00001000};

void setup() {
  pinMode(CLOCK_PIN, OUTPUT);
  pinMode(STROBE_PIN, OUTPUT);
  pinMode(DATA_PIN, OUTPUT);
  digitalWrite(CLOCK_PIN, LOW);
  digitalWrite(STROBE_PIN, LOW);
  digitalWrite(DATA_PIN, LOW);
}

void loop() {
  for(uint8_t i = 0; i < 4; i++) {
    for(uint8_t j = 0; j < 8; j++) {
      if((0b10000000 >> j) & bitArrays[i]) {
        digitalWrite(DATA_PIN, HIGH);
      } 
      else {
        digitalWrite(DATA_PIN, LOW);
      }
      delayMicroseconds(2);
      digitalWrite(CLOCK_PIN, HIGH);
      delayMicroseconds(2);
      digitalWrite(CLOCK_PIN, LOW);
      delayMicroseconds(2);
      digitalWrite(DATA_PIN, LOW);
      delayMicroseconds(2);
    }
    digitalWrite(STROBE_PIN, HIGH);
    delayMicroseconds(2);
    digitalWrite(STROBE_PIN, LOW);
    delay(100);
  }
}
