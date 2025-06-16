#include <Servo.h>

Servo servo;

const int LED_PIN = 13;  // LED integrado en Arduino Uno

void setup() {
  Serial.begin(9600);
  pinMode(LED_PIN, OUTPUT);

  servo.attach(9);  // Conecta el servo al pin 9
  servo.write(0);   // Posición inicial del servo (pluma abajo)
}

void subirPluma() {
  digitalWrite(LED_PIN, HIGH);
  for (int pos = 0; pos <= 90; pos++) {
    servo.write(pos);
    delay(15);
  }
}

void bajarPluma() {
  digitalWrite(LED_PIN, LOW);
  for (int pos = 90; pos >= 0; pos--) {
    servo.write(pos);
    delay(15);
  }
}

void loop() {
  if (Serial.available()) {
    char data = Serial.read();

    if (data == '1') {
      subirPluma();
    } 
    else if (data == '0') {
      bajarPluma();
    }
  }
}


