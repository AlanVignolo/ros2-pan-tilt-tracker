#include <Arduino.h>

// Prueba de cableado: lee el joystick y el interruptor
// y muestra los valores por el puerto serie cada 100 ms.

const int PIN_VRX = 34;          // eje X del joystick (entrada analógica, ADC1)
const int PIN_VRY = 35;          // eje Y del joystick (entrada analógica, ADC1)
const int PIN_INTERRUPTOR = 25;  // interruptor contra GND

void setup() {
  // Abre el puerto serie por USB a 115200 baudios, para ver los valores en la laptop.
  Serial.begin(115200);

  // INPUT_PULLUP: entrada con la resistencia interna hacia 3,3 V.
  // Con el interruptor en ON el pin queda a GND y lee 0. En OFF lee 1.
  pinMode(PIN_INTERRUPTOR, INPUT_PULLUP);
}

void loop() {
  // analogRead devuelve un número de 0 a 4095:
  // 0 = 0 V, 4095 = 3,3 V. El ADC del ESP32 es de 12 bits (2^12 = 4096 valores).
  int x = analogRead(PIN_VRX);
  int y = analogRead(PIN_VRY);
  int interruptor = digitalRead(PIN_INTERRUPTOR);

  Serial.print("X=");
  Serial.print(x);
  Serial.print("  Y=");
  Serial.print(y);
  Serial.print("  INT=");
  Serial.println(interruptor);   // println agrega el salto de línea al final

  delay(100);   // para la prueba alcanza: 10 lecturas por segundo
}