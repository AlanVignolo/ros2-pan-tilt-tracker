// Joystick inalámbrico del seguidor.
// Lee un joystick analógico y un switch, y envía por UDP a la Raspberry
// paquetes "J <x> <y> <manual>\n" a 20 Hz, con x e y en el rango -1000..1000.

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include "secrets.h"   // WIFI_SSID y WIFI_PASS, fuera del repositorio (ver secrets.example.h)

// Pines. Los ejes usan ADC1 porque ADC2 no se puede leer con el WiFi activo.
const int PIN_VRX = 34;
const int PIN_VRY = 35;
const int PIN_INTERRUPTOR = 25;  // switch a GND con pull-up interno

const char *IP_RASPBERRY = "192.168.0.50";
const uint16_t PUERTO = 5005;

const int ZONA_MUERTA = 250;     // cuentas de ADC alrededor del centro que se toman como 0
const uint32_t PERIODO_MS = 50;

WiFiUDP udp;
int centro_x = 0;
int centro_y = 0;
uint32_t t_ultimo_envio = 0;

// El centro real de cada eje varía entre joysticks, así que se mide al
// arrancar promediando 32 lecturas con la palanca suelta.
int medir_centro(int pin) {
  long suma = 0;
  for (int i = 0; i < 32; i++) {
    suma += analogRead(pin);
    delay(5);
  }
  return suma / 32;
}

// Promedio de 8 lecturas para filtrar el ruido del ADC del ESP32.
int leer_promedio(int pin) {
  long suma = 0;
  for (int i = 0; i < 8; i++) {
    suma += analogRead(pin);
  }
  return suma / 8;
}

// Escala la lectura a -1000..1000. Cada mitad se escala por separado porque
// el centro no cae justo en 2048, y así ambos extremos llegan a ±1000.
int normalizar(int crudo, int centro) {
  int d = crudo - centro;
  if (abs(d) < ZONA_MUERTA) {
    return 0;
  }
  long valor;
  if (d > 0) {
    valor = (long)(d - ZONA_MUERTA) * 1000 / (4095 - centro - ZONA_MUERTA);
  } else {
    valor = (long)(d + ZONA_MUERTA) * 1000 / (centro - ZONA_MUERTA);
  }
  return constrain(valor, -1000, 1000);
}

void conectar_wifi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.print("Conectando a WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(250);
    Serial.print(".");
  }
  Serial.print(" conectado, IP del ESP32: ");
  Serial.println(WiFi.localIP());
}

void setup() {
  Serial.begin(115200);
  pinMode(PIN_INTERRUPTOR, INPUT_PULLUP);

  // La palanca tiene que estar suelta durante el arranque.
  centro_x = medir_centro(PIN_VRX);
  centro_y = medir_centro(PIN_VRY);
  Serial.printf("Centro medido: X=%d Y=%d\n", centro_x, centro_y);

  conectar_wifi();
}

void loop() {
  // Sin WiFi no se envía nada. El stack del ESP32 reconecta solo, y del lado
  // de la Raspberry el nodo detecta la falta de paquetes y pone los ejes en cero.
  if (WiFi.status() != WL_CONNECTED) {
    return;
  }

  if (millis() - t_ultimo_envio < PERIODO_MS) {
    return;
  }
  t_ultimo_envio = millis();

  int x = normalizar(leer_promedio(PIN_VRX), centro_x);
  int y = normalizar(leer_promedio(PIN_VRY), centro_y);
  int manual = (digitalRead(PIN_INTERRUPTOR) == LOW) ? 1 : 0;

  char mensaje[32];
  snprintf(mensaje, sizeof(mensaje), "J %d %d %d\n", x, y, manual);

  udp.beginPacket(IP_RASPBERRY, PUERTO);
  udp.print(mensaje);
  udp.endPacket();

  Serial.print(mensaje);
}
