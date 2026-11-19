#include <Arduino.h>
#include <WiFi.h>      // conexión WiFi del ESP32
#include <WiFiUdp.h>   // envío de paquetes UDP
#include "secrets.h"   // WIFI_SSID y WIFI_PASS (no está en el repositorio)

// --- Pines ---
const int PIN_VRX = 34;          // eje X del joystick (ADC1)
const int PIN_VRY = 35;          // eje Y del joystick (ADC1)
const int PIN_INTERRUPTOR = 25;  // interruptor contra GND, con pull-up interno

// --- Red ---
const char *IP_RASPBERRY = "192.168.0.50";  // dirección fija de la Raspberry por cable
const uint16_t PUERTO = 5005;               // puerto donde la Raspberry escucha

// --- Joystick ---
const int ZONA_MUERTA = 250;      // cuentas del ADC alrededor del centro que se toman como 0
const uint32_t PERIODO_MS = 50;   // un envío cada 50 ms = 20 por segundo

WiFiUDP udp;                 // objeto que maneja el envío de paquetes UDP
int centro_x = 0;            // valor del ADC con la palanca suelta, medido al arrancar
int centro_y = 0;
uint32_t t_ultimo_envio = 0; // momento del último envío, en ms desde que arrancó

// Promedia 32 lecturas con la palanca suelta para encontrar el centro real.
// Un promedio reduce el efecto del ruido de cada lectura individual.
int medir_centro(int pin) {
  long suma = 0;
  for (int i = 0; i < 32; i++) {
    suma += analogRead(pin);
    delay(5);
  }
  return suma / 32;
}

// Lee el pin 8 veces seguidas y devuelve el promedio.
// El ruido sube y baja al azar, así que al promediar se cancela en parte.
// 8 lecturas tardan unos pocos microsegundos: no afecta al envío cada 50 ms.
int leer_promedio(int pin) {
  long suma = 0;
  for (int i = 0; i < 8; i++) {
    suma += analogRead(pin);
  }
  return suma / 8;
}

// Convierte una lectura del ADC (0 a 4095) a un valor de -1000 a 1000.
// Cada lado del centro se escala por separado, porque el centro no está
// justo en la mitad: así los dos extremos llegan a 1000 y -1000.
int normalizar(int crudo, int centro) {
  int d = crudo - centro;
  if (abs(d) < ZONA_MUERTA) {
    return 0;   // dentro de la zona muerta: palanca considerada suelta
  }
  long valor;
  if (d > 0) {
    valor = (long)(d - ZONA_MUERTA) * 1000 / (4095 - centro - ZONA_MUERTA);
  } else {
    valor = (long)(d + ZONA_MUERTA) * 1000 / (centro - ZONA_MUERTA);
  }
  return constrain(valor, -1000, 1000);   // constrain limita el valor entre dos extremos
}

// Se conecta al WiFi y espera hasta lograrlo, mostrando el avance.
void conectar_wifi() {
  WiFi.mode(WIFI_STA);   // STA (station): se conecta a una red existente, como un celular
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

  // El centro se mide al arrancar: la palanca tiene que estar suelta.
  centro_x = medir_centro(PIN_VRX);
  centro_y = medir_centro(PIN_VRY);
  Serial.printf("Centro medido: X=%d Y=%d\n", centro_x, centro_y);

  conectar_wifi();
}

void loop() {
  // Si se cayó el WiFi, no se envía nada. El ESP32 se reconecta solo.
  if (WiFi.status() != WL_CONNECTED) {
    return;
  }

  // millis(): milisegundos desde que arrancó. Mismo esquema que el LED de la Nucleo:
  // se compara con el último envío en lugar de usar delay, así nada queda bloqueado.
  if (millis() - t_ultimo_envio < PERIODO_MS) {
    return;
  }
  t_ultimo_envio = millis();

  // Cada eje se lee promediado, para reducir el ruido del ADC.
  int x = normalizar(leer_promedio(PIN_VRX), centro_x);
  int y = normalizar(leer_promedio(PIN_VRY), centro_y);
  int manual = (digitalRead(PIN_INTERRUPTOR) == LOW) ? 1 : 0;  // ON = pin en 0 = manual

  char mensaje[32];
  snprintf(mensaje, sizeof(mensaje), "J %d %d %d\n", x, y, manual);

  // Un paquete UDP se arma en tres pasos: abrir hacia un destino, escribir, enviar.
  udp.beginPacket(IP_RASPBERRY, PUERTO);
  udp.print(mensaje);
  udp.endPacket();

  Serial.print(mensaje);   // copia por el monitor serie, para ver qué se envía
}