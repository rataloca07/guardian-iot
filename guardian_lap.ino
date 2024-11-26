/*#include <SoftwareSerial.h>
#include <TinyGPS++.h>

// Configuración del módulo GPS
SoftwareSerial gpsSerial(4, 3); // RX, TX (GPS Neo 6M conectado a D4 (TX) y D3 (RX))
TinyGPSPlus gps;

unsigned long previousMillis = 0;
const long interval = 3000;  // Intervalo de 3 segundos

void setup() {
  Serial.begin(9600);
  gpsSerial.begin(9600);
  Serial.println("Esperando señal GPS...");
}

void loop() {
  while (gpsSerial.available() > 0) {
    char c = gpsSerial.read();
    gps.encode(c);  // Procesar el carácter entrante
    Serial.write(c);  // Mostrar los datos sin procesar del GPS para verificar la salida
  }

  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;  // Actualizar el tiempo

    if (gps.location.isUpdated()) {
      Serial.print("Latitud: ");
      Serial.println(gps.location.lat(), 6);  // Mostrar la latitud con 6 decimales
      Serial.print("Longitud: ");
      Serial.println(gps.location.lng(), 6);  // Mostrar la longitud con 6 decimales
    } else {
      Serial.println("No hay datos GPS disponibles aún...");
    }
  }
}*/

/*#include <SoftwareSerial.h>
#include <TinyGPS++.h>

// Configuración del módulo GPS
SoftwareSerial gpsSerial(4, 3); // RX, TX (GPS Neo 6M conectado a D4 (TX) y D3 (RX))
TinyGPSPlus gps;

unsigned long previousMillis = 0;
const long interval = 3000;  // Intervalo de 3 segundos

void setup() {
  Serial.begin(9600);
  gpsSerial.begin(9600);
  Serial.println("Esperando señal GPS...");
}

void loop() {
  while (gpsSerial.available() > 0) {
    char c = gpsSerial.read();
    //Serial.println("Data raw 1: ");
    gps.encode(c);  // Procesar el carácter entrante
    Serial.write(c);  // Mostrar los datos sin procesar del GPS para verificar la salida
  }

  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;  // Actualizar el tiempo

    if (gps.location.isUpdated()) {
      // Almacenamos los valores explícitamente como double
      double latitud = gps.location.lat();
      double longitud = gps.location.lng();

      // Mostramos las coordenadas en formato double con 6 decimales
      Serial.print("Latitud: ");
      Serial.println(latitud, 6);  // Mostrar la latitud con 6 decimales
      Serial.print("Longitud: ");
      Serial.println(longitud, 6);  // Mostrar la longitud con 6 decimales
    } else {
      Serial.println("No hay datos GPS disponibles aún...");
    }
  }
}*/

 /*#include <SoftwareSerial.h>
SoftwareSerial Dta(4,3);

void setup(){
  Serial.begin(9600);
  Dta.begin(9600);

}

void loop(){
while(Dta.available()>0){
  byte gpsData = Dta.read();
  Serial.write(gpsData);
}
}*/

/*#include <SoftwareSerial.h>
SoftwareSerial Dta(4, 3);  // Configuración para el módulo GPS (RX = D4, TX = D3)

// Variables de tiempo
unsigned long previousMillis = 0;
const long interval = 5000;  // Intervalo de 5 segundos

void setup() {
  Serial.begin(9600);
  Dta.begin(9600);
}

void loop() {
  // Verifica si hay datos disponibles del GPS
  while (Dta.available() > 0) {
    byte gpsData = Dta.read();
    //Serial.write(gpsData);  // Imprimir los datos crudos del GPS
    processGPSData(gpsData);  // Procesar los datos NMEA para extraer las coordenadas
  }

  // Control de tiempo para ejecutar cada 5 segundos
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    printCoordinates();  // Imprimir las coordenadas convertidas a grados decimales
  }
}

// Variables globales para almacenar latitud y longitud en grados decimales
double latitude = 0.0;
double longitude = 0.0;

// Función para procesar los datos NMEA del GPS
void processGPSData(byte gpsData) {
  static char nmeaSentence[100];
  static int index = 0;

  // Construimos una cadena con los datos NMEA
  nmeaSentence[index++] = gpsData;
  if (gpsData == '\n') {
    nmeaSentence[index] = '\0';  // Finalizamos la cadena
    index = 0;

    // Procesamos solo las sentencias $GPGLL, $GPRMC o $GPGGA para extraer las coordenadas
    if (strstr(nmeaSentence, "$GPGLL") || strstr(nmeaSentence, "$GPRMC") || strstr(nmeaSentence, "$GPGGA")) {
      extractCoordinates(nmeaSentence);
    }
  }
}

// Función para extraer las coordenadas de las sentencias NMEA
void extractCoordinates(char *nmeaSentence) {
  char *token;
  char latDir, lonDir;
  float rawLat, rawLon;

  // $GPGLL
  if (strstr(nmeaSentence, "$GPGLL")) {
    token = strtok(nmeaSentence, ",");  // Saltamos el identificador $GPGLL
    token = strtok(NULL, ",");  // Latitud cruda
    rawLat = atof(token);
    token = strtok(NULL, ",");  // Dirección de la latitud (N/S)
    latDir = token[0];
    token = strtok(NULL, ",");  // Longitud cruda
    rawLon = atof(token);
    token = strtok(NULL, ",");  // Dirección de la longitud (E/W)
    lonDir = token[0];

    // Convertimos las coordenadas crudas a grados decimales
    latitude = convertToDecimalDegrees(rawLat, latDir);
    longitude = convertToDecimalDegrees(rawLon, lonDir);
  }

  // $GPRMC
  if (strstr(nmeaSentence, "$GPRMC")) {
    token = strtok(nmeaSentence, ",");  // Saltamos el identificador $GPRMC
    token = strtok(NULL, ",");  // Hora UTC
    token = strtok(NULL, ",");  // Estado (A = datos válidos, V = no válidos)
    if (*token == 'A') {
      token = strtok(NULL, ",");  // Latitud cruda
      rawLat = atof(token);
      token = strtok(NULL, ",");  // Dirección de la latitud (N/S)
      latDir = token[0];
      token = strtok(NULL, ",");  // Longitud cruda
      rawLon = atof(token);
      token = strtok(NULL, ",");  // Dirección de la longitud (E/W)
      lonDir = token[0];

      // Convertimos las coordenadas crudas a grados decimales
      latitude = convertToDecimalDegrees(rawLat, latDir);
      longitude = convertToDecimalDegrees(rawLon, lonDir);
    }
  }

  // $GPGGA
  if (strstr(nmeaSentence, "$GPGGA")) {
    token = strtok(nmeaSentence, ",");  // Saltamos el identificador $GPGGA
    token = strtok(NULL, ",");  // Hora UTC
    token = strtok(NULL, ",");  // Latitud cruda
    rawLat = atof(token);
    token = strtok(NULL, ",");  // Dirección de la latitud (N/S)
    latDir = token[0];
    token = strtok(NULL, ",");  // Longitud cruda
    rawLon = atof(token);
    token = strtok(NULL, ",");  // Dirección de la longitud (E/W)
    lonDir = token[0];

    // Convertimos las coordenadas crudas a grados decimales
    latitude = convertToDecimalDegrees(rawLat, latDir);
    longitude = convertToDecimalDegrees(rawLon, lonDir);
  }
}

// Función para convertir coordenadas en grados y minutos a grados decimales
double convertToDecimalDegrees(float rawCoordinate, char direction) {
  int degrees = (int)(rawCoordinate / 100);
  float minutes = rawCoordinate - (degrees * 100);
  double decimalDegrees = degrees + (minutes / 60);

  // Si es latitud sur o longitud oeste, hacemos que el valor sea negativo
  if (direction == 'S' || direction == 'W') {
    decimalDegrees *= -1;
  }

  return decimalDegrees;
}

// Función para imprimir las coordenadas en la consola
void printCoordinates() {
  Serial.print("Latitud (grados decimales): ");
  Serial.println(latitude, 6);  // Imprimir latitud con 6 decimales
  Serial.print("Longitud (grados decimales): ");
  Serial.println(longitude, 6);  // Imprimir longitud con 6 decimales
}*/


// para ver la gráfica y calibrar luego
/*const int pulsePin = A0;  // Pin donde está conectado el sensor de pulso

void setup() {
  Serial.begin(9600);  // Iniciar comunicación serie
}

void loop() {
  int pulseValue = analogRead(pulsePin);  // Leer el valor analógico del sensor
  Serial.println(pulseValue);  // Mostrar el valor en el monitor serie
  delay(100);  // Pausa para evitar saturar el monitor serie
}*/

/*const int pulsePin = A0;  // Pin donde está conectado el sensor de pulso

int upperThreshold = 750;  // Umbral superior ajustado para detectar latidos
unsigned long lastBeatTime = 0;  // Tiempo del último latido detectado
unsigned long previousMillis = 0;  // Tiempo anterior para calcular BPM
unsigned long interval = 1000;  // Intervalo de 1 segundo para calcular BPM
int beatCount = 0;  // Contador de latidos

void setup() {
  Serial.begin(9600);  // Iniciar comunicación serie
  Serial.println("Iniciando lectura del sensor de pulso...");
}

void loop() {
  int pulseValue = analogRead(pulsePin);  // Leer el valor del sensor

  // Detectar un latido solo si supera el umbral superior
  if (pulseValue > upperThreshold) {
    unsigned long currentMillis = millis();

    // Verificar que haya pasado suficiente tiempo desde el último latido (debounce de 300 ms)
    if (currentMillis - lastBeatTime > 300) {  // 300 ms para evitar latidos falsos
      beatCount++;  // Incrementar contador de latidos
      lastBeatTime = currentMillis;  // Actualizar tiempo del último latido detectado
    }
  }

  // Calcular BPM cada segundo
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;

    // Calcular las BPM (latidos por minuto)
    int bpm = beatCount * 60;  // Multiplicar por 60 porque contamos latidos por segundo

    // Imprimir solo si el valor de BPM es mayor a 0
    if (bpm > 0) {
      Serial.print("Pulsaciones por minuto (BPM): ");
      Serial.println(bpm);
    }

    // Reiniciar el contador de latidos
    beatCount = 0;
  }

  delay(20);  // Pequeña pausa para suavizar las lecturas
}*/

/*const int pulsePin = A0;  // Pin donde está conectado el sensor de pulso

void setup() {
  Serial.begin(9600);  // Iniciar la comunicación serie
  Serial.println("Iniciando la lectura del sensor de pulso...");
}

void loop() {
  // Leer el valor del sensor de pulso
  int pulseValue = analogRead(pulsePin);

  // Enviar el valor al puerto serie
  Serial.println(pulseValue);

  // Pausa para evitar saturar el monitor serie
  delay(100);  // Ajusta el delay según la frecuencia deseada para la captura de datos
}*/
/*const int pulsePin = A0;  // Pin donde está conectado el sensor de pulso

int changeThreshold = 50;  // Umbral de cambio entre valores consecutivos para contar un latido
unsigned long lastBeatTime = 0;  // Tiempo del último latido detectado
unsigned long previousMillis = 0;  // Tiempo anterior para calcular BPM
unsigned long interval = 1000;  // Intervalo de 1 segundo para calcular BPM
int beatCount = 0;  // Contador de latidos
int lastPulseValue = 0;  // Último valor del sensor
int smoothValue = 0;  // Valor suavizado del sensor

void setup() {
  Serial.begin(9600);  // Iniciar comunicación serie
  Serial.println("Iniciando lectura del sensor de pulso...");
}

void loop() {
  // Leer el valor actual del sensor de pulso
  int pulseValue = analogRead(pulsePin);

  // Suavizar la lectura con un filtro simple
  smoothValue = (smoothValue * 0.8) + (pulseValue * 0.2);

  // Detectar un latido si la diferencia con el valor anterior es mayor al umbral
  if (abs(smoothValue - lastPulseValue) > changeThreshold) {
    unsigned long currentMillis = millis();

    // Verificar que haya pasado suficiente tiempo desde el último latido (debounce de 500 ms)
    if (currentMillis - lastBeatTime > 500) {  // 500 ms para evitar latidos falsos
      beatCount++;  // Incrementar contador de latidos
      lastBeatTime = currentMillis;  // Actualizar tiempo del último latido detectado
    }
  }

  // Actualizar el último valor de pulso
  lastPulseValue = smoothValue;

  // Calcular BPM cada segundo
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;

    // Calcular las BPM (latidos por minuto)
    int bpm = beatCount * 60;  // Multiplicar por 60 porque contamos latidos por segundo

    // Imprimir solo si el valor de BPM es mayor a 0
    if (bpm > 0) {
      Serial.print("Pulsaciones por minuto (BPM): ");
      Serial.println(bpm);
    }

    // Reiniciar el contador de latidos
    beatCount = 0;
  }

  delay(20);  // Pausa pequeña antes de la próxima lectura
}*/

/*const int pulsePin = A0;  // Pin donde está conectado el sensor de pulso

void setup() {
  Serial.begin(9600);  // Iniciar comunicación serie
  Serial.println("Iniciando lectura del sensor de pulso...");
}

void loop() {
  // Leer el valor del sensor de pulso
  int pulseValue = analogRead(pulsePin);

  // Enviar el valor al puerto serie
  Serial.println(pulseValue);

  // Pausa para evitar saturar el monitor serie
  delay(100);  // Ajusta el delay según la frecuencia deseada para la captura de datos
}*/


/*const int pulsePin = A0;  // Pin donde está conectado el sensor de pulso
int changeThreshold = 150;  // Umbral para detectar cambios significativos
unsigned long lastBeatTime = 0;  // Tiempo del último latido detectado
unsigned long previousMillis = 0;  // Tiempo anterior para calcular BPM
unsigned long interval = 15000;  // Intervalo de 15 segundos para calcular BPM
int beatCount = 0;  // Contador de latidos
int lastPulseValue = 0;  // Último valor del sensor

void setup() {
  Serial.begin(9600);  // Iniciar comunicación serie
  Serial.println("Iniciando lectura del sensor de pulso...");
}

void loop() {
  // Leer el valor actual del sensor de pulso
  int pulseValue = analogRead(pulsePin);

  // Imprimir los valores del sensor de pulso para verificar la lectura
  Serial.print("Valor del sensor: ");
  Serial.println(pulseValue);

  // Detectar un latido si la diferencia con el valor anterior es mayor al umbral
  if (abs(pulseValue - lastPulseValue) > changeThreshold) {
    unsigned long currentMillis = millis();

    // Verificar que haya pasado suficiente tiempo desde el último latido (debounce de 600 ms)
    if (currentMillis - lastBeatTime > 600) {  // 600 ms para evitar latidos falsos
      beatCount++;  // Incrementar contador de latidos
      lastBeatTime = currentMillis;  // Actualizar tiempo del último latido detectado
    }
  }

  // Actualizar el último valor de pulso
  lastPulseValue = pulseValue;

  // Calcular BPM cada 15 segundos
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;

    // Calcular las BPM (latidos por minuto)
    int bpm = beatCount * (60 / 15);  // Multiplicar por 60 y dividir por 15 porque contamos latidos en 15 segundos

    // Imprimir solo si el valor de BPM es mayor a 0
    if (bpm > 0) {
      Serial.print("Pulsaciones por minuto (BPM): ");
      Serial.println(bpm);
    }

    // Reiniciar el contador de latidos
    beatCount = 0;
  }

  delay(20);  // Pausa pequeña antes de la próxima lectura
}*/

//Aquiiiiiiiiiiiiiiiiiiiiii

/*#include <SoftwareSerial.h>
SoftwareSerial Dta(4, 3);  // Configuración para el módulo GPS (RX = D4, TX = D3)

// Variables de tiempo
unsigned long previousMillis = 0;
const long interval = 5000;  // Intervalo de 5 segundos

void setup() {
  Serial.begin(9600);
  Dta.begin(9600);
}

void loop() {
  // Verifica si hay datos disponibles del GPS
  while (Dta.available() > 0) {
    byte gpsData = Dta.read();
    //Serial.write(gpsData);  // Imprimir los datos crudos del GPS
    processGPSData(gpsData);  // Procesar los datos NMEA para extraer las coordenadas
  }

  // Control de tiempo para ejecutar cada 5 segundos
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    printCoordinates();  // Imprimir las coordenadas convertidas a grados decimales
  }
}

// Variables globales para almacenar latitud y longitud en grados decimales
double latitude = 0.0;
double longitude = 0.0;

// Función para procesar los datos NMEA del GPS
void processGPSData(byte gpsData) {
  static char nmeaSentence[100];
  static int index = 0;

  // Construimos una cadena con los datos NMEA
  nmeaSentence[index++] = gpsData;
  if (gpsData == '\n') {
    nmeaSentence[index] = '\0';  // Finalizamos la cadena
    index = 0;

    // Procesamos solo las sentencias $GPGLL, $GPRMC o $GPGGA para extraer las coordenadas
    if (strstr(nmeaSentence, "$GPGLL") || strstr(nmeaSentence, "$GPRMC") || strstr(nmeaSentence, "$GPGGA")) {
      extractCoordinates(nmeaSentence);
    }
  }
}

// Función para extraer las coordenadas de las sentencias NMEA
void extractCoordinates(char *nmeaSentence) {
  char *token;
  char latDir, lonDir;
  float rawLat, rawLon;

  // $GPGLL
  if (strstr(nmeaSentence, "$GPGLL")) {
    token = strtok(nmeaSentence, ",");  // Saltamos el identificador $GPGLL
    token = strtok(NULL, ",");  // Latitud cruda
    rawLat = atof(token);
    token = strtok(NULL, ",");  // Dirección de la latitud (N/S)
    latDir = token[0];
    token = strtok(NULL, ",");  // Longitud cruda
    rawLon = atof(token);
    token = strtok(NULL, ",");  // Dirección de la longitud (E/W)
    lonDir = token[0];

    // Convertimos las coordenadas crudas a grados decimales
    latitude = convertToDecimalDegrees(rawLat, latDir);
    longitude = convertToDecimalDegrees(rawLon, lonDir);
  }

  // $GPRMC
  if (strstr(nmeaSentence, "$GPRMC")) {
    token = strtok(nmeaSentence, ",");  // Saltamos el identificador $GPRMC
    token = strtok(NULL, ",");  // Hora UTC
    token = strtok(NULL, ",");  // Estado (A = datos válidos, V = no válidos)
    if (*token == 'A') {
      token = strtok(NULL, ",");  // Latitud cruda
      rawLat = atof(token);
      token = strtok(NULL, ",");  // Dirección de la latitud (N/S)
      latDir = token[0];
      token = strtok(NULL, ",");  // Longitud cruda
      rawLon = atof(token);
      token = strtok(NULL, ",");  // Dirección de la longitud (E/W)
      lonDir = token[0];

      // Convertimos las coordenadas crudas a grados decimales
      latitude = convertToDecimalDegrees(rawLat, latDir);
      longitude = convertToDecimalDegrees(rawLon, lonDir);
    }
  }

  // $GPGGA
  if (strstr(nmeaSentence, "$GPGGA")) {
    token = strtok(nmeaSentence, ",");  // Saltamos el identificador $GPGGA
    token = strtok(NULL, ",");  // Hora UTC
    token = strtok(NULL, ",");  // Latitud cruda
    rawLat = atof(token);
    token = strtok(NULL, ",");  // Dirección de la latitud (N/S)
    latDir = token[0];
    token = strtok(NULL, ",");  // Longitud cruda
    rawLon = atof(token);
    token = strtok(NULL, ",");  // Dirección de la longitud (E/W)
    lonDir = token[0];

    // Convertimos las coordenadas crudas a grados decimales
    latitude = convertToDecimalDegrees(rawLat, latDir);
    longitude = convertToDecimalDegrees(rawLon, lonDir);
  }
}

// Función para convertir coordenadas en grados y minutos a grados decimales
double convertToDecimalDegrees(float rawCoordinate, char direction) {
  int degrees = (int)(rawCoordinate / 100);
  float minutes = rawCoordinate - (degrees * 100);
  double decimalDegrees = degrees + (minutes / 60);

  // Si es latitud sur o longitud oeste, hacemos que el valor sea negativo
  if (direction == 'S' || direction == 'W') {
    decimalDegrees *= -1;
  }

  return decimalDegrees;
}

// Función para imprimir las coordenadas en la consola
void printCoordinates() {
  Serial.print("Latitud (grados decimales): ");
  Serial.println(latitude, 6);  // Imprimir latitud con 6 decimales
  Serial.print("Longitud (grados decimales): ");
  Serial.println(longitude, 6);  // Imprimir longitud con 6 decimales
}*/

//PROBAR GPS
/*#include <SoftwareSerial.h>
#include <TinyGPS++.h>

// Configurar los pines para el módulo GPS
SoftwareSerial gpsSerial(4, 3); // RX (D4), TX (D3)
TinyGPSPlus gps;

void setup() {
  Serial.begin(9600);  // Iniciar comunicación con el monitor serie
  gpsSerial.begin(9600);  // Iniciar comunicación con el módulo GPS
  Serial.println("Esperando señal GPS...");
}

void loop() {
  // Verificar si el módulo GPS está enviando datos
  while (gpsSerial.available() > 0) {
    char c = gpsSerial.read();
    gps.encode(c);  // Procesar los datos entrantes del GPS

    // Imprimir los datos crudos del GPS para verificar
    Serial.write(c);  // Imprimir los datos NMEA crudos (opcional)

    // Verificar si se ha actualizado la ubicación
    if (gps.location.isUpdated()) {
      Serial.print("Latitud: ");
      Serial.println(gps.location.lat(), 6);  // Mostrar la latitud con 6 decimales
      Serial.print("Longitud: ");
      Serial.println(gps.location.lng(), 6);  // Mostrar la longitud con 6 decimales
      Serial.print("Altitud: ");
      Serial.println(gps.altitude.meters());  // Mostrar la altitud en metros
      Serial.print("Número de satélites: ");
      Serial.println(gps.satellites.value());  // Mostrar el número de satélites conectados
    }
  }

  // Si no hay datos válidos de GPS, mostrar un mensaje
  if (gps.charsProcessed() < 10) {
    Serial.println("Esperando datos del GPS...");
  }
}*/


/*#include <SoftwareSerial.h>

// Configuración de los pines para el SIM800L (RX en pin 8, TX en pin 9)
SoftwareSerial sim800l(8, 9);  // RX en pin 8, TX en pin 9

void setup() {
  Serial.begin(9600);      // Comunicación con el Monitor Serie a 9600 baudios
  sim800l.begin(9600);     // Prueba con diferentes velocidades de baudios: 4800, 9600, 19200, 115200, etc.

  Serial.println("Enviando comandos AT para sincronizar velocidad de baudios...");
}

void loop() {

  if (sim800l.available()) {
    Serial.write(sim800l.read());
  }

  if (Serial.available()) {
    sim800l.write(Serial.read());
  }
  

  //delay(2000);  // Espera 2 segundos antes de enviar otro comando AT
}*/

/*#include <SoftwareSerial.h>

// Configuración de los pines para el SIM800L (RX en pin 8, TX en pin 9)
SoftwareSerial sim800l(8, 9);  // RX en pin 8, TX en pin 9

void setup() {
  Serial.begin(9600);      // Comunicación con el Monitor Serie a 9600 baudios
  sim800l.begin(9600);     // Prueba con diferentes velocidades de baudios: 4800, 9600, 19200, 115200, etc.

  Serial.println("Enviando comandos AT para sincronizar velocidad de baudios...");
}

void loop() {
  // Enviar comando "AT" repetidamente para sincronizar la velocidad de baudios
  sim800l.println("AT");   // Envía el comando AT al SIM800L

  // Leer la respuesta del SIM800L
  if (sim800l.available()) {
    String response = sim800l.readString();
    Serial.println("Respuesta del SIM800L: " + response);
  }
  

  delay(2000);  // Espera 2 segundos antes de enviar otro comando AT
}*/

/*#include <SoftwareSerial.h>

// Configuración de SoftwareSerial para SIM800L (RX en 8, TX en 9)
// Create a SoftwareSerial object to communicate with the SIM800L module
SoftwareSerial mySerial(9, 8); // SIM800L Tx & Rx connected to Arduino pins #3 & #2
void setup()
{
  // Initialize serial communication with Arduino and the Arduino IDE (Serial Monitor)
  Serial.begin(9600);
  // Initialize serial communication with Arduino and the SIM800L module
  mySerial.begin(9600);
  Serial.println("Initializing...");
  delay(1000);*/
 /* mySerial.println("AT"); // Handshake test, should return "OK" on success
  updateSerial();
  mySerial.println("AT+GSN"); // Signal quality test, value range is 0-31, 31 is the best
  updateSerial();*/
  /*mySerial.println("AT+CSQ"); // Signal quality test, value range is 0-31, 31 is the best
  updateSerial();
  mySerial.println("AT+CCID"); // Read SIM information to confirm whether the SIM is inserted
  updateSerial();*/
 /* mySerial.println("AT+CREG?"); // Check if it's registered on the network
  updateSerial();
}
void loop()
{
  updateSerial();
}
void updateSerial()
{
  delay(500);
  /*while (Serial.available()) 
  {
    mySerial.write(Serial.read()); // Forward data from Serial to Software Serial Port
  }*/
  /*while (mySerial.available()) 
  {
    Serial.write(mySerial.read()); // Forward data from Software Serial to Serial Port
  }
}*/


/*
String readResponse() {
  String response = "";
  unsigned long startTime = millis();

  // Esperar hasta 5 segundos (5000 ms) para recibir la respuesta
  while (millis() - startTime < 5000) {
    if (sim800l.available()) {
      response += sim800l.readString();  // Leer la respuesta
    }
  }

  return response;
}

void setup() {
  Serial.begin(4800);
  sim800l.begin(4800);
  delay(1000);

  sim800l.println("AT+GSN");  // Enviar comando para obtener IMEI
  String response = readResponse();  // Leer respuesta del módulo
  Serial.println("Respuesta: " + response);
}

void loop() {
  // No es necesario realizar nada en loop para esta prueba
}
*/




/*#include <SoftwareSerial.h>
#include <TinyGPS++.h>

// Configurar los pines para el módulo GPS
SoftwareSerial gpsSerial(4, 3); // RX (D4), TX (D3)
TinyGPSPlus gps;

void setup() {
  Serial.begin(9600);  // Iniciar comunicación con el monitor serie
  gpsSerial.begin(9600);  // Iniciar comunicación con el módulo GPS
}

void loop() {
  // Verificar si el módulo GPS está enviando datos
  while (gpsSerial.available() > 0) {
    char c = gpsSerial.read();
    gps.encode(c);  // Procesar los datos entrantes del GPS

    // Verificar si se ha actualizado la ubicación
    if (gps.location.isUpdated()) {
      Serial.println("Latitud: ");
      Serial.println(gps.location.lat(), 6);  // Mostrar la latitud con 6 decimales en una línea
      Serial.println("Longitud: ");
      Serial.println(gps.location.lng(), 6);  // Mostrar la longitud con 6 decimales en la siguiente línea
    }
  }
}*/

/*const int pulsePin = A0;  // Pin donde está conectado el sensor de pulso
int changeThreshold = 150;  // Umbral para detectar cambios significativos
unsigned long lastBeatTime = 0;  // Tiempo del último latido detectado
unsigned long previousMillis = 0;  // Tiempo anterior para calcular BPM
unsigned long interval = 15000;  // Intervalo de 15 segundos para calcular BPM
int beatCount = 0;  // Contador de latidos
int lastPulseValue = 0;  // Último valor del sensor

void setup() {
  Serial.begin(9600);  // Iniciar comunicación serie
  Serial.println("Iniciando lectura del sensor de pulso...");
}

void loop() {
  // Leer el valor actual del sensor de pulso
  int pulseValue = analogRead(pulsePin);

  // Imprimir los valores del sensor de pulso para verificar la lectura
  /////Serial.print("Valor del sensor: ");
  //////Serial.println(pulseValue);

  // Detectar un latido si la diferencia con el valor anterior es mayor al umbral
  if (abs(pulseValue - lastPulseValue) > changeThreshold) {
    unsigned long currentMillis = millis();

    // Verificar que haya pasado suficiente tiempo desde el último latido (debounce de 600 ms)
    if (currentMillis - lastBeatTime > 600) {  // 600 ms para evitar latidos falsos
      beatCount++;  // Incrementar contador de latidos
      lastBeatTime = currentMillis;  // Actualizar tiempo del último latido detectado
    }
  }

  // Actualizar el último valor de pulso
  lastPulseValue = pulseValue;

  // Calcular BPM cada 15 segundos
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;

    // Calcular las BPM (latidos por minuto)
    int bpm = beatCount * (60 / 15);  // Multiplicar por 60 y dividir por 15 porque contamos latidos en 15 segundos

    // Imprimir solo si el valor de BPM es mayor a 0
    if (bpm > 0) {
      Serial.print("Pulsaciones por minuto (BPM): ");
      Serial.println(bpm);
    }

    // Reiniciar el contador de latidos
    beatCount = 0;
  }

  delay(20);  // Pausa pequeña antes de la próxima lectura
}*/

/*#include <SoftwareSerial.h>

// Configurar los pines para el SIM800L
SoftwareSerial sim800l(8, 9); // RX en pin 8, TX en pin 9

void setup() {
  Serial.begin(9600);      // Iniciar comunicación con el monitor serie a 9600 baudios
  sim800l.begin(9600);     // Configura el SIM800L a 9600 baudios (ajústalo si es necesario)

  Serial.println("Iniciando comunicación con el SIM800L...");
}

#include <SoftwareSerial.h>
#include <TinyGPS++.h>

SoftwareSerial sim800l(8, 9);  // RX, TX del SIM800L
SoftwareSerial gpsSerial(4, 3); // RX (D4), TX (D3) para el GPS
TinyGPSPlus gps;

// Pines y variables del sensor de ritmo cardíaco
const int pulsePin = A0;
int changeThreshold = 150; 
unsigned long lastBeatTime = 0;
unsigned long previousMillis = 0;
unsigned long interval = 15000;  // Intervalo de 15 segundos
int beatCount = 0;  
int lastPulseValue = 0;

// URL del servidor donde se enviarán los datos
const String serverURL = "https://guardian-service.onrender.com/api/paciente/actualizarIoT";

// Función para inicializar la comunicación
void setup() {
  Serial.begin(9600);
  sim800l.begin(9600);  // Inicializar el SIM800L
  gpsSerial.begin(9600);  // Inicializar el GPS
  Serial.println("Iniciando...");
  
  // Espera para asegurarse de que el SIM800L esté listo
  delay(3000);  
  sim800l.println("AT"); // Comando básico para verificar la conexión con el SIM800L
  delay(1000);
  sim800l.println("AT+CMGF=1"); // Configurar el SIM800L en modo texto para enviar datos
  delay(1000);
}

// Función para leer el número de la SIM
String obtenerNumeroSIM() {
  sim800l.println("AT+CNUM");
  delay(1000); 
  String response = "";
  while (sim800l.available()) {
    response += sim800l.readString();
  }
  // Extraer el número de la respuesta. Asegúrate de ajustar la extracción según el formato exacto.
  int index = response.indexOf("+CNUM: ");
  if (index != -1) {
    int start = response.indexOf("\"", index) + 1;
    int end = response.indexOf("\"", start);
    return response.substring(start, end);
  }
  return "Numero no encontrado";
}

// Función para enviar la información al servidor
void enviarDatos(String numeroSIM, float latitud, float longitud, int bpm) {
  sim800l.println("AT+HTTPINIT");  // Iniciar HTTP
  delay(1000);
  sim800l.println("AT+HTTPPARA=\"URL\",\"" + serverURL + "\"");  // Establecer la URL
  delay(1000);
  sim800l.println("AT+HTTPPARA=\"CONTENT\",\"application/json\"");  // Establecer el tipo de contenido
  delay(1000);

  // Crear el cuerpo del POST en formato JSON
  String postData = "{\"SIM\":\"" + numeroSIM + "\", \"Latitud\":" + String(latitud, 6) + ", \"Longitud\":" + String(longitud, 6) + ", \"RitmoCardiaco\":" + String(bpm) + "}";
  sim800l.println("AT+HTTPDATA=" + String(postData.length()) + ",10000");
  delay(1000);
  sim800l.print(postData);
  delay(1000);
  sim800l.println("AT+HTTPACTION=1");  // Enviar la solicitud POST
  delay(1000);
  sim800l.println("AT+HTTPTERM");  // Terminar HTTP
}

// Función principal para leer datos del sensor de pulso y GPS, y enviarlos cada 15 segundos
void loop() {
  String numeroSIM = obtenerNumeroSIM();  // Obtener el número de la SIM

  // Leer el valor actual del sensor de pulso
  int pulseValue = analogRead(pulsePin);

  // Detectar un latido si la diferencia con el valor anterior es mayor al umbral
  if (abs(pulseValue - lastPulseValue) > changeThreshold) {
    unsigned long currentMillis = millis();
    if (currentMillis - lastBeatTime > 600) {  // Evitar latidos falsos
      beatCount++;
      lastBeatTime = currentMillis;
    }
  }
  lastPulseValue = pulseValue;

  // Leer y actualizar las coordenadas del GPS
  while (gpsSerial.available() > 0) {
    char c = gpsSerial.read();
    gps.encode(c);  // Procesar los datos entrantes del GPS
  }

  // Calcular BPM y enviar datos cada 15 segundos
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    
    // Calcular las BPM
    int bpm = beatCount * (60 / 15);  // Multiplicar por 60 y dividir por 15 porque contamos latidos en 15 segundos
    beatCount = 0;

    // Si se han actualizado las coordenadas GPS
    if (gps.location.isUpdated()) {
      float latitud = gps.location.lat();
      float longitud = gps.location.lng();
      enviarDatos(numeroSIM, latitud, longitud, bpm);  // Enviar datos al servidor
    }
  }
  
  delay(20);  // Pausa pequeña antes de la próxima lectura
}
*/


/*#include <SoftwareSerial.h>

// Configuración de pines para el SIM800L
SoftwareSerial sim800l(8, 9); // RX, TX (Arduino -> SIM800L)

// Inicialización de variables
String respuesta;

void setup() {
  // Iniciar comunicación serial con el monitor y el módulo SIM800L
  Serial.begin(9600); // Monitor Serial
  sim800l.begin(4800); // Módulo SIM800L

  // Esperar un momento para que el módulo se inicialice
  delay(1000);
  Serial.println("Iniciando prueba del módulo SIM800L...");
}

void loop() {
  // Verificar si hay datos entrantes desde el SIM800L
  if (sim800l.available()) {
    char c = sim800l.read();
    Serial.write(c); // Imprimir respuesta del SIM800L en el Monitor Serial
  }

  // Verificar si hay datos entrantes desde el Monitor Serial
  if (Serial.available()) {
    char c = Serial.read();
    sim800l.write(c); // Enviar datos del Monitor Serial al SIM800L
  }

  delay(10); // Pequeño retraso para evitar saturar el buffer
}*/


/*#include <SoftwareSerial.h>

// Configuración del puerto serial para el SIM800L en los pines 8 (RX) y 9 (TX)
SoftwareSerial SIM800L(8, 9); // RX en 8, TX en 9

void setup() {
  // Iniciar comunicación serial con el PC y el módulo SIM800L
  Serial.begin(9600);      // Comunicación con el monitor serial (PC)
  SIM800L.begin(9600);      // Comunicación con el SIM800L
  
  // Mensaje de inicio para verificar que todo está listo
  Serial.println("SIM800L listo. Enviar comandos AT...");
}

void loop() {
  // Enviar comandos desde el monitor serial al SIM800L
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n'); // Leer comando del monitor serial
    SIM800L.println(command);  // Enviar al módulo SIM800L
  }

  // Mostrar respuesta del SIM800L en el monitor serial
  if (SIM800L.available()) {
    String response = SIM800L.readString();  // Leer respuesta del módulo
    Serial.println("Respuesta: " + response);  // Imprimir en el monitor serial
  }
}*/





/*#define TINY_GSM_MODEM_SIM800
#include <TinyGsmClient.h>
#include <SoftwareSerial.h>
#include <ArduinoHttpClient.h>

SoftwareSerial sim800Serial(8, 9);  // RX, TX
TinyGsm modem(sim800Serial);
TinyGsmClient client(modem);
HttpClient http(client, "guardian-service-iot.rojasanthony16.workers.dev", 80);  // HTTP

void setup() {
  Serial.begin(9600);
  sim800Serial.begin(9600);

  modem.restart();

  if (modem.gprsConnect("claro.pe", "", "")) {
    Serial.println("Conexión GPRS exitosa.");
    enviarDatos();
  } else {
    Serial.println("Error al conectar GPRS.");
  }
}

void enviarDatos() {
  String path = "/api/paciente/actualizarIoT";  // Ruta al servicio original
  String json = "{\"SIM\":\"5454454539999\",\"Latitud\":-12.057521,\"Longitud\":-77.083144,\"RitmoCardiaco\":25}";

  http.beginRequest();
  http.post(path);
  http.sendHeader("Content-Type", "application/json");
  http.sendHeader("Content-Length", json.length());
  http.beginBody();
  http.print(json);
  http.endRequest();

  int statusCode = http.responseStatusCode();
  Serial.print("Código de respuesta: ");
  Serial.println(statusCode);

  String response = http.responseBody();
  Serial.println("Respuesta: " + response);

  modem.gprsDisconnect();
  Serial.println("Conexión GPRS cerrada.");
}

void loop() {
  // Nada que hacer en el loop
}*/

//OBTENER IMEI PERFECTAMENTE

/*#define TINY_GSM_MODEM_SIM800
#include <TinyGsmClient.h>
#include <SoftwareSerial.h>
#include <ArduinoHttpClient.h>

// Configuración del puerto serial y del módulo SIM800L
SoftwareSerial sim800Serial(8, 9);  // RX en 8, TX en 9
TinyGsm modem(sim800Serial);
TinyGsmClient client(modem);

// Configuración del servidor HTTP
HttpClient http(client, "guardian-service-iot.rojasanthony16.workers.dev", 80);

// APNs de operadores en Perú
const char* apns[][2] = {
  {"claro.pe", ""}, 
  {"movistar.pe", ""}, 
  {"entel.pe", ""}, 
  {"bitel.pe", ""}
};

String imei;  // Almacenar IMEI del módulo

void setup() {
  Serial.begin(9600);
  sim800Serial.begin(9600);

  Serial.println("Inicializando SIM800L...");

  // Reiniciar el módem
  if (!modem.restart()) {
    Serial.println("Error al reiniciar el módem.");
    return;
  }

  Serial.println("Módem listo.");

  // Obtener el IMEI del módulo
  if (!obtenerIMEI()) {
    Serial.println("Error al obtener IMEI.");
    return;
  }
  Serial.println("IMEI obtenido: " + imei);

  // Intentar conectarse con los APNs disponibles
  if (!conectarGPRS()) {
    Serial.println("Error al conectar GPRS.");
    return;
  }

  // Enviar datos al servidor
  enviarDatos();
  Serial.println("Conexión GPRS cerrada.");
}

void loop() {
  // No hacer nada en el loop principal
}

// Función para leer el IMEI del módem
bool obtenerIMEI() {
  imei = modem.getIMEI();
  if (imei != "") {
    return true;
  }
  return false;
}

// Conectar al GPRS utilizando los APNs disponibles
bool conectarGPRS() {
  for (int i = 0; i < 4; i++) {
    Serial.print("Intentando conectar con APN: ");
    Serial.println(apns[i][0]);

    if (modem.gprsConnect(apns[i][0], "", "")) {
      Serial.println("Conexión GPRS exitosa.");
      return true;
    } else {
      Serial.println("Error al conectar con este APN.");
    }
  }
  return false;
}

// Enviar datos al servidor
void enviarDatos() {
  String path = "/api/paciente/actualizarIoT";
  String json = "{\"SIM\":\"" + imei + "\",\"Latitud\":-12.057521,\"Longitud\":-77.083144,\"RitmoCardiaco\":45}";

  http.beginRequest();
  http.post(path);
  http.sendHeader("Content-Type", "application/json");
  http.sendHeader("Content-Length", json.length());
  http.beginBody();
  http.print(json);
  http.endRequest();

  int statusCode = http.responseStatusCode();
  Serial.print("Código de respuesta: ");
  Serial.println(statusCode);

  String response = http.responseBody();
  Serial.println("Respuesta: " + response);

  modem.gprsDisconnect();  // Desconectar GPRS
}*/



/*#define TINY_GSM_MODEM_SIM800
#include <TinyGsmClient.h>
#include <SoftwareSerial.h>
#include <ArduinoHttpClient.h>
#include <TinyGPS++.h>

// Configuración del puerto serial y del módem
SoftwareSerial sim800Serial(8, 9);  // RX, TX del SIM800L
SoftwareSerial gpsSerial(4, 3);     // RX (D4), TX (D3) para el GPS
TinyGsm modem(sim800Serial);
TinyGsmClient client(modem);
HttpClient http(client, "guardian-service-iot.rojasanthony16.workers.dev", 80);  // HTTP
TinyGPSPlus gps;

// Pines y variables del sensor de ritmo cardíaco
const int pulsePin = A0;
int changeThreshold = 150; 
unsigned long lastBeatTime = 0;
unsigned long previousMillis = 0;
unsigned long interval = 15000;  // Intervalo de 15 segundos
int beatCount = 0;  
int lastPulseValue = 0;

// Variable global para almacenar el IMEI
String imei;

// Función para inicializar la comunicación
void setup() {
  Serial.begin(9600);
  sim800Serial.begin(9600);  // Inicializar el SIM800L
  //gpsSerial.begin(9600);     // Inicializar el GPS
  Serial.println("Iniciando...");

  // Reiniciar el módem
  modem.restart();

  Serial.println("Esperando la inicialización del módem...");
  delay(10000);  // Esperar 5 segundos antes de enviar comandos
  
  // Obtener el IMEI del módulo
  if (!obtenerIMEI()) {
    Serial.println("Error al obtener IMEI.");
    return;
  }

  // Intentar conectar GPRS
  if (modem.gprsConnect("claro.pe", "", "")) {
    Serial.println("Conexión GPRS exitosa.");
  } else {
    Serial.println("Error al conectar GPRS.");
  }
}

// Función para obtener el IMEI del módulo
bool obtenerIMEI() {
  modem.sendAT("+GSN");  // Comando AT para obtener IMEI
  if (modem.waitResponse(1000) == 1) {
    imei = modem.stream.readStringUntil('\n');  // Leer IMEI del buffer
    imei.trim();  // Eliminar espacios en blanco al inicio y al final
    Serial.print("IMEI del módulo: ");
    Serial.println(imei);
    return true;
  }
  return false;
}

// Función para enviar la información al servidor
void enviarDatos(float latitud, float longitud, int bpm) {
  String path = "/api/paciente/actualizarIoT";  // Ruta al servicio
  String json = "{\"SIM\":\"" + imei + "\",\"Latitud\":" + String(latitud, 6) + 
                ",\"Longitud\":" + String(longitud, 6) + ",\"RitmoCardiaco\":" + String(bpm) + "}";

  http.beginRequest();
  http.post(path);
  http.sendHeader("Content-Type", "application/json");
  http.sendHeader("Content-Length", json.length());
  http.beginBody();
  http.print(json);
  http.endRequest();

  int statusCode = http.responseStatusCode();
  Serial.print("Código de respuesta: ");
  Serial.println(statusCode);

  String response = http.responseBody();
  Serial.println("Respuesta: " + response);
}

// Función principal para leer datos del sensor y GPS, y enviarlos cada 15 segundos
void loop() {
  // Leer el valor actual del sensor de pulso
  int pulseValue = analogRead(pulsePin);

  // Detectar un latido si la diferencia con el valor anterior es mayor al umbral
  if (abs(pulseValue - lastPulseValue) > changeThreshold) {
    unsigned long currentMillis = millis();
    if (currentMillis - lastBeatTime > 600) {  // Evitar latidos falsos
      beatCount++;
      lastBeatTime = currentMillis;
    }
  }
  lastPulseValue = pulseValue;

  // Leer y actualizar las coordenadas del GPS
  while (gpsSerial.available() > 0) {
    char c = gpsSerial.read();
    gps.encode(c);  // Procesar los datos del GPS
  }

  // Enviar datos cada 15 segundos
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;

    // Calcular BPM
    int bpm = beatCount * (60 / 15);  // Latidos por minuto
    beatCount = 0;

    // Si se actualizaron las coordenadas GPS
    if (gps.location.isUpdated()) {
      float latitud = gps.location.lat();
      float longitud = gps.location.lng();
      enviarDatos(latitud, longitud, bpm);  // Enviar datos al servidor
    }
  }

  delay(20);  // Pausa antes de la próxima lectura
  
}*/


/*#define TINY_GSM_MODEM_SIM800
#include <TinyGsmClient.h>
#include <SoftwareSerial.h>
#include <ArduinoHttpClient.h>
#include <TinyGPS++.h>

// Configuración de los puertos seriales para SIM800L y GPS
SoftwareSerial sim800Serial(8, 9);  // RX, TX del SIM800L
SoftwareSerial gpsSerial(4, 3);     // RX, TX del GPS
TinyGsm modem(sim800Serial);
TinyGsmClient client(modem);
HttpClient http(client, "guardian-service-iot.rojasanthony16.workers.dev", 80);  // HTTP
TinyGPSPlus gps;

// Variables globales
String imei;  // Almacenar IMEI del módem

void setup() {
  Serial.begin(9600);  // Iniciar comunicación con PC
  iniciarModem();      // Inicializar el SIM800L
  iniciarGPS();        // Inicializar el GPS
}

// Inicializar el módem SIM800L
void iniciarModem() {
  sim800Serial.begin(9600);  // Inicializar SIM800L
  delay(1000);

  Serial.println("Verificando comunicación con SIM800L...");
  if (enviarComandoAT("AT", "OK")) {
    Serial.println("Módem conectado.");
    if (modem.restart()) {
      Serial.println("Módem reiniciado correctamente.");
      obtenerIMEI();
      conectarGPRS();
    } else {
      Serial.println("Error al reiniciar el módem.");
    }
  } else {
    Serial.println("No hay comunicación con el módem.");
  }
}

// Inicializar GPS
void iniciarGPS() {
  sim800Serial.end();  // Apagar SIM800L temporalmente
  gpsSerial.begin(9600);  // Inicializar GPS
}

// Enviar comandos AT y verificar respuesta
bool enviarComandoAT(const String& comando, const String& respuestaEsperada) {
  sim800Serial.println(comando);
  delay(2000);

  String respuesta = leerRespuesta();
  return respuesta.indexOf(respuestaEsperada) != -1;
}

// Leer respuesta del módem SIM800L
String leerRespuesta() {
  String respuesta = "";
  long timeout = millis() + 5000;

  while (millis() < timeout) {
    if (sim800Serial.available()) {
      char c = sim800Serial.read();
      respuesta += c;
    }
  }

  Serial.println("Respuesta: " + respuesta);
  return respuesta;
}

// Obtener IMEI del módem
void obtenerIMEI() {
  imei = modem.getIMEI();
  if (imei != "") {
    Serial.println("IMEI: " + imei);
  } else {
    Serial.println("Error al obtener IMEI.");
  }
}

// Conectar a GPRS
void conectarGPRS() {
  if (modem.gprsConnect("claro.pe", "", "")) {
    Serial.println("Conexión GPRS exitosa.");
  } else {
    Serial.println("Error al conectar GPRS.");
  }
}

// Enviar datos al servidor
void enviarDatos(float latitud, float longitud, int bpm) {
  String path = "/api/paciente/actualizarIoT";
  String json = "{\"SIM\":\"" + imei + "\",\"Latitud\":" + String(latitud, 6) +
                ",\"Longitud\":" + String(longitud, 6) + ",\"RitmoCardiaco\":" + String(bpm) + "}";

  http.beginRequest();
  http.post(path);
  http.sendHeader("Content-Type", "application/json");
  http.sendHeader("Content-Length", json.length());
  http.beginBody();
  http.print(json);
  http.endRequest();

  int statusCode = http.responseStatusCode();
  Serial.print("Código de respuesta: ");
  Serial.println(statusCode);

  String response = http.responseBody();
  Serial.println("Respuesta: " + response);

  modem.gprsDisconnect();
  Serial.println("Conexión GPRS cerrada.");
}

void loop() {
  // Leer coordenadas del GPS
  while (gpsSerial.available() > 0) {
    char c = gpsSerial.read();
    gps.encode(c);
  }

  // Enviar datos si hay nuevas coordenadas
  if (gps.location.isUpdated()) {
    float latitud = gps.location.lat();
    float longitud = gps.location.lng();
    enviarDatos(latitud, longitud, 72);  // BPM simulado
  } else {
    Serial.println("Esperando señal GPS...");
  }

  delay(15000);  // Pausa antes del siguiente envío
}*/


/*#define TINY_GSM_MODEM_SIM800
#include <TinyGsmClient.h>
#include <SoftwareSerial.h>
#include <ArduinoHttpClient.h>

// Configuración del puerto serial y del módulo SIM800L
SoftwareSerial sim800Serial(2, 3);  // RX en 8, TX en 9
TinyGsm modem(sim800Serial);
TinyGsmClient client(modem);

// Configuración del servidor HTTP
HttpClient http(client, "guardian-service-iot.rojasanthony16.workers.dev", 80);

// APNs de operadores en Perú
const char* apns[][2] = {
  {"claro.pe", ""}, 
  {"movistar.pe", ""}, 
  {"entel.pe", ""}, 
  {"bitel.pe", ""}
};

String imei;  // Almacenar IMEI del módulo

void setup() {
   Serial.begin(9600);
  sim800Serial.begin(9600);

  Serial.println("Inicializando SIM800L...");

  // Reiniciar el módem
  if (!modem.restart()) {
    Serial.println("Error al reiniciar el módem.");
    return;
  }

  Serial.println("Módem listo.");

  // Obtener el IMEI del módulo
  if (!obtenerIMEI()) {
    Serial.println("Error al obtener IMEI.");
    return;
  }
  Serial.println("IMEI obtenido: " + imei);

  // Intentar conectarse con los APNs disponibles
  if (!conectarGPRS()) {
    Serial.println("Error al conectar GPRS.");
    return;
  }

  // Enviar datos al servidor
  enviarDatos();
  Serial.println("Conexión GPRS cerrada.");
}

void loop() {
  // No hacer nada en el loop principal
}

// Función para leer el IMEI del módem
bool obtenerIMEI() {
  imei = modem.getIMEI();
  if (imei != "") {
    return true;
  }
  return false;
}

// Conectar al GPRS utilizando los APNs disponibles
bool conectarGPRS() {
  for (int i = 0; i < 4; i++) {
    Serial.print("Intentando conectar con APN: ");
    Serial.println(apns[i][0]);

    if (modem.gprsConnect(apns[i][0], "", "")) {
      Serial.println("Conexión GPRS exitosa.");
      return true;
    } else {
      Serial.println("Error al conectar con este APN.");
    }
  }
  return false;
}

// Enviar datos al servidor
void enviarDatos() {
  String path = "/api/paciente/actualizarIoT";
  String json = "{\"SIM\":\"" + imei + "\",\"Latitud\":-12.057521,\"Longitud\":-77.083144,\"RitmoCardiaco\":45}";

  http.beginRequest();
  http.post(path);
  http.sendHeader("Content-Type", "application/json");
  http.sendHeader("Content-Length", json.length());
  http.beginBody();
  http.print(json);
  http.endRequest();

  int statusCode = http.responseStatusCode();
  Serial.print("Código de respuesta: ");
  Serial.println(statusCode);

  String response = http.responseBody();
  Serial.println("Respuesta: " + response);

  modem.gprsDisconnect();  // Desconectar GPRS
}*/



/*#include <SoftwareSerial.h>
#include <TinyGPS++.h>

// Configurar los pines para el módulo GPS
SoftwareSerial gpsSerial(8, 9); // RX (D4), TX (D3)
TinyGPSPlus gps;

void setup() {
  Serial.begin(9600);  // Iniciar comunicación con el monitor serie
  gpsSerial.begin(9600);  // Iniciar comunicación con el módulo GPS
  Serial.println("Esperando señal GPS...");
}

void loop() {
  // Verificar si el módulo GPS está enviando datos
  while (gpsSerial.available() > 0) {
    char c = gpsSerial.read();
    gps.encode(c);  // Procesar los datos entrantes del GPS

    // Imprimir los datos crudos del GPS para verificar
    //Serial.write(c);  // Imprimir los datos NMEA crudos (opcional)

    // Verificar si se ha actualizado la ubicación
    
    if (gps.location.isUpdated()) {
      delay(3000);
      Serial.print("Latitud: ");
      Serial.println(gps.location.lat(), 6);  // Mostrar la latitud con 6 decimales
      Serial.print("Longitud: ");
      Serial.println(gps.location.lng(), 6);  // Mostrar la longitud con 6 decimales
    }
  }

  // Si no hay datos válidos de GPS, mostrar un mensaje
  if (gps.charsProcessed() < 10) {
    delay(3000);
    Serial.println("Esperando datos del GPS...");
  }
}*/




/*#define TINY_GSM_MODEM_SIM800
#include <SoftwareSerial.h>
#include <TinyGPS++.h>
#include <TinyGsmClient.h>
#include <ArduinoHttpClient.h>

// Configuración de los puertos seriales virtuales
SoftwareSerial sim800Serial(2, 3);  // RX, TX del SIM800L
SoftwareSerial gpsSerial(8, 9);     // RX, TX del GPS

TinyGPSPlus gps;
TinyGsm modem(sim800Serial);

// Variables de datos
String imei;
float latitud = 0.0, longitud = 0.0;
bool internet = false;
unsigned long previousMillis = 0;

TinyGsmClient client(modem);
HttpClient http(client, "guardian-service-iot.rojasanthony16.workers.dev", 80);

// Configuración del APN
const char* apns[][2] = {{"claro.pe", ""}, {"movistar.pe", ""}, {"entel.pe", ""}, {"bitel.pe", ""}};
const char* apn = "claro.pe";

void setup() {
  Serial.begin(9600);  // Monitor Serial

  sim800Serial.begin(9600);  // Inicializar SIM800L
  gpsSerial.begin(9600);     // Inicializar GPS

  Serial.println("Inicializando SIM800L...");
  sim800Serial.listen();  // Hacer que el SIM800L escuche
  if (!reiniciarModem()) {
    Serial.println("Error al reiniciar el módem.");
    return;
  }

  sim800Serial.listen();
  delay(200);
  if (!obtenerIMEI()) {
    Serial.println("Error al obtener IMEI.");
    return;
  }
  Serial.println("IMEI obtenido: " + imei);

  //if (!conectarGPRS()) {
  //  Serial.println("Error al conectar GPRS.");
  //  return;
  //}
}

void loop() {
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= 15000) {  // Cada 15 segundos
    previousMillis = currentMillis;

    obtenerCoordenadasGPS();

    if (internet && gps.location.isValid()) {
      enviarDatos();
    }
    //if(gps.location.isValid()) {
     // enviarDatos();
    //}
  }
}

// Reiniciar el SIM800L
bool reiniciarModem() {
  sim800Serial.listen();  // Activar la escucha del SIM800L
  sim800Serial.println("AT+CFUN=1,1");  // Reiniciar
  delay(10000);  // Esperar 10 segundos

  sim800Serial.println("AT");
  delay(1000);
  if (sim800Serial.find("OK")) {
    Serial.println("SIM800L reiniciado correctamente.");
    return true;
  }
  return false;
}

// Obtener coordenadas del GPS
void obtenerCoordenadasGPS() {
  Serial.println("Obteniendo datos del GPS...");
  gpsSerial.listen();  // Activar la escucha del GPS

  unsigned long startTime = millis();  // Tiempo inicial
  bool ubicacionObtenida = false;

  // Esperar hasta 5 segundos para recibir una ubicación válida
  while (millis() - startTime < 5000) {
    while (gpsSerial.available() > 0) {
      char c = gpsSerial.read();
      gps.encode(c);  // Procesar el carácter NMEA
      Serial.write(c);  // Mostrar los datos NMEA crudos (opcional)

      // Verificar si la ubicación se actualizó
      if (gps.location.isUpdated()) {
        latitud = gps.location.lat();
        longitud = gps.location.lng();
        Serial.print("\nLatitud: ");
        Serial.println(latitud, 6);
        Serial.print("Longitud: ");
        Serial.println(longitud, 6);
        ubicacionObtenida = true;
        break;  // Salir del ciclo si se obtuvo la ubicación
      }
    }

    if (ubicacionObtenida) break;  // Salir si ya se tiene la ubicación
  }

  if (!ubicacionObtenida) {
    Serial.println("No se pudo obtener la ubicación.");
  }
}

// Obtener el IMEI del SIM800L
bool obtenerIMEI() {
  sim800Serial.listen();
  sim800Serial.println("AT+GSN");  // Enviar comando para obtener el IMEI

  String respuesta = "";
  unsigned long startTime = millis();

  // Esperar 5 segundos para recibir la respuesta completa
  while (millis() - startTime < 5000) {
    if (sim800Serial.available()) {
      char c = sim800Serial.read();
      respuesta += c;
    }
  }

  Serial.println("Respuesta completa: " + respuesta);

  // Buscar el IMEI dentro de la respuesta
  int imeiStart = respuesta.indexOf("8635");  // El IMEI comienza con 8635
  if (imeiStart != -1) {
    imei = respuesta.substring(imeiStart, imeiStart + 15);  // Extraer los 15 dígitos
    Serial.println("IMEI obtenido: " + imei);
    return true;
  }

  Serial.println("No se encontró el IMEI en la respuesta.");
  return false;
}

// Conectar al GPRS
bool conectarGPRS() {
  sim800Serial.listen();  // Activar la escucha del SIM800L
 
  for (int i = 0; i < 4; i++) {
    Serial.print("Intentando conectar con APN: ");
    Serial.println(apns[i][0]);

    if (modem.gprsConnect(apns[i][0], "", "")) {
      Serial.println("Conexión GPRS exitosa.");
      internet = true;
      return true;
    } else {
      Serial.println("Error al conectar con este APN.");
    }
  }
  return false;
}

void reiniciarGPRS() {
  modem.sendAT("AT+SAPBR=0,1");  // Cerrar cualquier perfil GPRS abierto
  modem.waitResponse(3000);
  modem.sendAT("AT+SAPBR=1,1");  // Volver a abrir el perfil GPRS
  modem.waitResponse(3000);
}

// Enviar datos al servidor
void enviarDatos() {
  sim800Serial.listen();  // Aseguramos que el SIM800L esté activo
  // Verificación de conexión GPRS
  bool gprs = false;
  for (int i = 0; i < 4; i++) {
    if (!gprs) {
      Serial.print("Intentando conectar con APN: ");
      Serial.println(apns[i][0]);

    reiniciarGPRS();
      delay(200);
      if (modem.gprsConnect(apns[i][0], "", "")) {
        Serial.println("Conexión GPRS exitosa.");
        gprs = true;
        delay(200);  // Breve pausa tras la conexión
      } else {
        Serial.println("Error al conectar con este APN.");
      }
    }
  }
  //reiniciarGPRS();
  //Serial.print("Intentando conectar con APN: ");
  //if (modem.gprsConnect(apn, "", "")) {
     //   Serial.println("Conexión GPRS exitosa.");
    //    gprs = true;
   //     delay(200);  // Breve pausa tras la conexión
  //    } else {
 //       Serial.println("Error al conectar con este APN.");
//      }

delay(200);
  if (gprs) {
    delay(200);
    // Validamos que los datos sean correctos
    if (imei == "" || latitud == 0.0 || longitud == 0.0) {
      modem.gprsDisconnect();
      delay(200);
      if(imei==""){
          if (!obtenerIMEI()) {
          Serial.println("Error al obtener IMEI.");
          return;
      }
        Serial.println("IMEI obtenido otra vez: " + imei);
      }
      delay(500);
      Serial.println("imei: ");
      Serial.println(imei);
      delay(200);
      Serial.println("latitud: ");
      Serial.println(latitud);
      delay(200);
      Serial.println("longitud: ");
      Serial.println(longitud);
      delay(200);
      Serial.println("Error: Datos incompletos o incorrectos.");
      return;
    }

    // Reservamos memoria para el JSON (optimización de memoria)
    String json;
    json.reserve(128);  // Reservamos 128 bytes para evitar problemas de realocación

    // Construimos el JSON de forma eficiente
    json = "{";
    json += "\"SIM\":\"";
    json += imei;
    json += "\",\"Latitud\":";
    json += String(latitud, 6);
    json += ",\"Longitud\":";
    json += String(longitud, 6);
    json += ",\"RitmoCardiaco\":50";
    json += "}";

    Serial.println("Payload JSON: " + json);  // Verificamos la cadena JSON

    // Iniciamos la solicitud HTTP POST
    http.beginRequest();
    http.post("/");
    http.sendHeader("Content-Type", "application/json");
    http.sendHeader("Content-Length", json.length());
    http.beginBody();
    http.print(json);  // Enviamos el JSON al servidor
    http.endRequest();

    // Leemos el código de respuesta del servidor
    int statusCode = http.responseStatusCode();
    Serial.print("Código de respuesta: ");
    Serial.println(statusCode);

    // Leemos la respuesta del servidor
    String response = http.responseBody();
    Serial.println("Respuesta: " + response);

    // Desconectamos GPRS
    modem.gprsDisconnect();
  }
}*/





/*#include <HardwareSerial.h>

// Configuración de UART1 para SIM800L (GPIO4 y GPIO5)
HardwareSerial SIM800L(1);  // UART1 del ESP32

void setup() {
  Serial.begin(115200);  // Monitor serial del ESP32
  SIM800L.begin(9600, SERIAL_8N1, 5, 4);  // UART1 (RX=GPIO5, TX=GPIO4)

  Serial.println("Inicializando SIM800L...");
  delay(1000);

  // Enviar un comando AT para verificar la conexión
  enviarComandoAT("AT");

  // Enviar un comando para revisar el estado del módulo
  enviarComandoAT("AT+CSQ");  // Verificar intensidad de señal
  enviarComandoAT("AT+CCID");  // Obtener la identificación de la SIM
  enviarComandoAT("AT+CREG?");  // Verificar registro en la red
}

void loop() {
  // Revisar si hay respuestas desde el SIM800L y mostrar en monitor serial
  while (SIM800L.available()) {
    String respuesta = SIM800L.readString();
    Serial.println("Respuesta: " + respuesta);
  }
  delay(1000);  // Espera para evitar saturación del puerto serial
}

void enviarComandoAT(const char* comando) {
  Serial.print("Enviando: ");
  Serial.println(comando);

  SIM800L.println(comando);  // Enviar comando AT al SIM800L
  delay(1000);  // Esperar por la respuesta del SIM800L
}*/

/*#include <HardwareSerial.h>
#include <TinyGPS++.h>

// Configuración del UART2 para el GPS
HardwareSerial SerialGPS(2); // UART2
TinyGPSPlus gps;

void setup() {
  Serial.begin(115200);  // Inicializar el monitor serie
  SerialGPS.begin(9600, SERIAL_8N1, 16, 17);  // RX2 = GPIO16, TX2 = GPIO17

  Serial.println("Iniciando GPS...");
}

void loop() {
  while (SerialGPS.available() > 0) {
    char c = SerialGPS.read();
    gps.encode(c);  // Procesar los datos NMEA

    // Mostrar datos NMEA crudos en el monitor serie (opcional)
    Serial.write(c);

    // Verificar si hay datos de ubicación disponibles
    if (gps.location.isUpdated()) {
      Serial.println("\n--- Datos de GPS recibidos ---");
      Serial.print("Latitud: ");
      Serial.println(gps.location.lat(), 6);
      Serial.print("Longitud: ");
      Serial.println(gps.location.lng(), 6);
      Serial.print("Altitud: ");
      Serial.println(gps.altitude.meters(), 2);
      Serial.print("Satélites: ");
      Serial.println(gps.satellites.value());
      Serial.print("Precisión HDOP: ");
      Serial.println(gps.hdop.hdop());
      Serial.println("----------------------------");
    }
  }
}*/

/*#define TINY_GSM_MODEM_SIM800
#include <HardwareSerial.h>
#include <TinyGPS++.h>
#include <TinyGsmClient.h>
#include <ArduinoHttpClient.h>

// Configuración de UART para SIM800L y GPS
HardwareSerial SerialSim800(1);  // UART1 para SIM800L
HardwareSerial SerialGPS(2);     // UART2 para GPS

TinyGPSPlus gps;
TinyGsm modem(SerialSim800);
TinyGsmClient client(modem);
HttpClient http(client, "guardian-service-iot.rojasanthony16.workers.dev", 80);

// APNs para operadores en Perú
const char* apns[][2] = {
  {"claro.pe", ""},
  {"movistar.pe", ""},
  {"entel.pe", ""},
  {"bitel.pe", ""}
};

String imei;
float latitud = 0.0, longitud = 0.0;
bool gprsConectado = false;
unsigned long previousMillis = 0;
const long intervaloEnvio = 15000;  // 15 segundos

void setup() {
  Serial.begin(115200);  // Monitor Serial
  SerialSim800.begin(9600, SERIAL_8N1, 5, 4);  // UART1 para SIM800L
  SerialGPS.begin(9600, SERIAL_8N1, 16, 17);   // UART2 para GPS

  Serial.println("Inicializando SIM800L...");
  if (!reiniciarModem()) {
    Serial.println("Error al reiniciar el módem.");
    return;
  }

  if (!obtenerIMEI()) {
    Serial.println("Error al obtener IMEI.");
    return;
  }
  Serial.println("IMEI obtenido: " + imei);
}

void loop() {
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= intervaloEnvio) {
    previousMillis = currentMillis;

    obtenerCoordenadasGPS();

    if (gprsConectado && gps.location.isValid()) {
      enviarDatos();
    } else {
      conectarGPRS();  // Intentar reconectar si no está conectado
    }
  }
}

// Reiniciar el SIM800L
bool reiniciarModem() {
  SerialSim800.println("AT+CFUN=1,1");  // Reiniciar módulo
  delay(10000);  // Esperar 10 segundos

  SerialSim800.println("AT");
  delay(1000);
  if (SerialSim800.find("OK")) {
    Serial.println("SIM800L reiniciado correctamente.");
    return true;
  }
  return false;
}

// Obtener el IMEI del SIM800L
bool obtenerIMEI() {
  SerialSim800.println("AT+GSN");  // Comando para IMEI
  String respuesta = leerRespuesta(5000);

  int imeiStart = respuesta.indexOf("8635");  // Buscar IMEI que empieza con 8635
  if (imeiStart != -1) {
    imei = respuesta.substring(imeiStart, imeiStart + 15);  // Extraer los 15 dígitos
    return true;
  }
  return false;
}

// Conectar al GPRS
void conectarGPRS() {
  for (int i = 0; i < 4; i++) {
    Serial.print("Intentando conectar con APN: ");
    Serial.println(apns[i][0]);

    if (modem.gprsConnect(apns[i][0], "", "")) {
      Serial.println("Conexión GPRS exitosa.");
      gprsConectado = true;
      return;
    }
    Serial.println("Error al conectar con este APN.");
  }
  gprsConectado = false;
}

// Obtener las coordenadas del GPS
void obtenerCoordenadasGPS() {
  Serial.println("Obteniendo datos del GPS...");
  while (SerialGPS.available() > 0) {
    char c = SerialGPS.read();
    gps.encode(c);
    Serial.write(c);  // Mostrar datos NMEA crudos

    if (gps.location.isUpdated()) {
      latitud = gps.location.lat();
      longitud = gps.location.lng();
      Serial.println("\n--- Coordenadas obtenidas ---");
      Serial.print("Latitud: ");
      Serial.println(latitud, 6);
      Serial.print("Longitud: ");
      Serial.println(longitud, 6);
    }
  }
}

// Enviar datos al servidor
void enviarDatos() {
  String json = "{\"SIM\":\"" + imei + "\",\"Latitud\":" + String(latitud, 6) +
                ",\"Longitud\":" + String(longitud, 6) + ",\"RitmoCardiaco\":50}";

  Serial.println("Enviando datos al servidor...");
  http.beginRequest();
  http.post("/");
  http.sendHeader("Content-Type", "application/json");
  http.sendHeader("Content-Length", json.length());
  http.beginBody();
  http.print(json);
  http.endRequest();

  int statusCode = http.responseStatusCode();
  Serial.print("Código de respuesta: ");
  Serial.println(statusCode);

  if (statusCode != 200) {
    Serial.println("Error en la petición HTTP.");
  } else {
    String response = http.responseBody();
    Serial.println("Respuesta del servidor: " + response);
  }
}

// Leer la respuesta del SIM800L
String leerRespuesta(unsigned long timeout) {
  String respuesta = "";
  unsigned long startTime = millis();
  while (millis() - startTime < timeout) {
    while (SerialSim800.available()) {
      char c = SerialSim800.read();
      respuesta += c;
    }
  }
  return respuesta;
}*/







/*#include <SoftwareSerial.h>

SoftwareSerial sim7600(2, 3); // RX, TX

void setup() {
  Serial.begin(9600);
  sim7600.begin(9600); // Prueba con 57600 baudios para el SIM7600SA

  delay(5000); // Espera 5 segundos para que el módulo esté completamente listo

  // Desactivar el eco para evitar caracteres duplicados
  sim7600.println("ATE0");
  delay(1000);

  Serial.println("Solicitando IMEI del SIM7600SA...");

  // Enviar comando AT+GSN para obtener el IMEI
  sim7600.println("AT+GSN");
  delay(2000);

  // Leer y mostrar la respuesta
  Serial.println("IMEI del SIM7600SA:");
  String imei = ""; // Variable para almacenar el IMEI
  while (sim7600.available()) {
    char c = sim7600.read();
    imei += c; // Agregar cada carácter a la variable IMEI
  }
  
  // Limpiar la respuesta para mostrar solo el IMEI
  imei.replace("OK", ""); // Elimina el OK si está en la respuesta
  imei.trim(); // Elimina cualquier espacio en blanco o caracteres extraños

  Serial.println(imei); // Mostrar solo el IMEI
}

void loop() {
  // No es necesario hacer nada en el loop para esta prueba
}*/

/*#include <HardwareSerial.h>

HardwareSerial SerialSim800(1); // UART1 para SIM800L

void setup() {
  // Inicializa la comunicación serial con el monitor y el módulo SIM
  Serial.begin(115200);               // Monitor serial (USB)
  SerialSim800.begin(9600, SERIAL_8N1, 5, 4); // UART1 para SIM800L (pines TX=5, RX=4)

  Serial.println("Listo para enviar comandos AT. Escribe un comando y presiona Enter.");
}

void loop() {
  // Enviar comandos del monitor serial al módulo SIM
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');  // Lee el comando ingresado en el monitor serial
    SerialSim800.println(command);                  // Envía el comando al módulo SIM
    Serial.println("Enviado: " + command);          // Muestra el comando enviado en el monitor serial
  }

  // Leer y mostrar la respuesta del módulo SIM en el monitor serial
  if (SerialSim800.available()) {
    String response = SerialSim800.readString();    // Lee la respuesta del módulo SIM
    Serial.println("Respuesta: " + response);       // Muestra la respuesta en el monitor serial
  }
}*/

/*#define TINY_GSM_MODEM_SIM800
#include <HardwareSerial.h>
#include <TinyGPS++.h>
#include <TinyGsmClient.h>
#include <ArduinoHttpClient.h>

// Configuración de UART para SIM800L y GPS
HardwareSerial SerialSim800(1);  // UART1 para SIM800L
HardwareSerial SerialGPS(2);     // UART2 para GPS

TinyGPSPlus gps;
TinyGsm modem(SerialSim800);
TinyGsmClient client(modem);
HttpClient http(client, "guardian-service-iot.rojasanthony16.workers.dev", 80);

String imei;
float latitud = 0.0, longitud = 0.0;
bool gprsConectado = false;
unsigned long previousMillis = 0;
const long intervaloEnvio = 15000;  // 15 segundos
const char* apnSeleccionado = "";  // Variable global para almacenar el APN seleccionado

void setup() {
  Serial.begin(115200);  // Monitor Serial
  SerialSim800.begin(9600, SERIAL_8N1, 5, 4);  // UART1 para SIM800L
  SerialGPS.begin(9600, SERIAL_8N1, 16, 17);   // UART2 para GPS

  Serial.println("Inicializando SIM800L...");
  if (!reiniciarModem()) {
    Serial.println("Error al reiniciar el módem.");
    return;
  }

  if (!obtenerIMEI()) {
    Serial.println("Error al obtener IMEI.");
    return;
  }
  Serial.println("IMEI obtenido: " + imei);

  // Configurar el APN correcto según el operador detectado
  configurarAPN();
}

void loop() {
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= intervaloEnvio) {
    previousMillis = currentMillis;

    obtenerCoordenadasGPS();

    if (gprsConectado && gps.location.isValid()) {
      enviarDatos();
    } else {
      conectarGPRS(apnSeleccionado, "", "");  // Intentar reconectar usando el APN almacenado
    }
  }
}

// Reiniciar el SIM800L
bool reiniciarModem() {
  SerialSim800.println("AT+CFUN=1,1");  // Reiniciar módulo
  delay(10000);  // Esperar 10 segundos

  SerialSim800.println("AT");
  delay(1000);
  if (SerialSim800.find("OK")) {
    Serial.println("SIM800L reiniciado correctamente.");
    return true;
  }
  return false;
}

// Obtener el IMEI del SIM800L
bool obtenerIMEI() {
  SerialSim800.println("AT+GSN");  // Comando para IMEI
  String respuesta = leerRespuesta(5000);

  int imeiStart = respuesta.indexOf("8635");  // Buscar IMEI que empieza con 8635
  if (imeiStart != -1) {
    imei = respuesta.substring(imeiStart, imeiStart + 15);  // Extraer los 15 dígitos
    return true;
  }
  return false;
}

// Detectar el operador y configurar el APN adecuado
void configurarAPN() {
  SerialSim800.println("AT+COPS?");
  delay(1000);
  String respuesta = leerRespuesta(5000);

  // Mostrar la respuesta completa en el monitor serial para diagnóstico
  Serial.println("Respuesta de AT+COPS?: " + respuesta);

  if (respuesta.indexOf("71606") != -1) { // Movistar Perú
    Serial.println("Detectado operador: Movistar Perú");
    apnSeleccionado = "movistar.pe";
  } else if (respuesta.indexOf("71610") != -1) { // Claro Perú
    Serial.println("Detectado operador: Claro Perú");
    apnSeleccionado = "claro.pe";
  } else if (respuesta.indexOf("71617") != -1) { // Entel Perú
    Serial.println("Detectado operador: Entel Perú");
    apnSeleccionado = "entel.pe";
  } else if (respuesta.indexOf("71607") != -1) { // Bitel Perú
    Serial.println("Detectado operador: Bitel Perú");
    apnSeleccionado = "bitel.pe";
  } else {
    Serial.println("Operador desconocido. Usando APN por defecto.");
    apnSeleccionado = "internet"; // APN genérico si no se detecta el operador
  }

  // Conectar al GPRS usando el APN seleccionado
  conectarGPRS(apnSeleccionado, "", "");
}

// Conectar al GPRS con un APN específico
void conectarGPRS(const char* apn, const char* user, const char* pass) {
  Serial.print("Intentando conectar con APN: ");
  Serial.println(apn);

  if (modem.gprsConnect(apn, user, pass)) {
    Serial.println("Conexión GPRS exitosa.");
    gprsConectado = true;
  } else {
    Serial.println("Error al conectar con este APN.");
    gprsConectado = false;
  }
}

// Obtener las coordenadas del GPS
void obtenerCoordenadasGPS() {
  Serial.println("Obteniendo datos del GPS...");
  while (SerialGPS.available() > 0) {
    char c = SerialGPS.read();
    gps.encode(c);
    Serial.write(c);  // Mostrar datos NMEA crudos

    if (gps.location.isUpdated()) {
      latitud = gps.location.lat();
      longitud = gps.location.lng();
      Serial.println("\n--- Coordenadas obtenidas ---");
      Serial.print("Latitud: ");
      Serial.println(latitud, 6);
      Serial.print("Longitud: ");
      Serial.println(longitud, 6);
    }
  }
}

// Enviar datos al servidor
void enviarDatos() {
  String json = "{\"SIM\":\"" + imei + "\",\"Latitud\":" + String(latitud, 6) +
                ",\"Longitud\":" + String(longitud, 6) + ",\"RitmoCardiaco\":50}";

  Serial.println("Enviando datos al servidor...");
  http.beginRequest();
  http.post("/");
  http.sendHeader("Content-Type", "application/json");
  http.sendHeader("Content-Length", json.length());
  http.beginBody();
  http.print(json);
  http.endRequest();

  int statusCode = http.responseStatusCode();
  Serial.print("Código de respuesta: ");
  Serial.println(statusCode);

  if (statusCode != 200) {
    Serial.println("Error en la petición HTTP.");
  } else {
    String response = http.responseBody();
    Serial.println("Respuesta del servidor: " + response);
  }
}

// Leer la respuesta del SIM800L
String leerRespuesta(unsigned long timeout) {
  String respuesta = "";
  unsigned long startTime = millis();
  while (millis() - startTime < timeout) {
    while (SerialSim800.available()) {
      char c = SerialSim800.read();
      respuesta += c;
    }
  }
  return respuesta;
}*/



/*const int pulsePin = 34;  // Pin analógico en el ESP32 donde está conectado el sensor de pulso
int changeThreshold = 150;  // Umbral para detectar cambios significativos
unsigned long lastBeatTime = 0;  // Tiempo del último latido detectado
unsigned long previousMillis = 0;  // Tiempo anterior para calcular BPM
unsigned long interval = 15000;  // Intervalo de 15 segundos para calcular BPM
int beatCount = 0;  // Contador de latidos
int lastPulseValue = 0;  // Último valor del sensor

void setup() {
  Serial.begin(115200);  // Iniciar comunicación serie a una velocidad más alta para el ESP32
  Serial.println("Iniciando lectura del sensor de pulso...");
}

void loop() {
  // Leer el valor actual del sensor de pulso
  int pulseValue = analogRead(pulsePin);

  // Imprimir los valores del sensor de pulso para verificar la lectura
  //Serial.print("Valor del sensor: ");
  //Serial.println(pulseValue);

  // Detectar un latido si la diferencia con el valor anterior es mayor al umbral
  if (abs(pulseValue - lastPulseValue) > changeThreshold) {
    unsigned long currentMillis = millis();

    // Verificar que haya pasado suficiente tiempo desde el último latido (debounce de 600 ms)
    if (currentMillis - lastBeatTime > 600) {  // 600 ms para evitar latidos falsos
      beatCount++;  // Incrementar contador de latidos
      lastBeatTime = currentMillis;  // Actualizar tiempo del último latido detectado
    }
  }

  // Actualizar el último valor de pulso
  lastPulseValue = pulseValue;

  // Calcular BPM cada 15 segundos
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;

    // Calcular las BPM (latidos por minuto)
    int bpm = beatCount * (60 / 15);  // Multiplicar por 60 y dividir por 15 porque contamos latidos en 15 segundos

    // Imprimir solo si el valor de BPM es mayor a 0
    if (bpm > 0) {
      Serial.print("Pulsaciones por minuto (BPM): ");
      Serial.println(bpm);
    }

    // Reiniciar el contador de latidos
    beatCount = 0;
  }

  delay(20);  // Pausa pequeña antes de la próxima lectura
}*/



/*#define TINY_GSM_MODEM_SIM800
#include <HardwareSerial.h>
#include <TinyGPS++.h>
#include <TinyGsmClient.h>
#include <ArduinoHttpClient.h>

// Configuración de UART para SIM800L y GPS
HardwareSerial SerialSim800(1);  // UART1 para SIM800L
HardwareSerial SerialGPS(2);     // UART2 para GPS

TinyGPSPlus gps;
TinyGsm modem(SerialSim800);
TinyGsmClient client(modem);
HttpClient http(client, "guardian-service-iot.rojasanthony16.workers.dev", 80);

// Variables para el ritmo cardíaco
const int pulsePin = 34;  // Pin analógico en el ESP32 para el sensor de pulso
int changeThreshold = 150;
unsigned long lastBeatTime = 0;
unsigned long previousMillisHeart = 0;
int beatCount = 0;
int lastPulseValue = 0;
int bpm = 0;

// Variables para el GPS y la conexión
String imei;
float latitud = 0.0, longitud = 0.0;
bool gprsConectado = false;
unsigned long previousMillis = 0;
const long intervaloEnvio = 15000;  // 15 segundos
const char* apnSeleccionado = "";  // Variable global para almacenar el APN seleccionado

void setup() {
  Serial.begin(115200);  // Monitor Serial
  SerialSim800.begin(9600, SERIAL_8N1, 5, 4);  // UART1 para SIM800L
  SerialGPS.begin(9600, SERIAL_8N1, 16, 17);   // UART2 para GPS

  Serial.println("Inicializando SIM800L...");
  if (!reiniciarModem()) {
    Serial.println("Error al reiniciar el módem.");
    return;
  }

  if (!obtenerIMEI()) {
    Serial.println("Error al obtener IMEI.");
    return;
  }
  Serial.println("IMEI obtenido: " + imei);

  // Configurar el APN correcto según el operador detectado
  configurarAPN();
}

void loop() {
  unsigned long currentMillis = millis();
  
  // Monitoreo del ritmo cardíaco
  monitorRitmoCardiaco();

  // Verificación y envío de datos cada 15 segundos
  if (currentMillis - previousMillis >= intervaloEnvio) {
    previousMillis = currentMillis;

    obtenerCoordenadasGPS();

    if (gprsConectado && gps.location.isValid()) {
      enviarDatos();
    } else {
      conectarGPRS(apnSeleccionado, "", "");  // Intentar reconectar usando el APN almacenado
    }
  }
}

// Función para monitorear el ritmo cardíaco y calcular BPM
void monitorRitmoCardiaco() {
  int pulseValue = analogRead(pulsePin);

  // Detección de latido
  if (abs(pulseValue - lastPulseValue) > changeThreshold) {
    unsigned long currentMillisHeart = millis();

    if (currentMillisHeart - lastBeatTime > 600) {  // Evitar latidos falsos (600 ms)
      beatCount++;
      lastBeatTime = currentMillisHeart;
    }
  }

  lastPulseValue = pulseValue;

  // Calcular BPM cada 15 segundos
  if (millis() - previousMillisHeart >= intervaloEnvio) {
    previousMillisHeart = millis();
    bpm = beatCount * (60 / 15);  // Calcular BPM basado en los últimos 15 segundos
    beatCount = 0;
    
    Serial.print("Pulsaciones por minuto (BPM): ");
    Serial.println(bpm);
  }
}

// Reiniciar el SIM800L
bool reiniciarModem() {
  SerialSim800.println("AT+CFUN=1,1");  // Reiniciar módulo
  delay(10000);  // Esperar 10 segundos

  SerialSim800.println("AT");
  delay(1000);
  if (SerialSim800.find("OK")) {
    Serial.println("SIM800L reiniciado correctamente.");
    return true;
  }
  return false;
}

// Obtener el IMEI del SIM800L
bool obtenerIMEI() {
  SerialSim800.println("AT+GSN");  // Comando para IMEI
  String respuesta = leerRespuesta(5000);

  int imeiStart = respuesta.indexOf("8635");  // Buscar IMEI que empieza con 8635
  if (imeiStart != -1) {
    imei = respuesta.substring(imeiStart, imeiStart + 15);  // Extraer los 15 dígitos
    return true;
  }
  return false;
}

// Detectar el operador y configurar el APN adecuado
void configurarAPN() {
  SerialSim800.println("AT+COPS?");
  delay(1000);
  String respuesta = leerRespuesta(5000);

  Serial.println("Respuesta de AT+COPS?: " + respuesta);

  if (respuesta.indexOf("71606") != -1) { // Movistar Perú
    Serial.println("Detectado operador: Movistar Perú");
    apnSeleccionado = "movistar.pe";
  } else if (respuesta.indexOf("71610") != -1) { // Claro Perú
    Serial.println("Detectado operador: Claro Perú");
    apnSeleccionado = "claro.pe";
  } else if (respuesta.indexOf("71617") != -1) { // Entel Perú
    Serial.println("Detectado operador: Entel Perú");
    apnSeleccionado = "entel.pe";
  } else if (respuesta.indexOf("71607") != -1) { // Bitel Perú
    Serial.println("Detectado operador: Bitel Perú");
    apnSeleccionado = "bitel.pe";
  } else {
    Serial.println("Operador desconocido. Usando APN por defecto.");
    apnSeleccionado = "internet";
  }

  conectarGPRS(apnSeleccionado, "", "");
}

// Conectar al GPRS con un APN específico
void conectarGPRS(const char* apn, const char* user, const char* pass) {
  Serial.print("Intentando conectar con APN: ");
  Serial.println(apn);

  if (modem.gprsConnect(apn, user, pass)) {
    Serial.println("Conexión GPRS exitosa.");
    gprsConectado = true;
  } else {
    Serial.println("Error al conectar con este APN.");
    gprsConectado = false;
  }
}

// Obtener las coordenadas del GPS
void obtenerCoordenadasGPS() {
  Serial.println("Obteniendo datos del GPS...");
  while (SerialGPS.available() > 0) {
    char c = SerialGPS.read();
    gps.encode(c);
    //Serial.write(c);

    if (gps.location.isUpdated()) {
      latitud = gps.location.lat();
      longitud = gps.location.lng();
      Serial.println("\n--- Coordenadas obtenidas ---");
      Serial.print("Latitud: ");
      Serial.println(latitud, 6);
      Serial.print("Longitud: ");
      Serial.println(longitud, 6);
    }
  }
}

// Enviar datos al servidor
void enviarDatos() {
  String json = "{\"SIM\":\"" + imei + "\",\"Latitud\":" + String(latitud, 6) +
                ",\"Longitud\":" + String(longitud, 6) + ",\"RitmoCardiaco\":" + String(bpm) + "}";

  Serial.println("Enviando datos al servidor...");
  http.beginRequest();
  http.post("/");
  http.sendHeader("Content-Type", "application/json");
  http.sendHeader("Content-Length", json.length());
  http.beginBody();
  http.print(json);
  http.endRequest();

  int statusCode = http.responseStatusCode();
  Serial.print("Código de respuesta: ");
  Serial.println(statusCode);

  if (statusCode != 200 || statusCode == -3) {
    Serial.println("Error en la petición HTTP. Intentando reconectar...");
    conectarGPRS(apnSeleccionado, "", "");
  } else {
    String response = http.responseBody();
    Serial.println("Respuesta del servidor: " + response);
  }
}

// Leer la respuesta del SIM800L
String leerRespuesta(unsigned long timeout) {
  String respuesta = "";
  unsigned long startTime = millis();
  while (millis() - startTime < timeout) {
    while (SerialSim800.available()) {
      char c = SerialSim800.read();
      respuesta += c;
    }
  }
  return respuesta;
}*/
/*#include <HardwareSerial.h>

HardwareSerial SerialSIM7600(2);  // Configura UART2 para el SIM7600SA

void setup() {
  // Inicializa el monitor serial para depuración
  Serial.begin(115200);
  delay(1000);
  Serial.println("Iniciando prueba del SIM7600SA...");

  // Inicializa la UART2 para el SIM7600SA en los pines RX2=16, TX2=17
  SerialSIM7600.begin(115200, SERIAL_8N1, 16, 17);
  delay(1000);

  // Envía un comando AT básico al SIM7600SA
  enviarComandoAT("AT");
}

void loop() {
  // Lee y muestra la respuesta del SIM7600SA en el monitor serial
  if (SerialSIM7600.available()) {
    String respuesta = SerialSIM7600.readStringUntil('\n');
    Serial.println("Respuesta del SIM7600SA: " + respuesta);
  }

  // Envía comandos AT adicionales desde el monitor serial
  if (Serial.available()) {
    String comando = Serial.readStringUntil('\n');
    enviarComandoAT(comando);
  }

  delay(100);  // Pequeña pausa para evitar saturar el bucle
}

// Función para enviar un comando AT al SIM7600SA
void enviarComandoAT(String comando) {
  SerialSIM7600.println(comando);
  Serial.println("Enviado: " + comando);
}*/

/*#define TINY_GSM_MODEM_SIM7600
#include <HardwareSerial.h>
#include <TinyGPS++.h>
#include <TinyGsmClient.h>
#include <ArduinoHttpClient.h>

// Configuración de UART para SIM7600SA y GPS
HardwareSerial SerialSIM7600(2);  // UART2 para el SIM7600SA (RX2=16, TX2=17)

TinyGPSPlus gps;
TinyGsm modem(SerialSIM7600);
TinyGsmClient client(modem);
HttpClient http(client, "guardian-service-iot.rojasanthony16.workers.dev", 80);

// Variables para el ritmo cardíaco
const int pulsePin = 34;  // Pin analógico en el ESP32 para el sensor de pulso
int changeThreshold = 150;
unsigned long lastBeatTime = 0;
unsigned long previousMillisHeart = 0;
int beatCount = 0;
int lastPulseValue = 0;
int bpm = 0;

// Variables para el GPS y la conexión
String imei;
float latitud = 0.0, longitud = 0.0;
bool gprsConectado = false;
bool gpsActivo = false;
unsigned long previousMillis = 0;
const long intervaloEnvio = 15000;  // 15 segundos

// APN de Claro
const char* apn = "claro.pe";

void setup() {
  Serial.begin(115200);                   // Monitor Serial
  SerialSIM7600.begin(115200, SERIAL_8N1, 16, 17);  // UART2 para SIM7600SA

  Serial.println("Inicializando SIM7600SA...");
  if (!reiniciarModem()) {
    Serial.println("Error al reiniciar el módem.");
    return;
  }

  if (!obtenerIMEI()) {
    Serial.println("Error al obtener IMEI.");
    return;
  }
  Serial.println("IMEI obtenido: " + imei);

  // Activar el GPS
  activarGPS();

  // Conectar usando el APN de Claro
  configurarGPRS();
}

void loop() {
  unsigned long currentMillis = millis();
  
  // Monitoreo del ritmo cardíaco
  monitorRitmoCardiaco();

  // Verificación y envío de datos cada 15 segundos
  if (currentMillis - previousMillis >= intervaloEnvio) {
    previousMillis = currentMillis;

    if (obtenerCoordenadasGPS()) {  // Intentar obtener coordenadas GPS
      if (gprsConectado) {
        enviarDatos();
      } else {
        configurarGPRS();  // Reintentar conectar si no está conectado
      }
    } else {
      Serial.println("Esperando señal GPS válida...");
    }
  }
}

// Función para monitorear el ritmo cardíaco y calcular BPM
void monitorRitmoCardiaco() {
  int pulseValue = analogRead(pulsePin);

  // Detección de latido
  if (abs(pulseValue - lastPulseValue) > changeThreshold) {
    unsigned long currentMillisHeart = millis();

    if (currentMillisHeart - lastBeatTime > 600) {  // Evitar latidos falsos (600 ms)
      beatCount++;
      lastBeatTime = currentMillisHeart;
    }
  }

  lastPulseValue = pulseValue;

  // Calcular BPM cada 15 segundos
  if (millis() - previousMillisHeart >= intervaloEnvio) {
    previousMillisHeart = millis();
    bpm = beatCount * (60 / 15);  // Calcular BPM basado en los últimos 15 segundos
    beatCount = 0;
    
    Serial.print("Pulsaciones por minuto (BPM): ");
    Serial.println(bpm);
  }
}

// Activar el GPS
void activarGPS() {
  enviarComandoAT("AT+CGPS=1,1");  // Enciende el GPS en modo autónomo
  delay(2000);
  gpsActivo = true;
  Serial.println("GPS activado.");
}

// Reiniciar el SIM7600SA
bool reiniciarModem() {
  enviarComandoAT("AT");  // Comando básico para verificar si está en línea
  delay(3000);
  if (leerRespuesta(5000).indexOf("OK") != -1) {
    Serial.println("SIM7600SA está en línea.");
    return true;
  }
  Serial.println("No se pudo verificar el estado del módem.");
  return false;
}

// Obtener el IMEI del SIM7600SA de forma general
bool obtenerIMEI() {
  enviarComandoAT("AT+GSN");  // Comando para IMEI
  String respuesta = leerRespuesta(5000);
  delay(3000);

  // Extraer solo los dígitos del IMEI (15 caracteres numéricos)
  respuesta.trim();  // Eliminar espacios en blanco
  if (respuesta.length() >= 15) {
    imei = respuesta;
    return true;
  }
  return false;
}

// Configurar GPRS usando el APN de Claro
void configurarGPRS() {
  Serial.print("Intentando conectar con APN: ");
  Serial.println(apn);

  if (conectarGPRS(apn, "", "")) {
    Serial.println("Conexión GPRS exitosa.");
    gprsConectado = true;
  } else {
    Serial.println("Error al conectar con el APN de Claro.");
    gprsConectado = false;
  }
}

// Conectar al GPRS con un APN específico
bool conectarGPRS(const char* apn, const char* user, const char* pass) {
  return modem.gprsConnect(apn, user, pass);
}

// Obtener las coordenadas del GPS usando AT+CGPSINF
bool obtenerCoordenadasGPS() {
  if (!gpsActivo) {
    activarGPS();  // Activar el GPS si no está activo
  }

  enviarComandoAT("AT+CGPSINF=0");  // Solicitar información GPS
  String respuesta = leerRespuesta(5000);

  // Parsear respuesta para verificar si hay datos válidos
  if (respuesta.indexOf("+CGPSINF:") != -1) {
    // Extraer latitud y longitud
    int latStart = respuesta.indexOf(",", respuesta.indexOf("+CGPSINF:") + 9);
    int longStart = respuesta.indexOf(",", latStart + 1);

    latitud = respuesta.substring(latStart + 1, longStart).toFloat();
    longitud = respuesta.substring(longStart + 1, respuesta.indexOf(",", longStart + 1)).toFloat();

    Serial.print("Latitud: ");
    Serial.println(latitud, 6);
    Serial.print("Longitud: ");
    Serial.println(longitud, 6);

    return true;  // Señal GPS válida
  }

  return false;  // Señal GPS aún no válida
}

// Enviar datos al servidor
void enviarDatos() {
  String json = "{\"SIM\":\"" + imei + "\",\"Latitud\":" + String(latitud, 6) +
                ",\"Longitud\":" + String(longitud, 6) + ",\"RitmoCardiaco\":" + String(bpm) + "}";

  Serial.println("Enviando datos al servidor...");
  http.beginRequest();
  http.post("/");
  http.sendHeader("Content-Type", "application/json");
  http.sendHeader("Content-Length", json.length());
  http.beginBody();
  http.print(json);
  http.endRequest();

  int statusCode = http.responseStatusCode();
  Serial.print("Código de respuesta: ");
  Serial.println(statusCode);

  if (statusCode != 200) {
    Serial.println("Error en la petición HTTP.");
    gprsConectado = false;  // Intentar reconectar si falla la petición
  } else {
    String response = http.responseBody();
    Serial.println("Respuesta del servidor: " + response);
  }
}

// Leer la respuesta del SIM7600SA, ignorando líneas vacías
String leerRespuesta(unsigned long timeout) {
  String respuesta = "";
  unsigned long startTime = millis();
  while (millis() - startTime < timeout) {
    while (SerialSIM7600.available()) {
      String linea = SerialSIM7600.readStringUntil('\n');
      linea.trim();  // Eliminar espacios en blanco
      if (linea.length() > 0) {
        respuesta += linea + "\n";
      }
    }
  }
  return respuesta;
}

// Función para enviar un comando AT
void enviarComandoAT(const char* comando) {
  SerialSIM7600.println(comando);
  Serial.print("Enviado: ");
  Serial.println(comando);
}*/

/*#include <HardwareSerial.h>

HardwareSerial SerialSIM7600(2);  // UART2 para el SIM7600SA (RX2=16, TX2=17)

const int pulsePin = 34;  // Pin analógico en el ESP32 para el sensor de pulso
int changeThreshold = 150;
unsigned long lastBeatTime = 0;
unsigned long previousMillisHeart = 0;
int beatCount = 0;
int lastPulseValue = 0;
int bpm = 0;

unsigned long previousMillis = 0;
const long intervaloEnvio = 15000;  // 15 segundos

void setup() {
  Serial.begin(115200);  // Monitor serial para depuración
  SerialSIM7600.begin(115200, SERIAL_8N1, 16, 17);  // UART2 para SIM7600SA

  Serial.println("Prueba de GPS en SIM7600SA...");

  // Reiniciar el GPS y comenzar en modo autónomo
  SerialSIM7600.println("AT+CGPS=1,1");
  delay(2000);
  mostrarRespuesta();
}

void loop() {
  unsigned long currentMillis = millis();
  
  // Monitorear el ritmo cardíaco
  monitorRitmoCardiaco();

  // Verificar y enviar datos cada 15 segundos
  if (currentMillis - previousMillis >= intervaloEnvio) {
    previousMillis = currentMillis;

    obtenerCoordenadasGPS();  // Obtener y procesar las coordenadas GPS
    enviarDatos();            // Enviar datos al servidor
  }
}

// Función para monitorear el ritmo cardíaco y calcular BPM
void monitorRitmoCardiaco() {
  int pulseValue = analogRead(pulsePin);

  if (abs(pulseValue - lastPulseValue) > changeThreshold) {
    unsigned long currentMillisHeart = millis();
    if (currentMillisHeart - lastBeatTime > 600) {
      beatCount++;
      lastBeatTime = currentMillisHeart;
    }
  }
  
  lastPulseValue = pulseValue;

  if (millis() - previousMillisHeart >= intervaloEnvio) {
    previousMillisHeart = millis();
    bpm = beatCount * (60 / 15);
    beatCount = 0;
    Serial.print("Pulsaciones por minuto (BPM): ");
    Serial.println(bpm);
  }
}

// Función para obtener y procesar las coordenadas GPS
void obtenerCoordenadasGPS() {
  SerialSIM7600.println("AT+CGPSINFO");
  delay(2000);
  
  while (SerialSIM7600.available()) {
    String respuesta = SerialSIM7600.readStringUntil('\n');
    respuesta.trim();
    
    if (respuesta.startsWith("+CGPSINFO:")) {
      String gpsData = respuesta.substring(10);  // Extraer los datos después de "+CGPSINFO:"
      procesarCoordenadas(gpsData);
    }
  }
}

// Función para procesar y convertir las coordenadas GPS a formato decimal
void procesarCoordenadas(String gpsData) {
  int commaIndex = gpsData.indexOf(',');
  
  if (commaIndex != -1 && gpsData.length() > 20) {
    String latStr = gpsData.substring(0, commaIndex);  // Latitud cruda
    char latHem = gpsData.charAt(commaIndex + 1);      // Hemisferio de latitud
    String lonStr = gpsData.substring(commaIndex + 10, gpsData.indexOf(',', commaIndex + 10));  // Longitud cruda
    char lonHem = gpsData.charAt(gpsData.indexOf(',', commaIndex + 10) + 1);  // Hemisferio de longitud

    float latDecimal = convertirDecimal(latStr, latHem == 'S' ? -1 : 1);
    float lonDecimal = convertirDecimal(lonStr, lonHem == 'W' ? -1 : 1);

    Serial.print("Latitud (decimal): ");
    Serial.println(latDecimal, 6);
    Serial.print("Longitud (decimal): ");
    Serial.println(lonDecimal, 6);
  } else {
    Serial.println("Error al obtener datos GPS.");
  }
}

// Función para convertir grados/minutos a decimal
float convertirDecimal(String coordStr, int signo) {
  float grados = coordStr.substring(0, coordStr.length() - 7).toFloat();
  float minutos = coordStr.substring(coordStr.length() - 7).toFloat();
  return signo * (grados + minutos / 60.0);
}

// Función para enviar los datos al servidor
void enviarDatos() {
  // Aquí iría la lógica para enviar los datos, reemplaza con el código correspondiente a tu servicio.
  Serial.println("Enviando datos al servidor...");
}

// Función para mostrar la respuesta del módulo
void mostrarRespuesta() {
  while (SerialSIM7600.available()) {
    String respuesta = SerialSIM7600.readStringUntil('\n');
    respuesta.trim();
    Serial.println("Respuesta del SIM7600SA: " + respuesta);
  }
}*/

/*#include <HardwareSerial.h>

HardwareSerial SerialSIM7600(2);  // UART2 para el SIM7600SA (RX2=16, TX2=17)

float Lat = 0.0;
float Log = 0.0;

void setup() {
  Serial.begin(115200);  // Monitor serial para depuración
  SerialSIM7600.begin(115200, SERIAL_8N1, 16, 17);  // UART2 para SIM7600SA

  Serial.println("Inicializando SIM7600SA...");

  // Reiniciar y activar el GPS
  reiniciarGPS();

  // Obtener el IMEI del módulo
  obtenerIMEI();
}

void loop() {
  // Obtener coordenadas GPS cada 15 segundos
  obtenerCoordenadasGPS();
  delay(15000);  
}

// Función para reiniciar y activar el GPS en el SIM7600SA
void reiniciarGPS() {
  Serial.println("Reiniciando GPS...");

  // Desactivar GPS (para reiniciar)
  SerialSIM7600.println("AT+CGPS=0");
  delay(2000);
  leerRespuesta();

  // Activar GPS en modo autónomo
  SerialSIM7600.println("AT+CGPS=1,1");
  delay(3000);
  String estadoGPS = leerRespuesta();

  if (estadoGPS.indexOf("OK") != -1) {
    Serial.println("GPS activado correctamente.");
  } else {
    Serial.println("Error al activar el GPS.");
  }
}

// Función para obtener el IMEI del módulo SIM7600SA
void obtenerIMEI() {
  SerialSIM7600.println("AT+GSN");  // Comando para obtener el IMEI
  delay(1000);
  String respuesta = leerRespuesta();
  
  if (respuesta.length() > 0) {
    Serial.println("IMEI del SIM7600SA: " + respuesta);
  } else {
    Serial.println("No se pudo obtener el IMEI.");
  }
}

// Función para obtener y convertir las coordenadas GPS
void obtenerCoordenadasGPS() {
  Serial.println("Obteniendo datos del GPS...");

  for (int intentos = 0; intentos < 3; intentos++) {
    SerialSIM7600.println("AT+CGPSINFO");
    delay(2000);
    String respuesta = leerRespuesta();

    if (respuesta.indexOf("+CGPSINFO:") != -1 && respuesta.indexOf(",,,,,,,") == -1) {
      // Extraer y convertir latitud y longitud si los datos son válidos
      if (procesarDatosGPS(respuesta)) {
        Serial.print("Latitud: ");
        Serial.println(Lat, 6);
        Serial.print("Longitud: ");
        Serial.println(Log, 6);
        return;
      }
    } else {
      Serial.println("No se pudieron obtener las coordenadas GPS. Reintentando...");
      delay(5000);  
    }
  }

  Serial.println("No se pudieron obtener las coordenadas GPS después de varios intentos.");
}

// Función para procesar la cadena de datos GPS y convertir las coordenadas
bool procesarDatosGPS(String datosGPS) {
  int indexLat = datosGPS.indexOf(':') + 1;
  int indexNS = datosGPS.indexOf(',', indexLat + 1);
  int indexLong = datosGPS.indexOf(',', indexNS + 1);
  int indexEW = datosGPS.indexOf(',', indexLong + 1);

  String latStr = datosGPS.substring(indexLat + 1, indexNS);
  String ns = datosGPS.substring(indexNS + 1, indexNS + 2);
  String lonStr = datosGPS.substring(indexLong + 1, indexEW);
  String ew = datosGPS.substring(indexEW + 1, indexEW + 2);

  if (latStr.length() == 0 || lonStr.length() == 0) {
    Serial.println("Datos GPS inválidos.");
    return false;
  }

  // Convertir latitud
  Lat = convertirCoordenada(latStr, ns);

  // Convertir longitud
  Log = convertirCoordenada(lonStr, ew);

  return true;
}

// Función para convertir una coordenada de grados/minutos a decimal
float convertirCoordenada(String coord, String direccion) {
  int grados = (direccion == "N" || direccion == "S") ? coord.substring(0, 2).toInt() : coord.substring(0, 3).toInt();
  float minutos = coord.substring((direccion == "N" || direccion == "S") ? 2 : 3).toFloat();
  float decimal = grados + (minutos / 60.0);

  if (direccion == "S" || direccion == "W") {
    decimal = -decimal;
  }
  return decimal;
}

// Función para leer la respuesta del módulo
String leerRespuesta() {
  String respuesta = "";
  unsigned long startTime = millis();
  while (millis() - startTime < 5000) {  // Esperar hasta 5 segundos
    while (SerialSIM7600.available()) {
      char c = SerialSIM7600.read();
      respuesta += c;
    }
  }
  respuesta.trim();
  Serial.println("Respuesta del SIM7600SA: " + respuesta);
  return respuesta;
}*/


/*#include <HardwareSerial.h>

HardwareSerial SerialSIM7600(2);  // UART2 para el SIM7600SA (RX2=16, TX2=17)

const int pulsePin = 34;  // Pin analógico en el ESP32 para el sensor de pulso
unsigned long previousMillis = 0;
const long interval = 15000;  // Intervalo de 15 segundos

float latitud = 0.0;
float longitud = 0.0;
int bpm = 0;
int beatCount = 0;
int lastPulseValue = 0;
unsigned long lastBeatTime = 0;

void setup() {
  Serial.begin(115200);  // Monitor serial para depuración
  SerialSIM7600.begin(115200, SERIAL_8N1, 16, 17);  // UART2 para SIM7600SA

  Serial.println("Inicializando el módulo SIM7600SA y GPS...");

  // Activar el GPS en el módulo (reintento en caso de error)
  activarGPS();
}

void loop() {
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;

    // Obtener y mostrar ritmo cardíaco
    calcularRitmoCardiaco();
    Serial.print("Pulsaciones por minuto (BPM): ");
    Serial.println(bpm);

    // Obtener y mostrar coordenadas GPS
    obtenerCoordenadasGPS();
  }
}

// Función para activar el GPS
void activarGPS() {
  SerialSIM7600.println("AT+CGPS=1,1");  // Activar el GPS en modo autónomo
  delay(3000);
  mostrarRespuesta();

  // Comprobar si el GPS está realmente activado
  SerialSIM7600.println("AT+CGPS?");
  delay(2000);
  String estadoGPS = leerRespuesta();

  if (estadoGPS.indexOf("+CGPS: 1") == -1) {
    Serial.println("Error al activar el GPS. Reintentando...");
    delay(5000);  // Esperar antes de reintentar
    activarGPS();  // Reintentar la activación
  } else {
    Serial.println("GPS activado correctamente.");
  }
}

// Función para obtener y convertir las coordenadas GPS
void obtenerCoordenadasGPS() {
  Serial.println("Obteniendo datos del GPS...");

  for (int intentos = 0; intentos < 3; intentos++) {
    SerialSIM7600.println("AT+CGPSINFO");
    delay(2000);
    String respuesta = leerRespuesta();

    if (respuesta.indexOf("+CGPSINFO:") != -1 && respuesta.indexOf(",,,,,,,") == -1) {
      // Extraer y convertir latitud y longitud si los datos son válidos
      if (procesarDatosGPS(respuesta)) {
        Serial.print("Latitud: ");
        Serial.println(latitud, 6);
        Serial.print("Longitud: ");
        Serial.println(longitud, 6);
        return;
      }
    } else {
      Serial.println("No se pudieron obtener las coordenadas GPS. Reintentando...");
      delay(5000);  
    }
  }

  Serial.println("No se pudieron obtener las coordenadas GPS después de varios intentos.");
}

// Función para procesar la cadena de datos GPS y convertir las coordenadas
bool procesarDatosGPS(String datosGPS) {
  int indexLat = datosGPS.indexOf(':') + 1;
  int indexNS = datosGPS.indexOf(',', indexLat + 1);
  int indexLong = datosGPS.indexOf(',', indexNS + 1);
  int indexEW = datosGPS.indexOf(',', indexLong + 1);

  String latStr = datosGPS.substring(indexLat + 1, indexNS);
  String ns = datosGPS.substring(indexNS + 1, indexNS + 2);
  String lonStr = datosGPS.substring(indexLong + 1, indexEW);
  String ew = datosGPS.substring(indexEW + 1, indexEW + 2);

  if (latStr.length() == 0 || lonStr.length() == 0) {
    Serial.println("Datos GPS inválidos.");
    return false;
  }

  // Convertir latitud
  latitud = convertirCoordenada(latStr, ns);

  // Convertir longitud
  longitud = convertirCoordenada(lonStr, ew);

  return true;
}

float convertirCoordenada(String coord, String direccion) {
  int grados = (direccion == "N" || direccion == "S") ? coord.substring(0, 2).toInt() : coord.substring(0, 3).toInt();
  float minutos = coord.substring((direccion == "N" || direccion == "S") ? 2 : 3).toFloat();
  float decimal = grados + (minutos / 60.0);

  if (direccion == "S" || direccion == "W") {
    decimal = -decimal;
  }
  return decimal;
}

// Función para calcular el ritmo cardíaco
void calcularRitmoCardiaco() {
  int pulseValue = analogRead(pulsePin);

  // Detección de latido
  if (abs(pulseValue - lastPulseValue) > 150) {  // Umbral de cambio
    unsigned long currentMillis = millis();

    if (currentMillis - lastBeatTime > 600) {  // Evitar latidos falsos (600 ms)
      beatCount++;
      lastBeatTime = currentMillis;
    }
  }

  lastPulseValue = pulseValue;

  // Calcular BPM cada intervalo de 15 segundos
  bpm = beatCount * 4;  // Calcular BPM basado en 15 segundos
  beatCount = 0;
}

// Función para leer la respuesta del módulo
String leerRespuesta() {
  String respuesta = "";
  unsigned long startTime = millis();
  while (millis() - startTime < 5000) {  // Esperar hasta 5 segundos
    while (SerialSIM7600.available()) {
      char c = SerialSIM7600.read();
      respuesta += c;
    }
  }
  respuesta.trim();
  Serial.println("Respuesta del SIM7600SA: " + respuesta);
  return respuesta;
}

// Función para mostrar la respuesta del módulo (para depuración)
void mostrarRespuesta() {
  while (SerialSIM7600.available()) {
    String respuesta = SerialSIM7600.readStringUntil('\n');
    respuesta.trim();
    Serial.println("Respuesta del SIM7600SA: " + respuesta);
  }
}*/

/*const int pulsePin = 34;  // Pin donde está conectado el sensor de pulso en el ESP32
int changeThreshold = 150;  // Umbral para detectar cambios significativos en la señal
unsigned long lastBeatTime = 0;  // Tiempo del último latido detectado
unsigned long previousMillis = 0;  // Tiempo anterior para calcular BPM
unsigned long interval = 15000;  // Intervalo de 15 segundos para calcular BPM
int beatCount = 0;  // Contador de latidos
int lastPulseValue = 0;  // Último valor del sensor

void setup() {
  Serial.begin(115200);  // Iniciar comunicación serie
  Serial.println("Iniciando lectura del sensor de pulso en ESP32...");
}

void loop() {
  // Leer el valor actual del sensor de pulso
  int pulseValue = analogRead(pulsePin);

  // Detectar un latido si la diferencia con el valor anterior es mayor al umbral
  if (abs(pulseValue - lastPulseValue) > changeThreshold) {
    unsigned long currentMillis = millis();

    // Verificar que haya pasado suficiente tiempo desde el último latido (600 ms para evitar falsos positivos)
    if (currentMillis - lastBeatTime > 600) {
      beatCount++;  // Incrementar el contador de latidos
      lastBeatTime = currentMillis;  // Actualizar tiempo del último latido detectado
    }
  }

  // Actualizar el último valor de pulso
  lastPulseValue = pulseValue;

  // Calcular BPM cada 15 segundos
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;

    // Calcular las BPM (latidos por minuto)
    int bpm = beatCount * (60 / 15);  // Multiplicar por 60 y dividir por 15 porque estamos contando latidos en 15 segundos

    // Imprimir BPM solo si es mayor a 0
    if (bpm > 0) {
      Serial.print("Pulsaciones por minuto (BPM): ");
      Serial.println(bpm);
    } else {
      Serial.println("No se detectaron pulsaciones en el intervalo.");
    }

    // Reiniciar el contador de latidos para el siguiente ciclo de 15 segundos
    beatCount = 0;
  }

  delay(20);  // Pequeña pausa antes de la próxima lectura
}*/


/*#include <HardwareSerial.h>

HardwareSerial SerialSIM7600(2);  // UART2 para el SIM7600SA (RX2=16, TX2=17)

const int pulsePin = 34;  // Pin donde está conectado el sensor de pulso en el ESP32
int changeThreshold = 150;  // Umbral para detectar cambios significativos en la señal
unsigned long lastBeatTime = 0;  // Tiempo del último latido detectado
unsigned long previousMillis = 0;  // Tiempo anterior para calcular BPM y coordenadas GPS
const long interval = 15000;  // Intervalo de 15 segundos para cálculo de BPM y obtención de GPS

float latitud = 0.0;
float longitud = 0.0;
int bpm = 0;
int beatCount = 0;
int lastPulseValue = 0;

void setup() {
  Serial.begin(115200);  // Monitor serial para depuración
  SerialSIM7600.begin(115200, SERIAL_8N1, 16, 17);  // UART2 para SIM7600SA

  Serial.println("Inicializando el módulo SIM7600SA y GPS...");

  // Activar el GPS en el módulo (reintento en caso de error)
  activarGPS();
}

void loop() {
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;

    // Calcular y mostrar ritmo cardíaco
    bpm = calcularRitmoCardiaco();
    if (bpm > 0) {
      Serial.print("Pulsaciones por minuto (BPM): ");
      Serial.println(bpm);
    } else {
      Serial.println("No se detectaron pulsaciones en el intervalo.");
    }

    // Obtener y mostrar coordenadas GPS
    obtenerCoordenadasGPS();
  }
}

// Función para activar el GPS
void activarGPS() {
  SerialSIM7600.println("AT+CGPS=1,1");  // Activar el GPS en modo autónomo
  delay(3000);
  mostrarRespuesta();

  // Comprobar si el GPS está realmente activado
  SerialSIM7600.println("AT+CGPS?");
  delay(2000);
  String estadoGPS = leerRespuesta();

  if (estadoGPS.indexOf("+CGPS: 1") == -1) {
    Serial.println("Error al activar el GPS. Reintentando...");
    delay(5000);  // Esperar antes de reintentar
    activarGPS();  // Reintentar la activación
  } else {
    Serial.println("GPS activado correctamente.");
  }
}

// Función para obtener y convertir las coordenadas GPS
void obtenerCoordenadasGPS() {
  Serial.println("Obteniendo datos del GPS...");

  for (int intentos = 0; intentos < 3; intentos++) {
    SerialSIM7600.println("AT+CGPSINFO");
    delay(2000);
    String respuesta = leerRespuesta();

    if (respuesta.indexOf("+CGPSINFO:") != -1 && respuesta.indexOf(",,,,,,,") == -1) {
      // Extraer y convertir latitud y longitud si los datos son válidos
      if (procesarDatosGPS(respuesta)) {
        Serial.print("Latitud: ");
        Serial.println(latitud, 6);
        Serial.print("Longitud: ");
        Serial.println(longitud, 6);
        return;
      }
    } else {
      Serial.println("No se pudieron obtener las coordenadas GPS. Reintentando...");
      delay(5000);  
    }
  }

  Serial.println("No se pudieron obtener las coordenadas GPS después de varios intentos.");
}

// Función para procesar la cadena de datos GPS y convertir las coordenadas
bool procesarDatosGPS(String datosGPS) {
  int indexLat = datosGPS.indexOf(':') + 1;
  int indexNS = datosGPS.indexOf(',', indexLat + 1);
  int indexLong = datosGPS.indexOf(',', indexNS + 1);
  int indexEW = datosGPS.indexOf(',', indexLong + 1);

  String latStr = datosGPS.substring(indexLat + 1, indexNS);
  String ns = datosGPS.substring(indexNS + 1, indexNS + 2);
  String lonStr = datosGPS.substring(indexLong + 1, indexEW);
  String ew = datosGPS.substring(indexEW + 1, indexEW + 2);

  if (latStr.length() == 0 || lonStr.length() == 0) {
    Serial.println("Datos GPS inválidos.");
    return false;
  }

  // Convertir latitud
  latitud = convertirCoordenada(latStr, ns);

  // Convertir longitud
  longitud = convertirCoordenada(lonStr, ew);

  return true;
}

float convertirCoordenada(String coord, String direccion) {
  int grados = (direccion == "N" || direccion == "S") ? coord.substring(0, 2).toInt() : coord.substring(0, 3).toInt();
  float minutos = coord.substring((direccion == "N" || direccion == "S") ? 2 : 3).toFloat();
  float decimal = grados + (minutos / 60.0);

  if (direccion == "S" || direccion == "W") {
    decimal = -decimal;
  }
  return decimal;
}

// Función para calcular el ritmo cardíaco
int calcularRitmoCardiaco() {
  int beatCount = 0;
  unsigned long lastBeatTime = 0;

  for (unsigned long start = millis(); millis() - start < interval; ) {
    int pulseValue = analogRead(pulsePin);

    if (abs(pulseValue - lastPulseValue) > changeThreshold) {
      unsigned long currentMillis = millis();

      if (currentMillis - lastBeatTime > 600) {  // Evitar latidos falsos (600 ms)
        beatCount++;
        lastBeatTime = currentMillis;
      }
    }
    lastPulseValue = pulseValue;
    delay(20);
  }

  return beatCount * 4;  // Calcular BPM basado en 15 segundos
}

// Función para leer la respuesta del módulo
String leerRespuesta() {
  String respuesta = "";
  unsigned long startTime = millis();
  while (millis() - startTime < 5000) {  // Esperar hasta 5 segundos
    while (SerialSIM7600.available()) {
      char c = SerialSIM7600.read();
      respuesta += c;
    }
  }
  respuesta.trim();
  Serial.println("Respuesta del SIM7600SA: " + respuesta);
  return respuesta;
}

// Función para mostrar la respuesta del módulo (para depuración)
void mostrarRespuesta() {
  while (SerialSIM7600.available()) {
    String respuesta = SerialSIM7600.readStringUntil('\n');
    respuesta.trim();
    Serial.println("Respuesta del SIM7600SA: " + respuesta);
  }
}*/





//PERFECTO EXCEPTO POR EL RITMO CARDIACO
/*#define TINY_GSM_MODEM_SIM7600
#include <HardwareSerial.h>
#include <TinyGPS++.h>
#include <TinyGsmClient.h>
#include <ArduinoHttpClient.h>

// Configuración de UART para SIM7600SA y GPS
HardwareSerial SerialSIM7600(2);  // UART2 para el SIM7600SA (RX2=16, TX2=17)
TinyGPSPlus gps;
TinyGsm modem(SerialSIM7600);
TinyGsmClient client(modem);
HttpClient http(client, "guardian-service-iot.rojasanthony16.workers.dev", 80);

// Variables para el ritmo cardíaco
const int pulsePin = 34;  // Pin analógico en el ESP32 para el sensor de pulso
int changeThreshold = 150;
unsigned long lastBeatTime = 0;
unsigned long previousMillisHeart = 0;
int beatCount = 0;
int lastPulseValue = 0;
int bpm = 0;

// Variables para el GPS y la conexión
String imei;
float latitud = 0.0, longitud = 0.0;
bool gprsConectado = false;
bool gpsActivo = false;
unsigned long previousMillis = 0;
const long intervaloEnvio = 15000;  // 15 segundos

// APN de Claro
const char* apn = "claro.pe";

void setup() {
  Serial.begin(115200);                   // Monitor Serial
  SerialSIM7600.begin(115200, SERIAL_8N1, 16, 17);  // UART2 para SIM7600SA

  Serial.println("Inicializando SIM7600SA...");
  if (!reiniciarModem()) {
    Serial.println("Error al reiniciar el módem.");
    return;
  }

  if (!obtenerIMEI()) {
    Serial.println("Error al obtener IMEI.");
    return;
  }
  Serial.println("IMEI obtenido: " + imei);

  // Activar el GPS
  activarGPS();

  // Conectar usando el APN de Claro
  configurarGPRS();
}

void loop() {
  unsigned long currentMillis = millis();
  
  // Monitoreo del ritmo cardíaco
  monitorRitmoCardiaco();

  // Verificación y envío de datos cada 15 segundos
  if (currentMillis - previousMillis >= intervaloEnvio) {
    previousMillis = currentMillis;

    if (obtenerCoordenadasGPS()) {  // Intentar obtener coordenadas GPS
      if (gprsConectado) {
        enviarDatos();
      } else {
        configurarGPRS();  // Reintentar conectar si no está conectado
      }
    } else {
      Serial.println("Esperando señal GPS válida...");
    }
  }
}

// Función para monitorear el ritmo cardíaco y calcular BPM
void monitorRitmoCardiaco() {
  int pulseValue = analogRead(pulsePin);

  // Detección de latido
  if (abs(pulseValue - lastPulseValue) > changeThreshold) {
    unsigned long currentMillisHeart = millis();

    if (currentMillisHeart - lastBeatTime > 600) {  // Evitar latidos falsos (600 ms)
      beatCount++;
      lastBeatTime = currentMillisHeart;
    }
  }

  lastPulseValue = pulseValue;

  // Calcular BPM cada 15 segundos
  if (millis() - previousMillisHeart >= intervaloEnvio) {
    previousMillisHeart = millis();
    bpm = beatCount * (60 / 15);  // Calcular BPM basado en los últimos 15 segundos
    beatCount = 0;
    
    Serial.print("Pulsaciones por minuto (BPM): ");
    Serial.println(bpm);
  }
}

// Activar el GPS
void activarGPS() {
  enviarComandoAT("AT+CGPS=1,1");  // Enciende el GPS en modo autónomo
  delay(2000);
  gpsActivo = true;
  Serial.println("GPS activado.");
}


// Función para obtener y convertir las coordenadas GPS
bool obtenerCoordenadasGPS() {
  Serial.println("Obteniendo datos del GPS...");

  for (int intentos = 0; intentos < 3; intentos++) {
    SerialSIM7600.println("AT+CGPSINFO");
    delay(2000);
    String respuesta = leerRespuesta(5000);

    if (respuesta.indexOf("+CGPSINFO:") != -1 && respuesta.indexOf(",,,,,,,") == -1) {
      // Extraer y convertir latitud y longitud si los datos son válidos
      if (procesarDatosGPS(respuesta)) {
        Serial.print("Latitud: ");
        Serial.println(latitud, 6);
        Serial.print("Longitud: ");
        Serial.println(longitud, 6);
        return true;
      }
    } else {
      Serial.println("No se pudieron obtener las coordenadas GPS. Reintentando...");
      delay(5000);  
    }
  }

  Serial.println("No se pudieron obtener las coordenadas GPS después de varios intentos.");

  return false;
}

// Función para procesar la cadena de datos GPS y convertir las coordenadas
bool procesarDatosGPS(String datosGPS) {
  int indexLat = datosGPS.indexOf(':') + 1;
  int indexNS = datosGPS.indexOf(',', indexLat + 1);
  int indexLong = datosGPS.indexOf(',', indexNS + 1);
  int indexEW = datosGPS.indexOf(',', indexLong + 1);

  String latStr = datosGPS.substring(indexLat + 1, indexNS);
  String ns = datosGPS.substring(indexNS + 1, indexNS + 2);
  String lonStr = datosGPS.substring(indexLong + 1, indexEW);
  String ew = datosGPS.substring(indexEW + 1, indexEW + 2);

  if (latStr.length() == 0 || lonStr.length() == 0) {
    Serial.println("Datos GPS inválidos.");
    return false;
  }

  // Convertir latitud
  latitud = convertirCoordenada(latStr, ns);

  // Convertir longitud
  longitud = convertirCoordenada(lonStr, ew);

  return true;
}

float convertirCoordenada(String coord, String direccion) {
  int grados = (direccion == "N" || direccion == "S") ? coord.substring(0, 2).toInt() : coord.substring(0, 3).toInt();
  float minutos = coord.substring((direccion == "N" || direccion == "S") ? 2 : 3).toFloat();
  float decimal = grados + (minutos / 60.0);

  if (direccion == "S" || direccion == "W") {
    decimal = -decimal;
  }
  return decimal;
}


// Reiniciar el SIM7600SA
bool reiniciarModem() {
  enviarComandoAT("AT");  // Comando básico para verificar si está en línea
  delay(3000);
  if (leerRespuesta(5000).indexOf("OK") != -1) {
    Serial.println("SIM7600SA está en línea.");
    return true;
  }
  Serial.println("No se pudo verificar el estado del módem.");
  return false;
}


// Obtener el IMEI del SIM7600SA
bool obtenerIMEI() {
  enviarComandoAT("AT+GSN");  // Comando para obtener el IMEI
  String respuesta = leerRespuesta(5000);

  // Eliminar cualquier espacio o salto de línea adicional en la respuesta
  respuesta.trim();

  // Buscar el IMEI en la respuesta
  int imeiStart = respuesta.indexOf('\n') + 1;  // Buscar el primer salto de línea tras el comando AT+GSN

  if (imeiStart > 0) {
    imei = respuesta.substring(imeiStart, imeiStart + 15); // Extraer los primeros 15 caracteres para el IMEI
    Serial.println("IMEI extraído: " + imei);
    return true;
  }

  Serial.println("Error al extraer el IMEI.");
  return false;
}

// Configurar GPRS usando el APN de Claro
void configurarGPRS() {
  Serial.print("Intentando conectar con APN: ");
  Serial.println(apn);

  if (conectarGPRS(apn, "", "")) {
    Serial.println("Conexión GPRS exitosa.");
    gprsConectado = true;
  } else {
    Serial.println("Error al conectar con el APN de Claro.");
    gprsConectado = false;
  }
}

// Conectar al GPRS con un APN específico
bool conectarGPRS(const char* apn, const char* user, const char* pass) {
  return modem.gprsConnect(apn, user, pass);
}



// Enviar datos al servidor
void enviarDatos() {
  String json = "{\"SIM\":\"" + imei + "\",\"Latitud\":" + String(latitud, 6) +
                ",\"Longitud\":" + String(longitud, 6) + ",\"RitmoCardiaco\":" + String(bpm) + "}";

  Serial.print("Datos JSON a enviar: ");
  Serial.println(json);
  //delay(200);
  Serial.println("Enviando datos al servidor...");
  http.beginRequest();
  http.post("/");
  http.sendHeader("Content-Type", "application/json");
  http.sendHeader("Content-Length", json.length());
  http.beginBody();
  http.print(json);
  http.endRequest();


  int statusCode = http.responseStatusCode();
  Serial.print("Código de respuesta: ");
  Serial.println(statusCode);

  if (statusCode != 200) {
    Serial.println("Error en la petición HTTP.");
    gprsConectado = false;  // Intentar reconectar si falla la petición
    String response = http.responseBody();
    Serial.println("Respuesta del servidor: " + response);
  } else {
    String response = http.responseBody();
    Serial.println("Respuesta del servidor: " + response);
  }
}

// Leer la respuesta del SIM7600SA, ignorando líneas vacías
String leerRespuesta(unsigned long timeout) {
  String respuesta = "";
  unsigned long startTime = millis();
  while (millis() - startTime < timeout) {
    while (SerialSIM7600.available()) {
      String linea = SerialSIM7600.readStringUntil('\n');
      linea.trim();  // Eliminar espacios en blanco
      if (linea.length() > 0) {
        respuesta += linea + "\n";
      }
    }
  }
  return respuesta;
}


// Función para mostrar la respuesta del módulo (para depuración)
void mostrarRespuesta() {
  while (SerialSIM7600.available()) {
    String respuesta = SerialSIM7600.readStringUntil('\n');
    respuesta.trim();
    Serial.println("Respuesta del SIM7600SA: " + respuesta);
  }
}

// Función para enviar un comando AT
void enviarComandoAT(const char* comando) {
  SerialSIM7600.println(comando);
  Serial.print("Enviado: ");
  Serial.println(comando);
}*/

//CÓDIGO FINAL EN ESP32
/*#define TINY_GSM_MODEM_SIM7600
#include <HardwareSerial.h>
//#include <TinyGPS++.h>
#include <TinyGsmClient.h>
#include <ArduinoHttpClient.h>

// Configuración de UART para SIM7600SA y GPS
HardwareSerial SerialSIM7600(2);  // UART2 para el SIM7600SA (RX2=16, TX2=17)
//TinyGPSPlus gps;
TinyGsm modem(SerialSIM7600);
TinyGsmClient client(modem);
HttpClient http(client, "guardian-service-iot.rojasanthony16.workers.dev", 80);

//variables para leds de GPS y envío de datos
const int ledGPS = 18;  // Pin GPIO para el LED1
const int ledHTTP = 19;  // Pin GPIO para el LED2

// Variables para el ritmo cardíaco
const int pulsePin = 34;  // Pin analógico en el ESP32 para el sensor de pulso
int changeThreshold = 150;
unsigned long lastBeatTime = 0;
unsigned long previousMillisHeart = 0;
int beatCount = 0;
int lastPulseValue = 0;
int bpm = 0;
int reintentosImei =3;

// Variables para el GPS y la conexión
String imei;
float latitud = 0.0, longitud = 0.0;
bool gprsConectado = false;
bool gpsActivo = false;
unsigned long previousMillis = 0;
const long intervaloEnvio = 15000;  // 15 segundos

// APN de Claro
const char* apn = "claro.pe";

void setup() {
  pinMode(ledGPS, OUTPUT); 
  pinMode(ledHTTP, OUTPUT); 

  Serial.begin(115200);                   // Monitor Serial
  SerialSIM7600.begin(115200, SERIAL_8N1, 16, 17);  // UART2 para SIM7600SA

  delay(40000); //ESPERAR A QUE EL MÓDULO REACCIONE
  Serial.println("Inicializando SIM7600SA...");
  if (!reiniciarModem()) {
    Serial.println("Error al reiniciar el módem.");
    return;
  }
  //if (!obtenerIMEI()) {
    //Serial.println("Error al obtener IMEI.");
   // return;
  //}
  //Serial.println("IMEI obtenido: " + imei);
  int reintentos =0;
  while(!obtenerIMEI()|| imei.isEmpty()){
    delay(200);
    Serial.println("Error al obtener IMEI.");
    reintentos++;
    delay(200);
    if(reintentos==reintentosImei){
      reiniciarTodo();
      reintentos=0;
      delay(200);
    }
  }
  digitalWrite(ledHTTP, HIGH);
  delay(200);
  digitalWrite(ledHTTP, LOW);
  delay(200);
  digitalWrite(ledHTTP, HIGH);
  delay(200);
  digitalWrite(ledHTTP, LOW);
  delay(200);
  digitalWrite(ledHTTP, HIGH);
  delay(200);
  digitalWrite(ledHTTP, LOW);

  Serial.println("IMEI obtenido: " + imei);
  // Activar el GPS
  delay(10000);
  activarGPS();
  delay(5000);

  // Conectar usando el APN de Claro
  configurarGPRS();
}

void loop() {

  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= intervaloEnvio) {
    previousMillis = currentMillis;

    // Calcular y mostrar ritmo cardíaco
    bpm = calcularRitmoCardiaco();
    if (bpm > 0) {
      Serial.print("Pulsaciones por minuto (BPM): ");
      Serial.println(bpm);
    } else {
      Serial.println("No se detectaron pulsaciones en el intervalo.");
    }

    // Obtener y mostrar coordenadas GPS
    if (obtenerCoordenadasGPS()) {  // Intentar obtener coordenadas GPS
      if (gprsConectado) {
        enviarDatos();
      } else {
        configurarGPRS();  // Reintentar conectar si no está conectado
      }
    } else {
      Serial.println("Esperando señal GPS válida...");
    }
  }

}

void reiniciarTodo(){
SerialSIM7600.println("AT+CFUN=1,1");
delay(20000);
}

// Función para monitorear el ritmo cardíaco y calcular BPM
void monitorRitmoCardiaco() {
  int pulseValue = analogRead(pulsePin);

  // Detección de latido
  if (abs(pulseValue - lastPulseValue) > changeThreshold) {
    unsigned long currentMillisHeart = millis();

    if (currentMillisHeart - lastBeatTime > 600) {  // Evitar latidos falsos (600 ms)
      beatCount++;
      lastBeatTime = currentMillisHeart;
    }
  }

  lastPulseValue = pulseValue;

  // Calcular BPM cada 15 segundos
  if (millis() - previousMillisHeart >= intervaloEnvio) {
    previousMillisHeart = millis();
    bpm = beatCount * (60 / 15);  // Calcular BPM basado en los últimos 15 segundos
    beatCount = 0;
    
    Serial.print("Pulsaciones por minuto (BPM): ");
    Serial.println(bpm);
  }
}

// Activar el GPS
void activarGPS() {
  enviarComandoAT("AT+CGPS=1,1");  // Enciende el GPS en modo autónomo
  delay(2000);
  gpsActivo = true;
  Serial.println("GPS activado.");
}


// Función para obtener y convertir las coordenadas GPS
bool obtenerCoordenadasGPS() {
  Serial.println("Obteniendo datos del GPS...");

  for (int intentos = 0; intentos < 3; intentos++) {
    SerialSIM7600.println("AT+CGPSINFO");
    delay(2000);
    String respuesta = leerRespuesta(5000);

    Serial.print("Respuesta del GPS: ");
    Serial.print(respuesta);
    delay(200);
    if (respuesta.indexOf("+CGPSINFO:") != -1 && respuesta.indexOf(",,,,,,,") == -1) {
      // Extraer y convertir latitud y longitud si los datos son válidos
      if (procesarDatosGPS(respuesta)) {
        Serial.print("Latitud: ");
        Serial.println(latitud, 6);
        Serial.print("Longitud: ");
        Serial.println(longitud, 6);
        digitalWrite(ledGPS, HIGH);
        delay(200);
        digitalWrite(ledGPS, LOW);
        delay(200);
        digitalWrite(ledGPS, HIGH);
        delay(200);
        digitalWrite(ledGPS, LOW);
        return true;
      }
    } else {
      digitalWrite(ledGPS, HIGH);
      delay(500);
      digitalWrite(ledGPS, LOW);
      Serial.println("No se pudieron obtener las coordenadas GPS. Reintentando...");
      Serial.println("Intentando Activar GPS...");
      activarGPS();
      //delay(5000);  
    }
  }

  Serial.println("No se pudieron obtener las coordenadas GPS después de varios intentos.");

  Serial.println("Reiniciando GPS...");
  SerialSIM7600.println("AT+CGPSRST=1");
  delay(12000);
  SerialSIM7600.println("AT+CGPS=1,1");
  delay(2000);
  SerialSIM7600.println("AT+CGPS?");

  return false;
}

// Función para procesar la cadena de datos GPS y convertir las coordenadas
bool procesarDatosGPS(String datosGPS) {
  int indexLat = datosGPS.indexOf(':') + 1;
  int indexNS = datosGPS.indexOf(',', indexLat + 1);
  int indexLong = datosGPS.indexOf(',', indexNS + 1);
  int indexEW = datosGPS.indexOf(',', indexLong + 1);

  String latStr = datosGPS.substring(indexLat + 1, indexNS);
  String ns = datosGPS.substring(indexNS + 1, indexNS + 2);
  String lonStr = datosGPS.substring(indexLong + 1, indexEW);
  String ew = datosGPS.substring(indexEW + 1, indexEW + 2);

  if (latStr.length() == 0 || lonStr.length() == 0) {
    Serial.println("Datos GPS inválidos.");
    return false;
  }

  // Convertir latitud
  latitud = convertirCoordenada(latStr, ns);

  // Convertir longitud
  longitud = convertirCoordenada(lonStr, ew);

  return true;
}

float convertirCoordenada(String coord, String direccion) {
  int grados = (direccion == "N" || direccion == "S") ? coord.substring(0, 2).toInt() : coord.substring(0, 3).toInt();
  float minutos = coord.substring((direccion == "N" || direccion == "S") ? 2 : 3).toFloat();
  float decimal = grados + (minutos / 60.0);

  if (direccion == "S" || direccion == "W") {
    decimal = -decimal;
  }
  return decimal;
}


// Reiniciar el SIM7600SA
bool reiniciarModem() {
  enviarComandoAT("AT");  // Comando básico para verificar si está en línea
  delay(3000);
  if (leerRespuesta(5000).indexOf("OK") != -1) {
    Serial.println("SIM7600SA está en línea.");
    return true;
  }
  Serial.println("No se pudo verificar el estado del módem.");
  return false;
}


// Obtener el IMEI del SIM7600SA
bool obtenerIMEI() {
  enviarComandoAT("AT+GSN");  // Comando para obtener el IMEI
  String respuesta = leerRespuesta(5000);

  // Eliminar cualquier espacio o salto de línea adicional en la respuesta
  respuesta.trim();

  // Buscar el IMEI en la respuesta
  int imeiStart = respuesta.indexOf('\n') + 1;  // Buscar el primer salto de línea tras el comando AT+GSN

  if (imeiStart > 0) {
    imei = respuesta.substring(imeiStart, imeiStart + 15); // Extraer los primeros 15 caracteres para el IMEI
    Serial.println("IMEI extraído: " + imei);
    return true;
  }

  Serial.println("Error al extraer el IMEI.");
  return false;
}

// Configurar GPRS usando el APN de Claro
void configurarGPRS() {
  Serial.print("Intentando conectar con APN: ");
  Serial.println(apn);

  if (conectarGPRS(apn, "", "")) {
    Serial.println("Conexión GPRS exitosa.");
    gprsConectado = true;
  } else {
    Serial.println("Error al conectar con el APN de Claro.");
    gprsConectado = false;
  }
}

// Conectar al GPRS con un APN específico
bool conectarGPRS(const char* apn, const char* user, const char* pass) {
  return modem.gprsConnect(apn, user, pass);
}




void enviarDatos() {
  String json = "{\"SIM\":\"" + imei + "\",\"Latitud\":" + String(latitud, 6) +
                ",\"Longitud\":" + String(longitud, 6) + ",\"RitmoCardiaco\":" + String(bpm) + "}";

  Serial.print("Datos JSON a enviar: ");
  Serial.println(json);

  Serial.println("Enviando datos al servidor...");

  // Configuración de timeout y conexión
  http.connectionKeepAlive(); // Mantener la conexión activa
  http.setTimeout(10000);     // Tiempo máximo de espera reducido a 10 segundos

  int maxReintentos = 3; // Intentar hasta 3 veces en caso de fallo
  int reintentos = 0;
  int statusCode = -1;
  
  while (reintentos < maxReintentos && statusCode != 200) {
    http.beginRequest();
    http.post("/");
    http.sendHeader("Content-Type", "application/json");
    http.sendHeader("Content-Length", json.length());
    http.beginBody();
    http.print(json);
    http.endRequest();

    // Obtener el código de respuesta
    statusCode = http.responseStatusCode();
    Serial.print("Código de respuesta: ");
    Serial.println(statusCode);

    if (statusCode != 200) {
      Serial.println("Error en la petición HTTP. Reintentando...");
      digitalWrite(ledHTTP, HIGH);
      delay(500);
      digitalWrite(ledHTTP, LOW);

      // Intentar reconectar GPRS y HTTP después de un error
      gprsConectado = false;
      modem.gprsDisconnect();
      delay(2000);
      configurarGPRS();
      
      reintentos++;
      delay(500);  // Esperar un momento antes de reintentar
    } else {
      Serial.println("Datos enviados correctamente.");
      digitalWrite(ledHTTP, HIGH);
      delay(200);
      digitalWrite(ledHTTP, LOW);
      String response = http.responseBody();
      Serial.println("Respuesta del servidor: " + response);
    }
  }

  // Comprobar si el envío de datos falló después de todos los reintentos
  if (statusCode != 200) {
    Serial.println("No se pudo enviar los datos después de varios intentos.");
    reiniciarTodo();
    // Realizar otras acciones de recuperación si es necesario
  }
}


// Leer la respuesta del SIM7600SA, ignorando líneas vacías
String leerRespuesta(unsigned long timeout) {
  String respuesta = "";
  unsigned long startTime = millis();
  while (millis() - startTime < timeout) {
    while (SerialSIM7600.available()) {
      String linea = SerialSIM7600.readStringUntil('\n');
      linea.trim();  // Eliminar espacios en blanco
      if (linea.length() > 0) {
        respuesta += linea + "\n";
      }
    }
  }
  return respuesta;
}


// Función para mostrar la respuesta del módulo (para depuración)
void mostrarRespuesta() {
  while (SerialSIM7600.available()) {
    String respuesta = SerialSIM7600.readStringUntil('\n');
    respuesta.trim();
    Serial.println("Respuesta del SIM7600SA: " + respuesta);
  }
}

// Función para enviar un comando AT
void enviarComandoAT(const char* comando) {
  SerialSIM7600.println(comando);
  Serial.print("Enviado: ");
  Serial.println(comando);
}




int calcularRitmoCardiaco() {
  int beatCount = 0;
  unsigned long lastBeatTime = 0;

  for (unsigned long start = millis(); millis() - start < intervaloEnvio; ) {
    int pulseValue = analogRead(pulsePin);

    if (abs(pulseValue - lastPulseValue) > changeThreshold) {
      unsigned long currentMillis = millis();

      if (currentMillis - lastBeatTime > 600) {  // Evitar latidos falsos (600 ms)
        beatCount++;
        lastBeatTime = currentMillis;
      }
    }
    lastPulseValue = pulseValue;
    delay(20);
  }

  return beatCount * 4;  // Calcular BPM basado en 15 segundos
}
*/


/*const int pulsePin = 34;         // Pin del sensor de ritmo cardíaco en el ESP32
int readings[10];                // Almacenamiento de lecturas para filtro de media móvil
int readIndex = 0;
int total = 0;
int average = 0;
int threshold = 0;               // Umbral dinámico calculado
int peak = 0;                    // Pico reciente
int valley = 4096;               // Valle reciente (valor alto inicial para ajustar)
unsigned long lastBeatTime = 0;  // Tiempo del último latido detectado
int beatCount = 0;
int bpm = 0;
const int interval = 1000;        // Intervalo mínimo entre latidos (ms)
unsigned long lastBpmCalcTime = 0;


void setup() {
  Serial.begin(115200);
  for (int i = 0; i < 10; i++) readings[i] = 0;  // Inicializa el array de lecturas
  Serial.println("Iniciando detección de ritmo cardíaco...");
}


void loop() {
  // Lee el valor del sensor
  int pulseValue = analogRead(pulsePin);

  // Actualiza el filtro de media móvil
  total = total - readings[readIndex];
  readings[readIndex] = pulseValue;
  total = total + readings[readIndex];
  readIndex = (readIndex + 1) % 10; // Avanza el índice en el array circular
  average = total / 10;

  // Ajusta el pico y el valle dinámicamente
  if (average > peak) peak = average;       // Actualiza el pico
  if (average < valley) valley = average;   // Actualiza el valle

  // Calcula el umbral dinámico como punto medio entre pico y valle
  threshold = (peak + valley) / 2;

  // Detección de latido usando el umbral dinámico
  if (average > threshold && (millis() - lastBeatTime > interval)) {
    beatCount++;
    lastBeatTime = millis();
    Serial.println("Latido detectado");
  }

  // Calcular BPM cada 15 segundos independientemente de lastBeatTime
  if (millis() - lastBpmCalcTime >= 15000) {
    bpm = (beatCount * 4); // Basado en 15 segundos
    Serial.print("BPM: ");
    Serial.println(bpm);
    beatCount = 0;
    peak = 0;        // Reinicia el pico para recalcular en la siguiente ventana
    valley = 4096;   // Reinicia el valle para recalcular
    lastBpmCalcTime = millis();
  }

  delay(10);
}*/


/*#include <HardwareSerial.h>

// Configuración de UART para el módulo SIM7600SA
HardwareSerial SerialSIM7600(2);  // UART2 (RX2=16, TX2=17)

void setup() {
  // Configurar la comunicación serial con el PC y el módulo SIM7600SA
  Serial.begin(115200);                   // Monitor Serial
  SerialSIM7600.begin(115200, SERIAL_8N1, 16, 17);  // UART2 para SIM7600SA

  Serial.println("Prueba de comandos AT para el SIM7600SA");
  Serial.println("Escribe un comando AT y presiona Enter:");
}

void loop() {
  // Enviar comandos AT desde el monitor serial al módulo SIM7600SA
  if (Serial.available()) {
    String comando = Serial.readStringUntil('\n');  // Leer el comando desde el monitor serial
    SerialSIM7600.println(comando);                 // Enviar el comando al módulo SIM7600SA
    Serial.print("Enviado: ");
    Serial.println(comando);
  }

  // Leer y mostrar la respuesta del módulo SIM7600SA en el monitor serial
  if (SerialSIM7600.available()) {
    String respuesta = SerialSIM7600.readStringUntil('\n');
    respuesta.trim();  // Eliminar espacios en blanco y saltos de línea adicionales
    if (respuesta.length() > 0) {
      Serial.print("Respuesta: ");
      Serial.println(respuesta);
    }
  }
}*/

/*
#define TINY_GSM_MODEM_SIM7600
#include <HardwareSerial.h>
#include <TinyGsmClient.h>
#include <ArduinoHttpClient.h>

// Configuración de los pines y puerto serial para el SIM7600SA
HardwareSerial SerialSIM7600(2);  // UART2 (RX=16, TX=17)
TinyGsm modem(SerialSIM7600);      // Configuración para TinyGSM y el SIM7600SA
TinyGsmClient client(modem);       // Cliente GPRS
HttpClient http(client, "guardian-service-iot.rojasanthony16.workers.dev", 80); // URL de destino

// Datos a enviar
String sim = "863427045143212";
float latitud = -11.919392;
float longitud = -77.087097;
int ritmoCardiaco = 80;

void setup() {
  Serial.begin(115200);
  SerialSIM7600.begin(115200, SERIAL_8N1, 16, 17);  // Configurar UART para el SIM7600SA (RX2=16, TX2=17)

  delay(3000);  // Espera inicial para el módulo
  
  // Inicializar el módem y la conexión GPRS
  Serial.println("Inicializando módem...");
  if (!modem.restart()) {
    Serial.println("Error al reiniciar el módem");
    return;
  }
  Serial.println("Módem inicializado.");

  // Conectar a la red GPRS
  const char apn[] = "claro.pe";  // Reemplaza "tu_apn" con el APN de tu operador (p. ej., "claro.pe" para Claro)
  if (!modem.gprsConnect(apn, "", "")) {
    Serial.println("Error al conectar a la red GPRS");
    return;
  }
  Serial.println("Conexión GPRS establecida.");
}

void loop() {
  // Crear el JSON con los datos
  String json = "{\"SIM\":\"" + sim + "\",\"Latitud\":" + String(latitud, 6) +
                ",\"Longitud\":" + String(longitud, 6) + ",\"RitmoCardiaco\":" + String(ritmoCardiaco) + "}";

  Serial.print("Datos JSON a enviar: ");
  Serial.println(json);

  // Enviar datos al servidor
  Serial.println("Enviando datos al servidor...");
  http.connectionKeepAlive();       // Mantener conexión activa
  http.beginRequest();
  http.post("/");
  http.sendHeader("Content-Type", "application/json");
  http.sendHeader("Content-Length", json.length());
  http.beginBody();
  http.print(json);
  http.endRequest();

  // Obtener respuesta del servidor
  int statusCode = http.responseStatusCode();
  String response = http.responseBody();
  
  Serial.print("Código de respuesta: ");
  Serial.println(statusCode);
  Serial.print("Respuesta del servidor: ");
  Serial.println(response);

  // Desconectar del GPRS después de la solicitud si deseas ahorrar energía
  // modem.gprsDisconnect();

  delay(10000); // Espera de 1 minuto antes de enviar el siguiente conjunto de datos
}*/



#define TINY_GSM_MODEM_SIM7600
#include <HardwareSerial.h>
#include <TinyGsmClient.h>
#include <ArduinoHttpClient.h>

// Configuración de los pines y puerto serial para el SIM7600SA
HardwareSerial SerialSIM7600(2);  // UART2 (RX=16, TX=17)
TinyGsm modem(SerialSIM7600);      // Configuración para TinyGSM y el SIM7600SA
TinyGsmClient client(modem);       // Cliente GPRS
HttpClient http(client, "guardian-service-iot.rojasanthony16.workers.dev", 80); // URL de destino
//HttpClient http(client, "guardian-service.onrender.com/api/paciente/actualizarIoT", 80); // URL de destino
//variables para leds de GPS y envío de datos
const int ledGPS = 18;  // Pin GPIO para el LED1
const int ledHTTP = 19;  // Pin GPIO para el LED2

// Datos de SIM y ritmo cardíaco
String imei = "";
int ritmoCardiaco = 0;
float latitud = 0.0;
float longitud = 0.0;


// Variables para el ritmo cardíaco
const int pulsePin = 34;  // Pin analógico en el ESP32 para el sensor de pulso
int changeThreshold = 150;
unsigned long lastBeatTime = 0;
unsigned long previousMillisHeart = 0;
int beatCount = 0;
int lastPulseValue = 0;
unsigned long previousMillis = 0;
const long intervaloEnvio = 15000;  // 15 segundos

void setup() {
  Serial.begin(115200);
  SerialSIM7600.begin(115200, SERIAL_8N1, 16, 17);  // Configurar UART para el SIM7600SA (RX2=16, TX2=17)
  delay(3000);  // Espera inicial para el módulo
  pinMode(ledGPS, OUTPUT); 
  pinMode(ledHTTP, OUTPUT); 
  
  // Inicializar el módem y la conexión GPRS
  Serial.println("Inicializando módem...");
  while (!modem.restart()) {
    parpadeoLed(ledHTTP, 1, 2000);
    Serial.println("Error al reiniciar el módem. Reintentando...");
    //delay(2000);
  }
  Serial.println("Módem inicializado.");

  Serial.println("Obteniendo IMEI...");

  while(!obtenerIMEI()|| imei.isEmpty()){
    parpadeoLed(ledHTTP, 2, 2000);
    Serial.println("Error al obtener el IMEI. Reintentando...");
  }

  // Activar GPS del módulo
  Serial.println("Activando GPS...");
  SerialSIM7600.println("AT+CGPS=1,1");  // Comando para activar el GPS
  delay(2000);  // Espera para que el GPS se active

  while(!conectarRed()){
    delay(2000);
  }
  Serial.println("Conexión GPRS establecida.");
}

bool conectarRed(){
 // Conectar a la red GPRS
  const char apn[] = "claro.pe";  // Reemplaza "tu_apn" con el APN de tu operador
  if (!modem.gprsConnect(apn, "", "")) {
    Serial.println("Error al conectar a la red GPRS");
    return false;
  }
  return true;
}



void loop() {
    unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= intervaloEnvio) {
    previousMillis = currentMillis;

    // Calcular y mostrar ritmo cardíaco
    ritmoCardiaco = calcularRitmoCardiaco();
    if (ritmoCardiaco > 0) {
      Serial.print("Pulsaciones por minuto (BPM): ");
      Serial.println(ritmoCardiaco);
    } else {
      Serial.println("No se detectaron pulsaciones en el intervalo.");
    }

    // Obtener y mostrar coordenadas GPS
    /*if (obtenerCoordenadasGPS()) {  // Intentar obtener coordenadas GPS
      if (gprsConectado) {
        enviarDatos();
      } else {
        configurarGPRS();  // Reintentar conectar si no está conectado
      }
    } else {
      Serial.println("Esperando señal GPS válida...");
    }*/
    if(obtenerCoordenadasGPS()){
    enviarDatos();
  } 
  }


  /*if(obtenerCoordenadasGPS()){
    enviarDatos();
  } */
}


bool obtenerCoordenadasGPS() {
  Serial.println("Obteniendo datos del GPS...");

  for (int intentos = 0; intentos < 3; intentos++) {

  SerialSIM7600.println("AT+CGPSINFO");  // Comando para obtener información GPS
  delay(2000);  // Espera a la respuesta del módulo

  String gpsData = "";
  while (SerialSIM7600.available()) {
    gpsData += (char)SerialSIM7600.read();
  }

    Serial.print("Respuesta del GPS: ");
    Serial.print(gpsData);
    delay(200);
    if (gpsData.indexOf("+CGPSINFO:") != -1 && gpsData.indexOf(",,,,,,,") == -1) {
      // Extraer y convertir latitud y longitud si los datos son válidos
      if (procesarDatosGPS(gpsData)) {
        Serial.print("Latitud: ");
        Serial.println(latitud, 6);
        Serial.print("Longitud: ");
        Serial.println(longitud, 6);
        parpadeoLed(ledGPS, 2, 200);
        return true;
      }
    } else {
      parpadeoLed(ledGPS, 1, 500);
      Serial.println("No se pudieron obtener las coordenadas GPS. Reintentando...");
    }
  }
  Serial.println("No se pudieron obtener las coordenadas GPS después de varios intentos.");

  Serial.println("Reiniciando GPS...");
  SerialSIM7600.println("AT+CGPSRST=1");
  delay(10000);
  SerialSIM7600.println("AT+CGPS=1,1");
  delay(2000);
  SerialSIM7600.println("AT+CGPS?");


  return false;
}

// Función para procesar la cadena de datos GPS y convertir las coordenadas
bool procesarDatosGPS(String datosGPS) {
  int indexLat = datosGPS.indexOf(':') + 1;
  int indexNS = datosGPS.indexOf(',', indexLat + 1);
  int indexLong = datosGPS.indexOf(',', indexNS + 1);
  int indexEW = datosGPS.indexOf(',', indexLong + 1);

  String latStr = datosGPS.substring(indexLat + 1, indexNS);
  String ns = datosGPS.substring(indexNS + 1, indexNS + 2);
  String lonStr = datosGPS.substring(indexLong + 1, indexEW);
  String ew = datosGPS.substring(indexEW + 1, indexEW + 2);

  if (latStr.length() == 0 || lonStr.length() == 0) {
    Serial.println("Datos GPS inválidos.");
    return false;
  }

  // Convertir latitud
  latitud = convertirCoordenada(latStr, ns);

  // Convertir longitud
  longitud = convertirCoordenada(lonStr, ew);

  return true;
}

float convertirCoordenada(String coord, String direccion) {
  int grados = (direccion == "N" || direccion == "S") ? coord.substring(0, 2).toInt() : coord.substring(0, 3).toInt();
  float minutos = coord.substring((direccion == "N" || direccion == "S") ? 2 : 3).toFloat();
  float decimal = grados + (minutos / 60.0);

  if (direccion == "S" || direccion == "W") {
    decimal = -decimal;
  }
  return decimal;
}

void enviarDatos(){
  String json = "{\"SIM\":\"" + imei + "\",\"Latitud\":" + String(latitud, 6) +
                ",\"Longitud\":" + String(longitud, 6) + ",\"RitmoCardiaco\":" + String(ritmoCardiaco) + "}";

  Serial.print("Datos JSON a enviar: ");
  Serial.println(json);

  // Enviar datos al servidor
  Serial.println("Enviando datos al servidor...");
  http.connectionKeepAlive();       // Mantener conexión activa
  http.beginRequest();
  http.post("/");
  http.sendHeader("Content-Type", "application/json");
  http.sendHeader("Content-Length", json.length());
  http.beginBody();
  http.print(json);
  http.endRequest();

  // Obtener respuesta del servidor
  int statusCode = http.responseStatusCode();
  String response = http.responseBody();

  if(statusCode!=200){
    parpadeoLed(ledHTTP, 1, 500);
  }else{
    parpadeoLed(ledHTTP, 2, 200);
  }
  
  Serial.print("Código de respuesta: ");
  Serial.println(statusCode);
  Serial.print("Respuesta del servidor: ");
  Serial.println(response);



  delay(200); 
}

// Leer la respuesta del SIM7600SA, ignorando líneas vacías
// Obtener el IMEI del SIM7600SA
bool obtenerIMEI() {
  enviarComandoAT("AT+GSN");  // Comando para obtener el IMEI
  String respuesta = leerRespuesta(5000);

  Serial.println("Respuesta IMEI");
  Serial.println(respuesta);

  respuesta.trim();  // Eliminar espacios en blanco
  if (respuesta.startsWith("OK")) {
    Serial.println("Error: no se recibió el IMEI correctamente.");
    return false;
  }

  imei = respuesta.substring(0, respuesta.indexOf('\n'));  // Tomar la primera línea como el IMEI
  imei.trim();

  if (imei.length() == 15) {
    Serial.println("IMEI extraído: " + imei);
    return true;
  } else {
    Serial.println("Error al extraer el IMEI.");
    return false;
  }
}

bool obtenerIMEI2() {
  enviarComandoAT("AT+GSN");  // Comando para obtener el IMEI
  String respuesta = leerRespuesta(5000);
  // Eliminar cualquier espacio o salto de línea adicional en la respuesta
  respuesta.trim();

  // Buscar el IMEI en la respuesta
  int imeiStart = respuesta.indexOf('\n') + 1;  // Buscar el primer salto de línea tras el comando AT+GSN

  if (imeiStart > 0) {
    imei = respuesta.substring(imeiStart, imeiStart + 15); // Extraer los primeros 15 caracteres para el IMEI
    Serial.println("IMEI extraído2: " + imei);
    if(imei.length()==15){
      return true;
    }
  }

  Serial.println("Error al extraer el IMEI2.");
  return false;
}


// Función para monitorear el ritmo cardíaco y calcular BPM
void monitorRitmoCardiaco() {
  int pulseValue = analogRead(pulsePin);

  // Detección de latido
  if (abs(pulseValue - lastPulseValue) > changeThreshold) {
    unsigned long currentMillisHeart = millis();

    if (currentMillisHeart - lastBeatTime > 600) {  // Evitar latidos falsos (600 ms)
      beatCount++;
      lastBeatTime = currentMillisHeart;
    }
  }

  lastPulseValue = pulseValue;

  // Calcular BPM cada 15 segundos
  if (millis() - previousMillisHeart >= intervaloEnvio) {
    previousMillisHeart = millis();
    ritmoCardiaco = beatCount * (60 / 15);  // Calcular BPM basado en los últimos 15 segundos
    beatCount = 0;
    
    Serial.print("Pulsaciones por minuto (BPM): ");
    Serial.println(ritmoCardiaco);
  }
}


int calcularRitmoCardiaco() {
  int beatCount = 0;
  unsigned long lastBeatTime = 0;

  for (unsigned long start = millis(); millis() - start < intervaloEnvio; ) {
    int pulseValue = analogRead(pulsePin);

    if (abs(pulseValue - lastPulseValue) > changeThreshold) {
      unsigned long currentMillis = millis();

      if (currentMillis - lastBeatTime > 600) {  // Evitar latidos falsos (600 ms)
        beatCount++;
        lastBeatTime = currentMillis;
      }
    }
    lastPulseValue = pulseValue;
    delay(20);
  }

  return beatCount * 4;  // Calcular BPM basado en 15 segundos
}




// Leer la respuesta del SIM7600SA, ignorando líneas vacías
String leerRespuesta(unsigned long timeout) {
  String respuesta = "";
  unsigned long startTime = millis();
  while (millis() - startTime < timeout) {
    while (SerialSIM7600.available()) {
      String linea = SerialSIM7600.readStringUntil('\n');
      linea.trim();  // Eliminar espacios en blanco
      if (linea.length() > 0) {
        respuesta += linea + "\n";
      }
    }
  }
  return respuesta;
}


// Función para mostrar la respuesta del módulo (para depuración)
void mostrarRespuesta() {
  while (SerialSIM7600.available()) {
    String respuesta = SerialSIM7600.readStringUntil('\n');
    respuesta.trim();
    Serial.println("Respuesta del SIM7600SA: " + respuesta);
  }
}

// Función para enviar un comando AT
void enviarComandoAT(const char* comando) {
  SerialSIM7600.println(comando);
  Serial.print("Enviado: ");
  Serial.println(comando);
}

void parpadeoLed(int led, int cantParpadeos, int duracion){
  for(int i=0; i<cantParpadeos;i++){
    digitalWrite(led, HIGH);
    delay(duracion);
    digitalWrite(led, LOW);
    delay(duracion);
  }
}










