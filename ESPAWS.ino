#include "SPIFFS.h"
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <Wire.h>
#include <PubSubClient.h>
#include <LiquidCrystal_I2C.h>  // Biblioteca para manejar LCD con I2C
#include <HTTPClient.h>
#include <ArduinoJson.h>  // Biblioteca para manejar JSON

//Credenciales de red Wifi
//const char* ssid = "<SSID de la red>";
//const char* password = "<Contraseña>"; 
const int valvePin = 26;

//Servidor MQTT
const char* mqtt_server = "servidor MQTT proporcionado por Amazon AWS IOT Core";
const int mqtt_port = 8883;

String Read_rootca;
String Read_cert;
String Read_privatekey;

// Configuración para LCD I2C
LiquidCrystal_I2C lcd(0x27, 16, 2);  // Dirección I2C y tamaño de la pantalla (16x2)

//********************************
#define BUFFER_LEN  256
long lastMsg = 0;
char msg[BUFFER_LEN];
int value = 0;
byte mac[6];
char mac_Id[18];
int count = 1;

// Configuración del caudalímetro
const int flowSensorPin = 27;
volatile int pulseCount = 0;
float flowRate = 0.0;
float totalLitres = 0.0;  // Agua total servida
float totalWater = -1;  // Total de agua por servir (50 litros)
unsigned long oldTime = 0;
int timeElapsed = 0;

bool sent = false;
//Configuración de cliente MQTT
WiFiClientSecure espClient;
PubSubClient client(espClient);

// Función para contar pulsos del sensor de flujo
void IRAM_ATTR pulseCounter() {
  pulseCount++;
}

//Conectar a red Wifi
void setup_wifi() {
  delay(10);
  Serial.println();
  Serial.print("Conectando.. ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi conectado");
  Serial.println("Direccion IP: ");
  Serial.println(WiFi.localIP());
}

//Callback
void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Mensaje recibido [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();
}

//Conectar a broker MQTT
void reconnect() {
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    String clientId = "ESP32-";
    clientId += String(random(0xffff), HEX);

    if (client.connect(clientId.c_str())) {
      Serial.println("Conectada");
      client.publish("ei_out", "hello world");
      client.subscribe("ei_in");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" Esperando 5 segundos");
      delay(5000);
    }
  }
}

// Hacer una solicitud GET y obtener el valor de totalLitres
int makeGetRequest() {
  int requestLitres = -1;
  HTTPClient http;
  //Esta es una URL de prueba para hacer una peticion GET y obtener la data proveniente de algun servicio y comprobar si se debe realizar un nuevo ciclo
  http.begin("https://example.mockapi.io/");  // URL de la solicitud GET
  int httpCode = http.GET();

  if (httpCode > 0) {
    String payload = http.getString();
    Serial.println("Respuesta GET: ");
    Serial.println(payload);

    // Procesar el JSON recibido
    DynamicJsonDocument doc(1024);
    DeserializationError error = deserializeJson(doc, payload);

    if (error) {
      Serial.print("Error al deserializar JSON: ");
      Serial.println(error.f_str());
      return -1;
    }

    // Obtener el primer objeto del arreglo
    JsonArray array = doc.as<JsonArray>();
    if (array.size() > 0) {
      JsonObject firstObject = array[0];
      requestLitres = firstObject["totalLitres"];
      Serial.println("Total Litros: ");
      Serial.print(requestLitres);
    } else {
      Serial.println("El arreglo está vacío.");
    }
  } else {
    Serial.println("Error en la solicitud GET");
  }

  http.end();
  return requestLitres;
}

// Hacer una solicitud POST al completar el proceso
void makePostRequest() {
  HTTPClient http;
  http.begin("<URL donde se almacenaran los datos>");  // URL de la solicitud POST
  http.addHeader("Content-Type", "application/json");

  // Crear el JSON a enviar
  DynamicJsonDocument doc(1024);
  doc["totalLitres"] = totalLitres;
  doc["elapsedTime"] = timeElapsed;

  String jsonStr;
  serializeJson(doc, jsonStr);

  // Enviar la solicitud POST
  int httpCode = http.POST(jsonStr);

  if (httpCode > 0) {
    String payload = http.getString();
    Serial.println("Respuesta POST: ");
    Serial.println(payload);
  } else {
    Serial.println("Error en la solicitud POST");
  }

  http.end();
}

void setup() {
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  pinMode(valvePin, OUTPUT);
  lcd.begin();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Iniciando...");
  
  pinMode(2, OUTPUT);
  setup_wifi();
  delay(1000);

  // Inicializar pantalla LCD
  

  // Inicializa SPIFFS y carga certificados
  if (!SPIFFS.begin(true)) {
    Serial.println("Error al montar SPIFFS");
    return;
  }

  // Leer archivos de certificados
  File file2 = SPIFFS.open("/AmazonRootCA1.pem", "r");
  if (!file2) {
    Serial.println("No se pudo abrir el archivo Root CA");
    return;
  }
  while (file2.available()) {
    Read_rootca = file2.readString();
  }
  //Archivos proporcionados por AWS IoT Core
  File file4 = SPIFFS.open("/53-certificate.pem.crt", "r");
  if (!file4) {
    Serial.println("No se pudo abrir el archivo Cert");
    return;
  }
  while (file4.available()) {
    Read_cert = file4.readString();
  }

  File file6 = SPIFFS.open("/53-private.pem.key", "r");
  if (!file6) {
    Serial.println("No se pudo abrir el archivo privateKey");
    return;
  }
  while (file6.available()) {
    Read_privatekey = file6.readString();
  }

  // Configurar certificados para el cliente MQTT
  espClient.setCACert(Read_rootca.c_str());
  espClient.setCertificate(Read_cert.c_str());
  espClient.setPrivateKey(Read_privatekey.c_str());

  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);

  // Inicializar el sensor de flujo
  pinMode(flowSensorPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(flowSensorPin), pulseCounter, FALLING);

  oldTime = millis();
  Serial.println("Finalizo el setup");
}

void loop() {
  totalWater = makeGetRequest();
  timeElapsed = 0;
  lcd.clear();
  lcd.setCursor(0, 0);
  oldTime = millis();
  totalLitres = 0;
  lcd.print("Buscando...");
  while(totalWater != -1) {
    digitalWrite(valvePin, HIGH);
    if ((millis() - oldTime) > 1000) {  // Cada segundo
    
      timeElapsed++;
      detachInterrupt(digitalPinToInterrupt(flowSensorPin));
      flowRate = (((1000.0 / (millis() - oldTime)) * pulseCount) / 6.4)+3;  // Ajustar el factor de conversión
      totalLitres += flowRate / 60;  // Convertir caudal a litros por segundo
      oldTime = millis();
      pulseCount = 0;
      attachInterrupt(digitalPinToInterrupt(flowSensorPin), pulseCounter, FALLING);
  
      // Calcular porcentaje y mostrar en LCD
      float percentage = (totalLitres / totalWater) * 100.0;
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print((int)percentage);
      lcd.print("%");
      lcd.setCursor(10, 0);
      lcd.print("ET:");
      lcd.print((int) timeElapsed);
      lcd.setCursor(0, 1);
      lcd.print(totalLitres);
      lcd.print("L");
      
      // Verificar si se alcanzó el total de agua
      if (totalLitres >= totalWater) {
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Listo! ET:");
        lcd.print(timeElapsed);
        lcd.print("s");
        
        lcd.setCursor(0, 1);
        lcd.print("Total:");
        lcd.print(totalLitres);
        lcd.print("L");
        totalWater = -1;
        totalLitres = -2;
        // Hacer la solicitud POST cuando el proceso termina
        if(!sent) {
          Serial.println("Finalizado");
          makePostRequest();
          digitalWrite(valvePin, LOW);
          sent = true;
        }
      }
    }
  
    // Enviar datos a MQTT
    if (!client.connected()) {
      reconnect();
    }
    client.loop();
  
    long now = millis();
    if (now - lastMsg > 5000) {
      lastMsg = now;
      String macIdStr = mac_Id;
      String FlowRate = String(flowRate, 2);
      snprintf(msg, BUFFER_LEN, "{\"mac_Id\" : \"%s\", \"FlowRate\" : %s}", macIdStr.c_str(), FlowRate.c_str());
      client.publish("water", msg);
      count++;
    }
  
    digitalWrite(2, HIGH);
    digitalWrite(2, LOW);
  }
}
