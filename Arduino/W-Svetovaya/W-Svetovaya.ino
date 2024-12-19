#include "rpcWiFi.h" //Wi-Fi library /////////GITHUB///////
#include "TFT_eSPI.h" //TFT LCD library 
#include "Free_Fonts.h" //free fonts library 
#include <SoftwareSerial.h>;
#include "DHT.h"
#include <OneWire.h>                
#include <DallasTemperature.h>
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"
#include <Arduino.h>
#include <Wire.h>
#include "SHT31.h"

SHT31 sht31 = SHT31();
OneWire ourWire(D3);          // 18 
#define sensorAgua1 D2
#define sensorAgua2 D3 
#define sensorAgua3 D4 
#define sensorAgua4 D5   
float RX = BCM14;             // 8
float TX = BCM15;             // 10      

//declarated valors
float temp1,temp2 = 0;
float lastTemperature1,lastTemperature2,avgTemp;
float co2 = 0;
float Agua_1,Agua_2,Agua_3,Agua_4,Agua_5;
float t3,h1 = 0;
float lastValidTemperature = 0;  // Variable para almacenar la última temperatura válida
float lastValidHumidity = 0;

int ndispositivos = 0;
float tempC;

unsigned long previousMillis = 0;
unsigned long currentMillis = 0;
unsigned long interval = 20000;
unsigned long interval2 = 60000;
unsigned long interval3 = 60000;

unsigned long prevTime_T1 = millis();
unsigned long prevTime_T2 = millis();
unsigned long prevTime_T3 = millis(); 

TFT_eSPI tft; //initialize TFT LCD
TFT_eSprite spr = TFT_eSprite(&tft); //initialize sprite buffer

SoftwareSerial myCo2(RX, TX); // RX=D10-io5-naranja, TX=D11-io23-cafe
DallasTemperature ds18(&ourWire); //Se declara una variable u objeto para nuestro sensor
DeviceAddress address1 = { 0x28, 0xE5, 0x85, 0x67, 0xD, 0x0, 0x0, 0xC3 }; //T-1
DeviceAddress address2 = { 0x28, 0x2F, 0xC0, 0xB5, 0xD, 0x0, 0x0, 0xAE }; //T-3
byte request[9] = {0xFF,0x01,0x86,0x00,0x00,0x00,0x00,0x00,0x79}; 
unsigned char response[9];
// Configuraciones del sistema----------------------
#define WLAN_SSID       "Fresh" 
#define WLAN_PASS       "OKM690wsx" // OKM690wsx
#define MQTT_SERVER      "agriru.ru" 
#define MQTT_SERVERPORT  1883
#define MQTT_USERNAME    ""
#define MQTT_KEY         ""
#define mqtt_co2      "DarMal/StasovoySvetovaya/L2C0"
#define mqtt_hum_0    "DarMal/StasovoySvetovaya/L2H0"
#define mqtt_hum_1    "DarMal/StasovoySvetovaya/L2H1"
#define mqtt_hum_2    "DarMal/StasovoySvetovaya/L2H2"
#define mqtt_hum_3    "DarMal/StasovoySvetovaya/L2H3"
#define mqtt_hum_4    "DarMal/StasovoySvetovaya/L2H4"
#define mqtt_temp0    "DarMal/StasovoySvetovaya/L2T0"
#define mqtt_temp1    "DarMal/StasovoySvetovaya/L2T1" //T-1
#define mqtt_temp2    "DarMal/StasovoySvetovaya/L2T2" //T-3
#define mqtt_temp3    "DarMal/StasovoySvetovaya/L2T3"  //T-PT100
#define temp_max      "DarMal/StasovoySvetovaya-temperature/max" 
#define temp_min      "DarMal/StasovoySvetovaya-temperature/min" 
#define hum_max       "DarMal/StasovoySvetovaya-humidity/max" 
#define hum_min       "DarMal/StasovoySvetovaya-humidity/min" 
#define co2_max       "DarMal/StasovoySvetovaya-co2/max" 
#define co2_min       "DarMal/StasovoySvetovaya-co2/min" 

// conexion y rutamiento de topics para los sensores--------
WiFiClient client;
Adafruit_MQTT_Client mqtt(&client, MQTT_SERVER, MQTT_SERVERPORT, MQTT_USERNAME, MQTT_USERNAME, MQTT_KEY);
Adafruit_MQTT_Publish co2_z19 = Adafruit_MQTT_Publish(&mqtt, mqtt_co2);
Adafruit_MQTT_Publish hum_0 = Adafruit_MQTT_Publish(&mqtt, mqtt_hum_0);
Adafruit_MQTT_Publish hum_1 = Adafruit_MQTT_Publish(&mqtt, mqtt_hum_1);
Adafruit_MQTT_Publish hum_2 = Adafruit_MQTT_Publish(&mqtt, mqtt_hum_2);
Adafruit_MQTT_Publish hum_3 = Adafruit_MQTT_Publish(&mqtt, mqtt_hum_3);
Adafruit_MQTT_Publish hum_4 = Adafruit_MQTT_Publish(&mqtt, mqtt_hum_4);
Adafruit_MQTT_Publish temp_0 = Adafruit_MQTT_Publish(&mqtt, mqtt_temp0);
Adafruit_MQTT_Publish temp_1 = Adafruit_MQTT_Publish(&mqtt, mqtt_temp1);
Adafruit_MQTT_Publish temp_2 = Adafruit_MQTT_Publish(&mqtt, mqtt_temp2);
Adafruit_MQTT_Publish temp_3 = Adafruit_MQTT_Publish(&mqtt, mqtt_temp3);
//-----------------------------------------------------------------------------------------------
void setup() {
    Serial.begin(115200);
    Serial.println("*******Dary Malinovky*******");
    connectWiFi();
    Serial.print("RRSI: ");
    Serial.println(WiFi.RSSI());
    connectMQTT();
    sht31.begin();
    myCo2.begin(9600);
    initLCD(); 
} 
void initLCD() {
  tft.begin();
  tft.setRotation(3);
  tft.fillScreen(TFT_BLACK);
  tft.fillRect(0, 0, 320, 50, TFT_GREEN);
  
  tft.setFreeFont(FSB12);
  tft.setTextColor(TFT_BLACK);
  tft.drawString("******   ESVETOVAYA   ******", 10, 20);
  
  tft.setTextColor(TFT_WHITE);
  tft.setFreeFont(FS12);
  tft.drawString("Humidity     :", 10, 51);
  tft.drawString("Temperature 1:", 10, 78);
  tft.drawString("Temperature 2:", 10, 105);
  tft.drawString("Temperature 3:", 10, 136);
  tft.drawString("Average temp :", 10, 167);
  tft.drawString("Co2 :", 10, 200);
  
  tft.setTextColor(TFT_GREEN);
  tft.setFreeFont(FMB12);
  tft.drawString("%", 235, 51);
  tft.drawString("C", 235, 78);
  tft.drawString("C", 235, 105);
  tft.drawString("C", 235, 136);
  tft.drawString("C", 235, 167);
  tft.drawString("PPM", 235, 200);
}
void showValueWIOSprite(int x, int y, float value) {
  spr.createSprite(55, 40);
  spr.fillSprite(TFT_BLACK);
  spr.setFreeFont(FMB12);
  spr.setTextColor(TFT_WHITE);
  spr.drawNumber(value, 0, 0, 4);
  spr.pushSprite(x, y);
  spr.deleteSprite();
}
void loop() {
  unsigned long currentTime = millis();
  if (currentTime - prevTime_T1 > 60000) {
    leerco2();
    leersht31();
    leerDS18();
    leerAgua1();
    leerAgua2();
    leerAgua3();
    leerAgua4();
    reconnect();  
    prevTime_T1 = currentTime;
    }
  if (currentTime - prevTime_T2 > 300000) {//cada 20 minutos
    sendDataMqtt();
    prevTime_T2 = currentTime;
    }
  checkWifi();
  reconnect();
}
void sendDataMqtt() {
  co2_z19.publish(co2); //  Serial.write((byte) 0x00);
  hum_0.publish(h1);
  hum_1.publish(Agua_1); 
  temp_0.publish(avgTemp);
  temp_1.publish(lastTemperature1);
  temp_2.publish(lastTemperature2);
  temp_3.publish(t3);
}
void leerAgua1() {
    int lecturaSensor_1 = analogRead(sensorAgua1);

    if (lecturaSensor_1 < 10) { // Detectar desconexión
        Serial.println("Sensor Suelo1: Desconectado");
        Agua_1 = 100; // Suelo seco como valor seguro
    } else {
        Agua_1 = map(lecturaSensor_1, 1023, 0, 0, 100);
        Serial.print("Humedad Suelo1: ");
        Serial.print(Agua_1);
        Serial.print("%  "); Serial.println(lecturaSensor_1);
    }
}
void leerAgua2() {
    int lecturaSensor_2 = analogRead(sensorAgua2);

    // Verificar si el sensor está desconectado (lecturas fuera de rango esperado)
    if (lecturaSensor_2 < 10 || lecturaSensor_2 > 1013) {
        // Sensor desconectado: enviar valor seguro
     
        Agua_2 = 99; // Suelo seco como valor seguro para evitar riego
        Serial.print("Sensor Suelo2: Desconectado ");Serial.println(Agua_2);
        Serial.println(lecturaSensor_2);
        
    } else {
        // Sensor conectado: mapear lectura a porcentaje
        Agua_2 = map(lecturaSensor_2, 1023, 0, 0, 100);
        Serial.print("Humedad Suelo2: ");
        Serial.print(Agua_2);
        Serial.println("%");
    }
}
void leerAgua3() {
    int lecturaSensor_3 = analogRead(sensorAgua3);

    // Verificar si el sensor está desconectado (lecturas fuera de rango esperado)
    if (lecturaSensor_3 < 10 || lecturaSensor_3 > 1013) {
        // Sensor desconectado: enviar valor seguro
        
        Agua_3 = 99; // Suelo seco como valor seguro para evitar riego
        Serial.print("Sensor Suelo3: Desconectado ");Serial.println(Agua_3);
        Serial.println(lecturaSensor_3);
        
    } else {
        // Sensor conectado: mapear lectura a porcentaje
        Agua_3 = map(lecturaSensor_3, 1023, 0, 0, 100);
        Serial.print("Humedad Suelo3: ");
        Serial.print(Agua_3);
        Serial.println("%");
    }
}
void leerAgua4() {
    int lecturaSensor_4 = analogRead(sensorAgua4);

    // Verificar si el sensor está desconectado (lecturas fuera de rango esperado)
    if (lecturaSensor_4 < 10 || lecturaSensor_4 > 1013) {
        // Sensor desconectado: enviar valor seguro
        Agua_4 = 99; // Suelo seco como valor seguro para evitar riego
        Serial.print("Sensor Suelo4: Desconectado ");Serial.println(Agua_4);
        Serial.println(lecturaSensor_4);
        
    } else {
        // Sensor conectado: mapear lectura a porcentaje
        Agua_4 = map(lecturaSensor_4, 1023, 0, 0, 100);
        Serial.print("Humedad Suelo4: ");
        Serial.print(Agua_4);
        Serial.println("%");
    }
}
void leersht31() {  
    float temp = sht31.getTemperature();  
    float hum = sht31.getHumidity(); 
  if (temp != 0 && !isnan(temp)) {
        t3 = temp;  // Almacena el valor válido en la variable t3
        lastValidTemperature = temp;  // Actualiza el último valor válido de temperatura
    }
    if (hum != 0 && !isnan(hum)) {
        h1 = hum;  
        lastValidHumidity = hum;
    }
  Serial.print("Temperatura K1T0: ");
  Serial.print(t3);
  Serial.println(" ºC.");
  Serial.print("Humedad K1H0: ");
  Serial.print(h1);
  Serial.println(" %.");
  showValueWIOSprite(170, 51, h1);
  showValueWIOSprite(170, 105, t3);
}
void leerDS18() {
  ds18.requestTemperatures();
  float sumTemps = 0.0;
  int countSensors = 0;
  temp1 = ds18.getTempC(address1);
  if (temp1 != 85.00 && temp1 != -127.00 && temp1 != 0) {
    sumTemps += temp1;
    countSensors++;
    lastTemperature1 = temp1;
  }
  temp2 = ds18.getTempC(address2);
  if (temp2 != 85.00 && temp2 != -127.00 && temp2 != 0) {
    sumTemps += temp2;
    countSensors++;
    lastTemperature2 = temp2;
  }
  if (countSensors > 0) {
    avgTemp = sumTemps / countSensors;  
    Serial.print("temp_ds12_1 = ");
    Serial.print(lastTemperature1);
    Serial.println(" C");
    Serial.print("temp_ds12_2 = ");
    Serial.print(lastTemperature2);
    Serial.println(" C");
    Serial.print("Average temperature DS18b20 : ");
    Serial.print(avgTemp);
    Serial.println(" C");
  }
  showValueWIOSprite(170, 78, lastTemperature1);
  showValueWIOSprite(170, 145, lastTemperature2);
  showValueWIOSprite(170, 173, avgTemp); 
}
void leerco2() { 
  myCo2.write(request, 9);
  myCo2.write((byte)0x00);

  memset(response, 0, 9);
  myCo2.readBytes(response, 9);
  int i;
  byte crc = 0;
  for (i = 1; i < 8; i++) crc+=response[i];
  crc = 255 - crc;
  crc++;
  if ( !(response[0] == 0xFF && response[1] == 0x86 && response[8] == crc) ) {
    Serial.println("CRC error");
  } else {
    unsigned int HLconcentration = (unsigned int) response[2];
    unsigned int LLconcentration = (unsigned int) response[3];
    co2 = (256*HLconcentration) + LLconcentration;
    Serial.println(co2);
    for (i = 0; i < 9; i++) {
      Serial.print("0x");
      Serial.print(response[i],HEX);
      Serial.print("  ");
    }
    Serial.println("  ");
  }
  showValueWIOSprite(170, 210, co2); 
}  
void connectWiFi() {
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    Serial.println("Connecting to WiFi..");
    tft.begin();
    tft.setRotation(3);
    tft.fillScreen(TFT_BLACK);
    tft.setFreeFont(FMB12);
    tft.setCursor((320 - tft.textWidth("Connecting to Wi-Fi.."))/2, 120);
    tft.print("Connecting to Wi-Fi..");
  Serial.println("Connecting to WiFi..");
    WiFi.begin(WLAN_SSID, WLAN_PASS);
    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
        attempts++;
        if (attempts > 240) {
         WiFi.disconnect();
         NVIC_SystemReset();
      }     
    }
    //tft.fillScreen(TFT_BLACK);
    //tft.setCursor((320 - tft.textWidth("Connected!"))/2, 120);
    //tft.print("Connected..!");
    Serial.println("Connected to the WiFi network");
    //Serial.print("IP Address: ");
    //Serial.println (WiFi.localIP()); // prints out the device's IP address
  
}
void checkWifi() {
  unsigned long currentMillis = millis();
  // if WiFi is down, try reconnecting every CHECK_WIFI_TIME seconds
  if ((WiFi.status() != WL_CONNECTED) && (currentMillis - previousMillis >=interval)) {
    Serial.print(millis());
    Serial.println("(void checkWifi) Reconnecting to WiFi...");
    WiFi.disconnect();
    NVIC_SystemReset();
    previousMillis = currentMillis;
  }
}
void connectMQTT() {
  if (mqtt.connected())
    return;
  Serial.print("Connecting to MQTT... ");
  int attempts2 = 0;
  while (mqtt.connect() != 0) {
       delay(500);
       Serial.print(".");
       attempts2++;
       mqtt.disconnect();
       if (attempts2 > 240) {
        NVIC_SystemReset();
      }
  }
  Serial.println("MQTT Connected!");
}
void reconnect() {
  int8_t ret;
  if (mqtt.connected()) {
    return;
  }
  Serial.print("test alive MQTT... ");
  uint8_t retries = 3;
  while ((ret = mqtt.connect()) != 0) { // connect will return 0 for connected
       Serial.println(mqtt.connectErrorString(ret));
       Serial.println("Retrying MQTT connection in 5 seconds...");
       mqtt.disconnect();
       delay(3000);  // wait 5 seconds
       retries--;
       if (retries == 0) {
         // basically die and wait for WDT to reset me
         NVIC_SystemReset();
       }
  }
}
