#include <Arduino.h>
#include <SoftwareSerial.h>
#include <LiquidCrystal_I2C.h>
#include <DHT.h>
#include <ArduinoJson.h>
#include <TimeLib.h>
#include <TinyGPS++.h>
#include <Adafruit_BMP085.h>
#include "../include/secrets.h"

#include <Wire.h>

LiquidCrystal_I2C lcd(0x27,16,4);     // Set the LCD address to 0x27 for a 16 chars and 2 line display

#define DHTPIN 5                      // Digital pin connected to the DHT sensor
#define DHTTYPE DHT22                 // KY-015 -> DHT11, AM2302 -> DHT22. WOKWI just has DHT22
DHT dht(DHTPIN, DHTTYPE);             // Set dht sensor pin and type

Adafruit_BMP085 bmp;                  // Set bmp sensor

// #define DEBUG_GPS                     // Sets fixed values defined in secrets.h; comment out to run GPS setup

#ifndef DEBUG_GPS
  #define GPSRX 12                      // Digital pin connected that will act as the Arduino RX, connected to the GPS module TX
  #define GPSTX 11                      // Digital pin connected that will act as the Arduino TX, connected to the GPS module RX

  SoftwareSerial GPSSerial(GPSRX, GPSTX);
  TinyGPSPlus GPS;
#endif

const char deviceID[] = "nano-001"; 
double latitude;
double longitude;

float humidity;
float temperature;
float pressure;

unsigned long measurement_millis {millis()};
const unsigned int measurement_interval {3000};

void clear_serial_read_buffer() {
  while (Serial.available() > 0) {
    Serial.read();
  }
}


void update_measurements(){
  humidity = dht.readHumidity();          // Default: %
  temperature = dht.readTemperature();    // Default: Celsius
  pressure = bmp.readPressure() / 1000;   // Default: Pascal (kPa when divided)
  measurement_millis = millis();
}


void display_measurements(){
  lcd.clear();                      
  lcd.setCursor(0,0);

  if (isnan(humidity) || isnan(temperature)) {
    lcd.print(F("DHT FAILURE"));
    return;
  }

  if (isnan(pressure)) {
    lcd.print(F("BMP FAILURE"));
    return;
  }

  lcd.print(F("H:"));
  lcd.print(humidity, 1);
  lcd.print(F("%"));
  lcd.print(F(" "));
  lcd.print(F("T:"));
  lcd.print(temperature, 1);
  lcd.print(F("C"));
  lcd.setCursor(0,1);
  lcd.print(F("P: "));
  lcd.print(pressure, 1);
  lcd.print(F("kPa"));
}


void send_measurements(String _request){
  unsigned long request_millis = millis();
  JsonDocument request_doc;
  deserializeJson(request_doc, _request);
  time_t request_timestamp = request_doc["time"];

  JsonDocument response_doc;
  response_doc["device"] = deviceID;
  response_doc["location"]["latitude"] = latitude;
  response_doc["location"]["longitude"] = longitude;
  response_doc["request_timestamp"] = request_timestamp;
  response_doc["measurement"]["humidity"] = humidity;
  response_doc["measurement"]["temperature"] = temperature;
  response_doc["measurement"]["pressure"] = pressure;
  response_doc["measurement"]["timestamp"] = request_timestamp - (request_millis - measurement_millis)/1000;
  serializeJson(response_doc, Serial);
  Serial.println();  
}


void setup() {
  lcd.init();
  lcd.backlight();

  if (WOKWI)
    lcd.print(F("SIMULATION"));
  else
    lcd.print(F("PHYSICAL DEVICE"));

  delay(2000);

  Serial.begin(115200);
  
  // GPS Setup

  #ifndef DEBUG_GPS
    lcd.clear();                      
    lcd.setCursor(0,0);
    lcd.print(F("CONNECTING GPS"));

    GPSSerial.begin(9600);

    while (!GPS.location.isValid()){
      while (GPSSerial.available() > 0){
        GPS.encode(GPSSerial.read());
        if (GPS.location.isValid()){
          break;
        }
      }
    }

    latitude = GPS.location.lat();
    longitude = GPS.location.lng();

    GPSSerial.end();
  #else
    latitude = SECRETS_LATITUDE;
    longitude = SECRETS_LONGITUDE;
  #endif

  lcd.clear();                      
  lcd.setCursor(0,0);
  lcd.print(F("Lat: "));
  lcd.print(latitude);

  lcd.setCursor(0,1);
  lcd.print(F("Lng: "));
  lcd.print(longitude);
  delay(2000);

  dht.begin();
  bmp.begin();
  clear_serial_read_buffer();           // Clear device read buffer just in case
}

void loop() {
  // clear_serial_read_buffer();
  unsigned long start_time = millis();

  while(millis() - start_time < measurement_interval){
    if(Serial.available() > 0){
      String request = Serial.readStringUntil('\n');
      send_measurements(request);
    }
  }
  update_measurements();
  display_measurements();
}