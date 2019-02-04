// BUG
// bad temp readings are given if not sensor is found

#if defined(ESP8266)
  #include <ESP8266WiFi.h>          //https://github.com/esp8266/Arduino
#else
  #include <WiFi.h>
#endif

//needed for library
#include <DNSServer.h>
//#include <ESPAsyncWebServer.h>
//#include <ESPAsyncWiFiManager.h>         //https://github.com/tzapu/WiFiManager

#include <Arduino.h>
#include "FS.h"
#if defined(ESP32)
  #include "SPIFFS.h"
#endif

#include <Time.h>
//#include <TinyGPS++.h>
#include <HardwareSerial.h>
//#include <SoftwareSerial.h>

#include <NMEAGPS.h>
//#include <GPSport.h>

#ifndef GPSport_h
#define GPSport_h

#define gpsPort Serial2
#define GPS_PORT_NAME "Serial2"
#define DEBUG_PORT Serial
#define RX_PIN 34
#define TX_PIN 35
#endif

#if defined(ESP32)
  #include "AsyncTCP.h"
#elif defined(ESP8266)
  #include "ESPAsyncTCP.h"
#endif

#include "ESPAsyncWebServer.h"
//#include "SparkFun_Si7021_Breakout_Library.h"
#include <NTPClient.h>
#include <WiFiUdp.h>

#include <Wire.h>
#include <SPI.h>
//#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

#include <GxEPD.h>

// select the display class to use, only one
//#include <GxGDEP015OC1/GxGDEP015OC1.cpp>    // 1.54" b/w
#include <GxGDEW0154Z04/GxGDEW0154Z04.cpp>  // 1.54" b/w/r
//#include <GxGDE0213B1/GxGDE0213B1.cpp>      // 2.13" b/w
//#include <GxGDEW0213Z16/GxGDEW0213Z16.cpp>  // 2.13" b/w/r
//#include <GxGDEH029A1/GxGDEH029A1.cpp>      // 2.9" b/w
//#include <GxGDEW029Z10/GxGDEW029Z10.cpp>    // 2.9" b/w/r
//#include <GxGDEW027C44/GxGDEW027C44.cpp>    // 2.7" b/w/r
//#include <GxGDEW042T2/GxGDEW042T2.cpp>      // 4.2" b/w
//#include <GxGDEW075T8/GxGDEW075T8.cpp>      // 7.5" b/w
//#include <GxGDEW075Z09/GxGDEW075Z09.cpp>    // 7.5" b/w/r

// uncomment next line for drawBitmap() test
#include GxEPD_BitmapExamples

// FreeFonts from Adafruit_GFX
//#include <Fonts/FreeMonoBold9pt7b.h>
#include <FreeMonoBold9pt7b-154.h>
#include <FreeMonoBold10pt7b-154.h>
#include <FreeMonoBold11pt7b-154.h>
#include <FreeMonoBold12pt7b-154.h>
//#include <Fonts/FreeMonoBold12pt7b.h>
//#include <FreeMonoBold18pt7b.h>
//#include <FreeMonoBold24pt7b.h>


#include <GxIO/GxIO_SPI/GxIO_SPI.cpp>
#include <GxIO/GxIO.cpp>

#if defined(ESP8266)

// generic/common.h
//static const uint8_t SS    = 15;
//static const uint8_t MOSI  = 13;
//static const uint8_t MISO  = 12;
//static const uint8_t SCK   = 14;
// pins_arduino.h
//static const uint8_t D8   = 15;
//static const uint8_t D7   = 13;
//static const uint8_t D6   = 12;
//static const uint8_t D5   = 14;

// GxIO_SPI(SPIClass& spi, int8_t cs, int8_t dc, int8_t rst = -1, int8_t bl = -1);
GxIO_Class io(SPI, SS, 0, 2); // arbitrary selection of D3(=0), D4(=2), selected for default of GxEPD_Class
// GxGDEP015OC1(GxIO& io, uint8_t rst = 2, uint8_t busy = 4);
GxEPD_Class display(io); // default selection of D4(=2), D2(=4)

#elif defined(ESP32)

// pins_arduino.h, e.g. LOLIN32
//static const uint8_t SS    = 5;
//static const uint8_t MOSI  = 23;
//static const uint8_t MISO  = 19;
//static const uint8_t SCK   = 18;

// GxIO_SPI(SPIClass& spi, int8_t cs, int8_t dc, int8_t rst = -1, int8_t bl = -1);
GxIO_Class io(SPI, SS, 17, 16); // arbitrary selection of 17, 16
// GxGDEP015OC1(GxIO& io, uint8_t rst = D4, uint8_t busy = D2);
GxEPD_Class display(io, 16, 4); // arbitrary selection of (16), 4

#elif defined(ARDUINO_ARCH_SAMD)

// variant.h of MKR1000
//#define PIN_SPI_MISO  (10u)
//#define PIN_SPI_MOSI  (8u)
//#define PIN_SPI_SCK   (9u)
//#define PIN_SPI_SS    (24u) // should be 4?
// variant.h of MKRZERO
//#define PIN_SPI_MISO  (10u)
//#define PIN_SPI_MOSI  (8u)
//#define PIN_SPI_SCK   (9u)
//#define PIN_SPI_SS    (4u)

GxIO_Class io(SPI, 4, 7, 6);
GxEPD_Class display(io, 6, 5);

#elif defined(_BOARD_GENERIC_STM32F103C_H_)

// STM32 Boards (STM32duino.com)
// Generic STM32F103C series
// aka BluePill
// board.h
//#define BOARD_SPI1_NSS_PIN        PA4
//#define BOARD_SPI1_MOSI_PIN       PA7
//#define BOARD_SPI1_MISO_PIN       PA6
//#define BOARD_SPI1_SCK_PIN        PA5
//enum {
//    PA0, PA1, PA2, PA3, PA4, PA5, PA6, PA7, PA8, PA9, PA10, PA11, PA12, PA13,PA14,PA15,
//  PB0, PB1, PB2, PB3, PB4, PB5, PB6, PB7, PB8, PB9, PB10, PB11, PB12, PB13,PB14,PB15,
//  PC13, PC14,PC15
//};
// variant.h
//static const uint8_t SS   = BOARD_SPI1_NSS_PIN;
//static const uint8_t SS1  = BOARD_SPI2_NSS_PIN;
//static const uint8_t MOSI = BOARD_SPI1_MOSI_PIN;
//static const uint8_t MISO = BOARD_SPI1_MISO_PIN;
//static const uint8_t SCK  = BOARD_SPI1_SCK_PIN;

// GxIO_SPI(SPIClass& spi, int8_t cs, int8_t dc, int8_t rst = -1, int8_t bl = -1);
GxIO_Class io(SPI, SS, 8, 9);
// GxGDEP015OC1(GxIO& io, uint8_t rst = 9, uint8_t busy = 7);
GxEPD_Class display(io, 9, 3);

#else

// pins_arduino.h, e.g. AVR
//#define PIN_SPI_SS    (10)
//#define PIN_SPI_MOSI  (11)
//#define PIN_SPI_MISO  (12)
//#define PIN_SPI_SCK   (13)

GxIO_Class io(SPI, SS, 8, 9); // arbitrary selection of 8, 9 selected for default of GxEPD_Class
//GxIO_DESTM32L io;
//GxIO_GreenSTM32F103V io;
GxEPD_Class display(io);

#endif

unsigned long delayTime;

WiFiUDP ntpUDP;

// By default 'time.nist.gov' is used with 60 seconds update interval and
// no offset
NTPClient timeClient(ntpUDP);

//static const int RXPin = 9, TXPin = 10;
//static const uint32_t GPSBaud = 9600;

// The TinyGPS++ object
//TinyGPSPlus gps;

// The serial connection to the GPS device
//SoftwareSerial ss(RXPin, TXPin);
//HardwareSerial ss;
//HardwareSerial ss(2);
//HardwareSerial Serial1(1);
NMEAGPS  gps; // This parses the GPS characters
gps_fix  fix; // This holds on to the latest valuesgps_fix  fix; // This holds on to the latest values



float humidity = 0;
float pressure = 0;
float tempf = 0;
float tempc = 0;

int power = 5;
int GND = 16;

#if defined(ESP8266)
  #define I2C_SDA 4
  #define I2C_SCL 0
  int sda = 4;
  int scl = 0;
#elif defined(ESP32)
  #define I2C_SDA 21
  #define I2C_SCL 22
  int sda = 21;
  int scl = 22;
#endif


#define BME280_ADD 0x76


#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_BME280 bme; // I2C


unsigned long timeUpdateInterval = 300000;
unsigned long timeUpdateMillisLast = 0;

//Create Instance of HTU21D or SI7021 temp and humidity sensor and MPL3115A2 barrometric sensor
//Weather sensor;


//---------------------------------------------------------------

void logWeather() {

  // wait until we have a valid time before we start logging
  if (now() != millis()/1000) {
    // open file for writing
    File f = SPIFFS.open("/data.txt", "a");
    if (!f) {
        Serial.println("file open failed");
    }
    f.print(now());
    f.print(",");
    f.print(tempf);
    f.print(",");

    f.print(humidity);
    f.println();
    f.close();
  } else {
    Serial.println("No valid time yet, not logging data");
  }
}

//---------------------------------------------------------------
void printInfo() {
  //This function prints the weather data out to the default Serial Port
  Serial.print(now());
  Serial.print(",");
  Serial.print("Temp:");
  Serial.print(tempf);
  Serial.print("F, ");

  Serial.print("Humidity:");
  Serial.print(humidity);
  Serial.println("%");
}

float pres = 0;
float winddir = 0;
float windspeed = 0;
float rain = 0;
float lux = 0;

AsyncWebServer server(80);
AsyncWebSocket ws("/ws"); // access at ws://[esp ip]/ws
AsyncEventSource events("/events"); // event source (Server-Sent events)

DNSServer dns;

const char* http_username = "admin";
const char* http_password = "admin";

//flag to use from web update to reboot the ESP
bool shouldReboot = false;

void onRequest(AsyncWebServerRequest *request){
  //Handle Unknown Request
  request->send(404);
}

void onBody(AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total){
  //Handle body
}

void onUpload(AsyncWebServerRequest *request, String filename, size_t index, uint8_t *data, size_t len, bool final){
  //Handle upload
}

void onEvent(AsyncWebSocket * server, AsyncWebSocketClient * client, AwsEventType type, void * arg, uint8_t *data, size_t len){
  //Handle WebSocket event
}

void setupAP() {
  WiFi.softAP("testAP");
  IPAddress myIP = WiFi.softAPIP();
	Serial.print("AP IP address: ");
	Serial.println(myIP);
}

void updateEinkDisplay() {
  //temp
  //feels like
  //humidity
  //rainfall
  //

  display.fillScreen(GxEPD_WHITE);
  display.setTextColor(GxEPD_BLACK);
  display.setCursor(0, 0);

  const GFXfont* f = &FreeMonoBold12pt7b;
  display.setFont(f);
  display.println();

  const GFXfont* f12 = &FreeMonoBold12pt7b;
  display.setFont(f12);

  display.print("Weather v0.01");
  display.println();

  display.print("Tmp: ");
  display.print(tempf);
  display.println();

  display.print("Hum: ");
  display.print(humidity);
  display.println();

  // divide pressure by 1013 to display ATM (Atmospheres)
  display.print("ATM: ");
  display.print(pressure/1013);
  display.println();
  display.print("Wind Speed: ");
  //display.print(windspeed);
  display.println();
  display.print("Wind Dir: ");
  //display.print(winddirection);
  display.println();
  display.print("Rain: ");
  //display.print(rain);
  display.println();
  display.update();
}

/*
void setupGPS() {
  // set pin modes for gps Serial
  pinMode(RXPin, INPUT);
  pinMode(TXPin, OUTPUT);
  ss.begin(GPSBaud, SERIAL_8N1, RXPin, TXPin);
//  ss.begin(GPSBaud);
  // set baud yo 9600
  ss.print("\xb5\x62\x06\x00\x14\x00\x01\x00\x00\x00\xd0\x08\x00\x00\x80\x25\x00\x00\x07\x00\x07\x00\x00\x00\x00\x00\xa6\xcd\xb5\x62\x06\x00\x01\x00\x01\x08\x22");
}
*/

void setup(){
  Serial.begin(9600);
//  gpsPort.begin(9600,SERIAL_8N1,9,10);
  gpsPort.begin(9600,SERIAL_8N1,34,35);

  display.init();

  //setupGPS();

  Serial.println(F("BME280 test"));
  bool status;
  status = bme.begin(BME280_ADD);
//  status = bme.begin();
  if (!status) {
      Serial.println("Could not find a valid BME280 sensor, check wiring!");
      while (1);
  }

  Serial.println("-- Default Test --");
  Serial.println("normal mode, 16x oversampling for all, filter off,");
  Serial.println("0.5ms standby period");
  delayTime = 5000;

  Serial.println("");

  //WiFiManager
  //Local intialization. Once its business is done, there is no need to keep it around
  //AsyncWiFiManager wifiManager(&server,&dns);
  //reset saved settings
  //wifiManager.resetSettings();
  //set custom ip for portal
  //wifiManager.setAPConfig(IPAddress(10,0,1,1), IPAddress(10,0,1,1), IPAddress(255,255,255,0));
  //fetches ssid and pass from eeprom and tries to connect
  //if it does not connect it starts an access point with the specified name
  //here  "AutoConnectAP"
  //and goes into a blocking loop awaiting configuration
  //wifiManager.autoConnect("AutoConnectAP");
  //or use this for auto generated name ESP + ChipID
  //wifiManager.autoConnect();
  //if you get here you have connected to the WiFi
  //Serial.println("connected...yeey :)");

//  WiFi.mode(WIFI_STA);
//  WiFi.begin(ssid, password);

  if (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.printf("WiFi Failed!\n");
    setupAP();
  } else {
    Serial.printf("WiFi Setup\n");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());
  }

  timeClient.begin();

  delay(10000);

  SPIFFS.begin();
  if (!SPIFFS.exists("/formatComplete.txt")) {
    Serial.println("Please wait 30 secs for SPIFFS to be formatted");
    SPIFFS.format();
    Serial.println("Spiffs formatted");

    File f = SPIFFS.open("/formatComplete.txt", "w");
    if (!f) {
        Serial.println("file open failed");
    } else {
        f.println("Format Complete");
    }
  } else {
    Serial.println("SPIFFS is formatted. Moving along...");
  }

  // attach AsyncWebSocket
  ws.onEvent(onEvent);
  server.addHandler(&ws);

  // attach AsyncEventSource
  server.addHandler(&events);

  // respond to GET requests on URL /heap
  server.on("/heap", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(200, "text/plain", String(ESP.getFreeHeap()));
  });

  // upload a file to /upload
  server.on("/upload", HTTP_POST, [](AsyncWebServerRequest *request){
    request->send(200);
  }, onUpload);

  // send a file when /index is requested
  server.on("/index", HTTP_ANY, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, "/index.htm");
  });

  server.on("/data", HTTP_ANY, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, "/data.txt");
  });

  // HTTP basic authentication
  server.on("/login", HTTP_GET, [](AsyncWebServerRequest *request){
    if(!request->authenticate(http_username, http_password))
        return request->requestAuthentication();
    request->send(200, "text/plain", "Login Success!");
  });

  //First request will return 0 results unless you start scan from somewhere else (loop/setup)
  //Do not request more often than 3-5 seconds
  server.on("/json", HTTP_GET, [](AsyncWebServerRequest *request){
    String json = "[";
    json += "{";
    json += "\"tempf\":"+ String(tempf);
    json += ",\"humidity\":"+ String(humidity);
    json += ",\"pres\":"+ String(pressure);
    json += ",\"winddir\":"+ String(winddir);
    json += ",\"windspeed\":"+ String(windspeed);
    json += ",\"rain\":"+ String(rain);
    json += ",\"lux\":"+ String(lux);
    json += ",\"uptime\":" + String(millis()/1000);
    //  sprintf(temp, "Seconds since boot: %u", millis()/1000);
    if (fix.valid.location) {
      json += ",\"latitude\":" + String(fix.latitude());
      json += ",\"longitude\":" + String(fix.longitude());
    }

    json += "}";
    json += "]";
    request->send(200, "text/json", json);
    json = String();
  });


//  // Simple Firmware Update Form
//  server.on("/update", HTTP_GET, [](AsyncWebServerRequest *request){
//    request->send(200, "text/html", "<form method='POST' action='/update' enctype='multipart/form-data'><input type='file' name='update'><input type='submit' value='Update'></form>");
//  });

/*  server.on("/update", HTTP_POST, [](AsyncWebServerRequest *request){
    shouldReboot = !Update.hasError();
    AsyncWebServerResponse *response = request->beginResponse(200, "text/plain", shouldReboot?"OK":"FAIL");
    response->addHeader("Connection", "close");
    request->send(response);
  },[](AsyncWebServerRequest *request, String filename, size_t index, uint8_t *data, size_t len, bool final){
    if(!index){
      Serial.printf("Update Start: %s\n", filename.c_str());
      Update.runAsync(true);
      if(!Update.begin((ESP.getFreeSketchSpace() - 0x1000) & 0xFFFFF000)){
        Update.printError(Serial);
      }
    }
    if(!Update.hasError()){
      if(Update.write(data, len) != len){
        Update.printError(Serial);
      }
    }
    if(final){
      if(Update.end(true)){
        Serial.printf("Update Success: %uB\n", index+len);
      } else {
        Update.printError(Serial);
      }
    }
  });
*/
  // attach filesystem root at URL /fs
  server.serveStatic("/fs", SPIFFS, "/");

  // Catch-All Handlers
  // Any request that can not find a Handler that canHandle it
  // ends in the callbacks below.
  server.onNotFound(onRequest);
  server.onFileUpload(onUpload);
  server.onRequestBody(onBody);

  server.begin();

}
void getNTPinfo() {
  // If we are in station mode STA then we can try and use NTP, otherwise skip
  // Serial.print("WiFi mode :");
  // Serial.println(WiFi.getMode());
  if (WiFi.getMode() == 1){
    Serial.println("in wifi client mode, get ntp data");

    timeClient.update();
    if (timeClient.getEpochTime() > 1000 ){
      // seems that it is a valid NTPtime
      Serial.print("NTP time updated : ");
      Serial.println(timeClient.getEpochTime());
      // set Arduino/ESP time to local time
      setTime(timeClient.getEpochTime());
    }
    else Serial.println("Update NTP time failed");
  }

}

void getGPSinfo() {
  Serial.println("DEBUG: checking for GPS");
  //while (gps.available( gpsPort )) {
    Serial.println("DEBUG: retrieving GPS data");
    fix = gps.read();

    Serial.print( F("Location: ") );
    if (fix.valid.location) {
      Serial.print( fix.latitude(), 6 );
      Serial.print( ',' );
      Serial.print( fix.longitude(), 6 );
    }

    Serial.print( F(", Altitude: ") );
    if (fix.valid.altitude)
      Serial.print( fix.altitude() );

    Serial.println();
  //}
    //  Serial.println();
//      if (gps.time.isValid() && gps.date.isValid()) {
//        Serial.println("received time from GPS, updating local time");
//        setTime(gps.time.hour(),gps.time.minute(),gps.time.second(),gps.date.day(),gps.date.month(),gps.date.year());
//        Serial.print("epoch: ");
//        Serial.print(now());
//        Serial.println();
//      }

//  }
}


void printValues() {
    Serial.print("Temperature = ");
    tempc = bme.readTemperature();
    tempf = tempc * 9.0 / 5.0 + 32.0;
    Serial.print(tempc);
    Serial.println(" *C");

    Serial.print("Pressure = ");
    pressure = bme.readPressure() / 100.0F;
    Serial.print(pressure);
    Serial.println(" hPa");

    Serial.print("Approx. Altitude = ");
    Serial.print(bme.readAltitude(SEALEVELPRESSURE_HPA));
    Serial.println(" m");

    Serial.print("Humidity = ");
    humidity = bme.readHumidity();
    Serial.print(humidity);
    Serial.println(" %");

    Serial.println();
}

void loop() {
  if(shouldReboot){
    Serial.println("Rebooting...");
    delay(100);
    ESP.restart();
  }

  // Only needed in forced mode! In normal mode, you can remove the next line.
  bme.takeForcedMeasurement(); // has no effect in normal mode

  printValues();
  delay(delayTime);


  static char temp[128];
  sprintf(temp, "Seconds since boot: %u", millis()/1000);
  events.send(temp, "time"); //send event "time"

  // we have never synced time
  if (now() == millis()/1000) {
    Serial.println("attempting initial time sync");
    getGPSinfo();
    getNTPinfo();
//  } else {
//    Serial.print("Current time: ");
//    Serial.println(now());
  }

  // Only check for time updates periodically
  if ((timeUpdateInterval + timeUpdateMillisLast) < millis()) {
    Serial.print("time update millis last: ");
    Serial.println(timeUpdateMillisLast);
    getGPSinfo();
    getNTPinfo();

    timeUpdateMillisLast = millis();
  }

  updateEinkDisplay();
  logWeather();
}
