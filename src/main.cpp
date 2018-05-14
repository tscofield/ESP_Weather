






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
#include <ESPAsyncWiFiManager.h>         //https://github.com/tzapu/WiFiManager

#include <Arduino.h>
#include "FS.h"
#if defined(ESP32)
  #include "SPIFFS.h"
#endif

#include <Time.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include "ESPAsyncTCP.h"
#include "ESPAsyncWebServer.h"
#include "SparkFun_Si7021_Breakout_Library.h"
#include <Wire.h>
#include <NTPClient.h>
#include <WiFiUdp.h>

WiFiUDP ntpUDP;

// By default 'time.nist.gov' is used with 60 seconds update interval and
// no offset
NTPClient timeClient(ntpUDP);


static const int RXPin = 14, TXPin = 12;
static const uint32_t GPSBaud = 4800;

// The TinyGPS++ object
TinyGPSPlus gps;

// The serial connection to the GPS device
SoftwareSerial ss(RXPin, TXPin);

float humidity = 0;
float tempf = 0;

int power = 5;
int GND = 16;
int sda = 4;
int scl = 0;

int timeUpdateInterval = 300000;
int timeUpdateMillisLast = 0;

//Create Instance of HTU21D or SI7021 temp and humidity sensor and MPL3115A2 barrometric sensor
Weather sensor;

//---------------------------------------------------------------
void getWeather() {
  // Measure Relative Humidity from the HTU21D or Si7021
  humidity = sensor.getRH();

  // Measure Temperature from the HTU21D or Si7021
  tempf = sensor.getTempF();
  // Temperature is measured every time RH is requested.
  // It is faster, therefore, to read it from previous RH
  // measurement with getTemp() instead with readTemp()

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


void setup(){
  Serial.begin(9600);

  //WiFiManager
  //Local intialization. Once its business is done, there is no need to keep it around
  AsyncWiFiManager wifiManager(&server,&dns);
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

    File f = SPIFFS.open("/formatComplete2.txt", "w");
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

  // test calling network config
  //server.on("/net", std::bind(&AsyncWiFiManager::handleRoot, this,std::placeholders::_1)).setFilter(ON_AP_FILTER);

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
    json += ",\"pres\":"+ String(pres);
    json += ",\"winddir\":"+ String(winddir);
    json += ",\"windspeed\":"+ String(windspeed);
    json += ",\"rain\":"+ String(rain);
    json += ",\"lux\":"+ String(lux);
    json += ",\"uptime\":" + String(millis()/1000);
    //  sprintf(temp, "Seconds since boot: %u", millis()/1000);

    json += "}";
    json += "]";
    request->send(200, "text/json", json);
    json = String();
  });


  // Simple Firmware Update Form
  server.on("/update", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(200, "text/html", "<form method='POST' action='/update' enctype='multipart/form-data'><input type='file' name='update'><input type='submit' value='Update'></form>");
  });
  server.on("/update", HTTP_POST, [](AsyncWebServerRequest *request){
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

  // attach filesystem root at URL /fs
  server.serveStatic("/fs", SPIFFS, "/");

  // Catch-All Handlers
  // Any request that can not find a Handler that canHandle it
  // ends in the callbacks below.
  server.onNotFound(onRequest);
  server.onFileUpload(onUpload);
  server.onRequestBody(onBody);

  server.begin();

  // setup power for temp sensor
  pinMode(power, OUTPUT);
  pinMode(GND, OUTPUT);

  digitalWrite(power, HIGH);
  digitalWrite(GND, LOW);

  Wire.begin(4,0); //4 as SDA and 0 as SCL

  //Initialize the I2C sensors and ping them
  sensor.begin();
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
  while (ss.available() > 0) {
    if (gps.encode(ss.read())) {

      Serial.print(F("Location: "));
      if (gps.location.isValid()) {
        Serial.print(gps.location.lat(), 6);
        Serial.print(F(","));
        Serial.print(gps.location.lng(), 6);
      } else {
        Serial.print(F("INVALID"));
      }
      Serial.println();

      if (gps.time.isValid() && gps.date.isValid()) {
        Serial.println("received time from GPS, updating local time");
        setTime(gps.time.hour(),gps.time.minute(),gps.time.second(),gps.date.day(),gps.date.month(),gps.date.year());
        Serial.print("epoch: ");
        Serial.print(now());
        Serial.println();
      }
    }
  }
}

void loop() {
  if(shouldReboot){
    Serial.println("Rebooting...");
    delay(100);
    ESP.restart();
  }
  getWeather();
  printInfo();
  delay(1000);

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

}
