
#include "BLEDevice.h"
//#include "BLEScan.h"
#include "wifi_details.h"
#include <ArduinoOTA.h>
#include <ESPmDNS.h>
#include "AsyncUDP.h"


#define RELAY_PIN          33//onboard relay
#define SKETCH_VERSION "1.0.33"

const int wdtTimeout = 10000;  //time in ms to trigger the watchdog
hw_timer_t *timerWDT = NULL;
AsyncUDP udp_server;
WiFiUDP udp_client;
String command_to_run;

void IRAM_ATTR resetModule() {
  ets_printf("Reboot, beacause no data from the server\n");
  esp_restart();
}


void setup() {
  Serial.begin(115200);
  Serial.println("Starting Arduino BLE Client application...");
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, LOW);

  // START WIFI INIT  
  // delete old config
  WiFi.disconnect(true);
  WiFi.onEvent(WiFiEvent);
  WiFi.begin(ssid, password);
  WiFi.setAutoReconnect(true);
  Serial.println("Wait for WiFi... ");
  
  //start Bonjour (mDNS) responder
  if (MDNS.begin("esp32")) {
    MDNS.setInstanceName("ESP32 Fountain Basement Board");
    Serial.println("mDNS responder started");
    MDNS.addService("control", "tcp", 1111);
  }
  else Serial.println("Error setting up MDNS responder!");


  ArduinoOTA
  .onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH)
      type = "sketch";
    else // U_SPIFFS
      type = "filesystem";
    // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
    Serial.println("Start updating " + type);
  })
  .onEnd([]() {
    Serial.println("\nEnd");
  })
  .onProgress([](unsigned int progress, unsigned int total) {
    //Serial.printf("Progress: %u%%\r", (progress / (total / 100))); led_status_green_blink();
  })
  .onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println("End Failed");
  });

  ArduinoOTA.begin();
  

  if(udp_server.listen(1111)) {
    Serial.print("UDP Listening on IP: ");
    Serial.println(WiFi.localIP());
    udp_server.onPacket([](AsyncUDPPacket packet) {
      Serial.print("UDP Packet Type: ");
      Serial.print(packet.isBroadcast()?"Broadcast":packet.isMulticast()?"Multicast":"Unicast");
      Serial.print(", From: ");
      Serial.print(packet.remoteIP());
      Serial.print(":");
      Serial.print(packet.remotePort());
      Serial.print(", To: ");
      Serial.print(packet.localIP());
      Serial.print(":");
      Serial.print(packet.localPort());
      Serial.print(", Length: ");
      Serial.print(packet.length());
      Serial.print(", Data: ");
      Serial.write(packet.data(), packet.length());
      Serial.println();
      command_to_run="";
      for (uint8_t i=0; i<packet.length();i++) command_to_run += (char)packet.data()[i];
      Serial.print("Command_to_run: ");
      Serial.println(command_to_run );
      udp_client.beginPacket(packet.remoteIP(), 1111);
      udp_client.printf("ACK");
      udp_client.endPacket();

      digitalWrite(LED_PIN, !digitalRead(LED_PIN));
      delay(100);
      digitalWrite(LED_PIN, !digitalRead(LED_PIN));
      if (command_to_run=="Enable Fountain") { 
        digitalWrite(RELAY_PIN, HIGH); 
        digitalWrite(LED_PIN, HIGH);
        timerWrite(timerWDT, 0);
      }
      else if (command_to_run=="Disable Fountain") { 
        digitalWrite(RELAY_PIN, LOW);
        digitalWrite(LED_PIN, LOW);
        timerWrite(timerWDT, 0);
      }
      else if (command_to_run=="PING WDT")   timerWrite(timerWDT, 0); //reset timer (feed watchdog);
    });
  }

  //setup WDT
  timerWDT = timerBegin(0, 80, true);                  //timer 0, div 80
  timerAttachInterrupt(timerWDT, &resetModule, true);  //attach callback
  timerAlarmWrite(timerWDT, wdtTimeout * 1000, false); //set time in us
  timerAlarmEnable(timerWDT);                          //enable interrupt


} // End of setup.


// This is the Arduino main loop function.
void loop() {
  ArduinoOTA.handle();
  delay(1000); // Delay a second between loops.
} // End of loop
