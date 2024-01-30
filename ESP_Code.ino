/*
   ____  __  __  ____  _  _  _____       ___  _____  ____  _  _
  (  _ \(  )(  )(_  _)( \( )(  _  )___  / __)(  _  )(_  _)( \( )
   )(_) ))(__)(  _)(_  )  (  )(_)((___)( (__  )(_)(  _)(_  )  (
  (____/(______)(____)(_)\_)(_____)     \___)(_____)(____)(_)\_)
  Official code for all ESP8266/32 boards            version 4.0
  Main .ino file

  The Duino-Coin Team & Community 2019-2024 © MIT Licensed
  https://duinocoin.com
  https://github.com/revoxhere/duino-coin

  If you don't know where to start, visit official website and navigate to
  the Getting Started page. Have fun mining!

  To edit the variables (username, WiFi settings, etc.) use the Settings.h tab!
*/

/* If optimizations cause problems, change them to -O0 (the default) */
#pragma GCC optimize("-Ofast")

/* If during compilation the line below causes a
  "fatal error: arduinoJson.h: No such file or directory"
  message to occur; it means that you do NOT have the
  ArduinoJSON library installed. To install it,
  go to the below link and follow the instructions:
  https://github.com/revoxhere/duino-coin/issues/832 */
#include <ArduinoJson.h>

#if defined(ESP8266)
    #include <ESP8266WiFi.h>
    #include <ESP8266mDNS.h>
    #include <ESP8266HTTPClient.h>
    #include <ESP8266WebServer.h>
#elif defined (ESP32)
    #include <ESPmDNS.h>
    #include <WiFi.h>
    #include <HTTPClient.h>
    #include <WebServer.h>

#elif defined(ARDUINO_ARCH_MBED_RP2040) || defined(ARDUINO_ARCH_RP2040)
    #include <WiFi.h>
    #include <HTTPClient.h>
//    #include "pico/stdlib.h"

#endif

#if defined (ESP8266) || defined(ESP32)
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <WiFiClient.h>
#include <Ticker.h>
#endif
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "MiningJob.h"
#include "Settings.h"


#if defined(WEB_DASHBOARD)
  #include "Dashboard.h"
#endif

// Auto adjust physical core count
// (ESP32-S2/C3 have 1 core, ESP32 has 2 cores, ESP8266 has 1 core)
#if defined(ESP8266)
    #define CORE 1
    typedef ESP8266WebServer WebServer;
#elif defined(CONFIG_FREERTOS_UNICORE)
    #define CORE 1
#elif defined(ARDUINO_ARCH_MBED_RP2040) || defined(ARDUINO_ARCH_RP2040)
    #define CORE 2      
// #include "pico/multicore.h" 
#else
    #define CORE 2
    // Install TridentTD_EasyFreeRTOS32 if you get an error
    #include <TridentTD_EasyFreeRTOS32.h>
#endif

#if defined(WEB_DASHBOARD)
    WebServer server(80);
#endif

namespace {
    MiningConfig *configuration = new MiningConfig(
        DUCO_USER,
        RIG_IDENTIFIER,
        MINER_KEY
    );
     // I couldt find any good RTOS lib's for RP2040, so we use these simple finctions for now.
     // To start core 1 make a setup1(), and loop1() function.
     // void rp2040.idleOtherCore();   will pause the other core
     // void rp2040.resumeOtherCore(); will resume the corew, where it left off
     // https://arduino-pico.readthedocs.io/en/latest/multicore.html
     #if defined(ESP32) && CORE == 2
      EasyMutex mutexClientData, mutexConnectToServer;
    #endif

    void UpdateHostPort(String input) {
        // Thanks @ricaun for the code
        DynamicJsonDocument doc(256);
        deserializeJson(doc, input);
        const char *name = doc["name"];

        configuration->host = doc["ip"].as<String>().c_str();
        configuration->port = doc["port"].as<int>();
        node_id = String(name);

        Serial.println("Poolpicker selected the best mining node: " + node_id);
    }

String httpGetString(String URL) {
    String payload = "";
    HTTPClient http;

    #if defined (ESP8266) || defined(ESP32)
    WiFiClientSecure client;
    client.setInsecure();
    if (http.begin(client, URL)) {
    #elif defined(ARDUINO_ARCH_MBED_RP2040) || defined(ARDUINO_ARCH_RP2040)
    http.setInsecure();  // Disable SSL certificate verification
    if (http.begin(URL)) {
    #else
    // Handle other platforms 
    #error "No ARCH defined in httpGetString()"
    #endif   

        int httpCode = http.GET();

        if (httpCode == HTTP_CODE_OK) {
            payload = http.getString();
            Serial.println("Received JSON data:");
            Serial.println(payload);
        } else {
            Serial.printf("Error fetching node from poolpicker: %s\n", http.errorToString(httpCode).c_str());
            Serial.println("HTTP error code: " + String(httpCode));
        }

        http.end();
    } else {
        // Handle the case when http.begin() fails, if necessary
        Serial.println("HTTP begin failed");
    }
    return payload;
}



    void SelectNode() {
        String input = "";
        int waitTime = 1;
        int poolIndex = 0;

        while (input == "") {
            Serial.println("Fetching mining node from the poolpicker in " + String(waitTime) + "s");
            input = httpGetString("https://server.duinocoin.com/getPool");
            
            delay(waitTime * 1000);
            // Increase wait time till a maximum of 32 seconds
            // (addresses: Limit connection requests on failure in ESP boards #1041)
            waitTime *= 2;
            if (waitTime > 32)
                  waitTime = 32;
        }

        UpdateHostPort(input);
      }

    void SetupWifi() {
        Serial.println("Connecting to: " + String(SSID));
        WiFi.mode(WIFI_STA); // Setup ESP in client mode
        #if defined(ESP8266)
            WiFi.setSleepMode(WIFI_NONE_SLEEP);
        #elif defined (ESP32)
            WiFi.setSleep(false);
        #elif defined(ARDUINO_ARCH_MBED_RP2040) || defined(ARDUINO_ARCH_RP2040)
            //  No wifi sleep for RP2040
        #else
        // Handle other platforms 
        #error "No ARCH defined in SetupWifi()"
        #endif   

        
        WiFi.begin(SSID, PASSWORD);

        int wait_passes = 0;
        while (WiFi.waitForConnectResult() != WL_CONNECTED) {
            delay(500);
            Serial.print(".");
            if (++wait_passes >= 10) {
                WiFi.begin(SSID, PASSWORD);
                wait_passes = 0;
            }
        }

        Serial.println("\n\nSuccessfully connected to WiFi");
        Serial.println("Local IP address: " + WiFi.localIP().toString());
        Serial.println("Rig name: " + String(RIG_IDENTIFIER));
        Serial.println();

        SelectNode();
    }
#if defined(ESP8266) || defined(ESP32)
    void SetupOTA() {
        // Prepare OTA handler
        ArduinoOTA.onStart([]()
                           { Serial.println("Start"); });
        ArduinoOTA.onEnd([]()
                         { Serial.println("\nEnd"); });
        ArduinoOTA.onProgress([](unsigned int progress, unsigned int total)
                              { Serial.printf("Progress: %u%%\r", (progress / (total / 100))); });
        ArduinoOTA.onError([](ota_error_t error)
                           {
        Serial.printf("Error[%u]: ", error);
        if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
        else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
        else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
        else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
        else if (error == OTA_END_ERROR) Serial.println("End Failed"); });

        ArduinoOTA.setHostname(RIG_IDENTIFIER); // Give port a name
        ArduinoOTA.begin();
    }
#endif    

void VerifyWifi() {
  if (WiFi.status() != WL_CONNECTED || WiFi.localIP() == IPAddress(0, 0, 0, 0)) {
    Serial.println("Reconnecting to WiFi...");

    #if defined(ARDUINO_ARCH_MBED_RP2040) || defined(ARDUINO_ARCH_RP2040)
    int attemptCounter = 0;
    while (WiFi.status() != WL_CONNECTED) {
      Serial.print("Attempting to reconnect to WiFi...");
      WiFi.begin(SSID, PASSWORD);
    
      while (WiFi.status() != WL_CONNECTED && attemptCounter < 10) {
        delay(500);
        Serial.print(".");
        attemptCounter++;
      }
    }
    #else
    // ESP8266 || ESP32 
    WiFi.reconnect();
    #endif
  }
}


    void handleSystemEvents(void) {
        VerifyWifi();
        #if defined (ESP8266)|| (ESP32)
        ArduinoOTA.handle();
        #endif
        yield();
    }

    #if defined(WEB_DASHBOARD)
        void dashboard() {
             Serial.println("Handling HTTP client");
             String s = WEBSITE;
             s.replace("@@IP_ADDR@@", WiFi.localIP().toString());
  
             s.replace("@@HASHRATE@@", String(hashrate / 1000));
             s.replace("@@DIFF@@", String(difficulty / 100));
             s.replace("@@SHARES@@", String(share_count));
             s.replace("@@NODE@@", String(node_id));
             
             #if defined(ESP8266)
                 s.replace("@@DEVICE@@", "ESP8266");
             #elif defined(CONFIG_FREERTOS_UNICORE)
                 s.replace("@@DEVICE@@", "ESP32-S2/C3");
             #else
                 s.replace("@@DEVICE@@", "ESP32");
             #endif
             
             s.replace("@@ID@@", String(RIG_IDENTIFIER));
             s.replace("@@MEMORY@@", String(ESP.getFreeHeap()));
             s.replace("@@VERSION@@", String(SOFTWARE_VERSION));
             server.send(200, "text/html", s);
        }
    #endif

} // End of namespace

MiningJob *job[CORE];

#if CORE == 2
  #if defined(ESP32)
  EasyFreeRTOS32 task1, task2;
  #elif defined(ARDUINO_ARCH_MBED_RP2040) || defined(ARDUINO_ARCH_RP2040)
  // Do nothing for now
  #endif
#endif   
  
# if defined (ESP32)
void task1_func(void *) {
    #if defined(ESP32) && CORE == 2
      VOID SETUP() { }

      VOID LOOP() {
        job[0]->mine();
      }
    #endif
}

void task2_func(void *) {
    #if defined(ESP32) && CORE == 2
      VOID SETUP() {
        job[1] = new MiningJob(1, configuration);
      }

      VOID LOOP() {
        job[1]->mine();
      }
    #endif
}
 #elif defined(ARDUINO_ARCH_MBED_RP2040) || defined(ARDUINO_ARCH_RP2040)

 void setup1() {job[1] = new MiningJob(1, configuration);}
 void loop1() { job[1]->mine();}

#endif 


void setup() {
#if defined(ARDUINO_ARCH_MBED_RP2040) || defined(ARDUINO_ARCH_RP2040)
    rp2040.idleOtherCore();  // pause core1
#endif    
    delay(500);
    Serial.begin(500000);
    Serial.println("\n\nDuino-Coin " + String(configuration->MINER_VER));

  
    pinMode(LED_BUILTIN, OUTPUT);

    assert(CORE == 1 || CORE == 2);
    WALLET_ID = String(random(0, 2811)); // Needed for miner grouping in the wallet
    job[0] = new MiningJob(0, configuration);

    #if defined(USE_DHT)
        Serial.println("Initializing DHT sensor (Duino IoT)");
        dht.begin();
        Serial.println("Test reading: " + String(dht.readHumidity()) + "% humidity");
        Serial.println("Test reading: temperature " + String(dht.readTemperature()) + "°C");
    #endif

    #if defined(USE_DS18B20)
        Serial.println("Initializing DS18B20 sensor (Duino IoT)");
        sensors.begin();
        sensors.requestTemperatures(); 
        Serial.println("Test reading: " + String(sensors.getTempCByIndex(0)) + "°C");
    #endif

    #if defined(USE_INTERNAL_SENSOR)
       Serial.println("Initializing internal ESP32 temperature sensor (Duino IoT)");
       temp_sensor_config_t temp_sensor = TSENS_CONFIG_DEFAULT();
       temp_sensor.dac_offset = TSENS_DAC_L2;
       temp_sensor_set_config(temp_sensor);
       temp_sensor_start();
       float result = 0;
       temp_sensor_read_celsius(&result);
       Serial.println("Test reading: " + String(result) + "°C");
    #endif

    SetupWifi();
#if defined (ESP8266) || (ESP32)
    SetupOTA();
#endif
    #if defined(WEB_DASHBOARD)
      if (!MDNS.begin(RIG_IDENTIFIER)) {
        Serial.println("mDNS unavailable");
      }
      MDNS.addService("http", "tcp", 80);
      Serial.println("Configured mDNS for dashboard on http://" + String(RIG_IDENTIFIER) 
                   + ".local (or http://" + WiFi.localIP().toString() + ")");
      server.on("/", dashboard);
      server.begin();
    #endif

    job[0]->blink(BLINK_SETUP_COMPLETE);

 #if CORE == 2
    #if defined(ESP32)  
      task1.start(task1_func);
      task2.start(task2_func);
    #elif defined(ARDUINO_ARCH_MBED_RP2040) || defined(ARDUINO_ARCH_RP2040) 
      rp2040.resumeOtherCore(); // run Forrest run! (release core 1)
    #else  
       // Handle other platforms 
    #error "No way to handle two cores for the given architecture"
  #endif
 #endif   

   
}

void loopOneCore() {
    job[0]->mine();

    #if defined(ESP8266)
        // Fastest clock mode for 8266s
        system_update_cpu_freq(160);
    #elif defined (ESP32)
        // Fastest clock mode for 32s
        setCpuFrequencyMhz(240);
    #elif defined(ARDUINO_ARCH_MBED_RP2040) || defined(ARDUINO_ARCH_RP2040)
    // No function for setting Freqency RP2040 ?
    #else
            // Handle other platforms 
    #endif 
    VerifyWifi();
    #if defined(ESP8266) || (ESP32)
    ArduinoOTA.handle();
    #endif 
    
    #if defined(WEB_DASHBOARD) 
        server.handleClient();
    #endif
}


void loop() {
    #if CORE == 1
         loopOneCore();
    #endif
    #if defined(ARDUINO_ARCH_MBED_RP2040) || defined(ARDUINO_ARCH_RP2040) //loop0
       job[0]->mine();
    #endif   
      
}
