#include "Arduino.h"
#include <WiFi.h>
#include <PubSubClient.h>
#include <ws2812.h>

// FUNCTIONS
void connectToWiFi(const char * ssid, const char * pwd);
void messageCallback(char* topic, byte* payload, unsigned int length);
void publishData();
void requestURL(const char * host, uint8_t port);
void reconnectMQTT();
void setNextPixelValue(float step);

// WiFi network name and password:
const char * networkName = "tulipallokone";
const char * networkPswd = "palitullo";

// On-board and LED stuff
const int BUTTON_PIN = 0;
const int ONBOARD_LED_PIN = 5;
const int LED_DATA_PIN = 14;
const uint16_t NUM_PIXELS = 10;  // <--- modify to suit your configuration
uint8_t MAX_COLOR_VAL = 255;
rgbVal *pixels;
bool fireLit = false;
bool transitioning = false;
float curColorVal = 0;
float step = 0.05;

// MQTT stuff
#define ORG "BOA"
#define DEVICE_TYPE "esp32"
#define DEVICE_ID "1"
char server[] = "172.24.1.1";
char topic[] = "/reports";
// char authMethod[] = "use-token-auth";
// char token[] = TOKEN;
char clientId[] = "d:" ORG ":" DEVICE_TYPE ":" DEVICE_ID;
char reportTopic[] = "/reports/" DEVICE_ID;
WiFiClient wifiClient;
PubSubClient MQTTclient(wifiClient);
// -- end MQTT stuff --

// int counter = 0;
// int ledDelay = 500;
// int mqttCounterFreq = 20;

void setup()
{
  // Initilize hardware:
  Serial.begin(115200);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(ONBOARD_LED_PIN, OUTPUT);
  // init led strip
  ws2812_init(LED_DATA_PIN);
  pixels = (rgbVal*)malloc(sizeof(rgbVal) * NUM_PIXELS);
  // pixels[0] = makeRGBVal(0,0,0);
  for (int i = 0; i < NUM_PIXELS; i++) {
    pixels[i] = makeRGBVal(0, 0, 0);
  }
  ws2812_setColors(NUM_PIXELS, pixels);

  // Connect to the WiFi network
  connectToWiFi(networkName, networkPswd);

  MQTTclient.setServer(server, 1883);
  MQTTclient.setCallback(messageCallback);
}

void loop()
{
    // reconnect to network if necessary
    if (WiFi.status() != WL_CONNECTED) {
        connectToWiFi(networkName, networkPswd);
    }

    // reconnect to mqtt broker if necessary
    if (!!!MQTTclient.connected()) {
        reconnectMQTT();
    }
    MQTTclient.loop();
    setNextPixelValue(step);
    //
    // if (digitalRead(BUTTON_PIN) == LOW) {// Button went down...
    //     while (digitalRead(BUTTON_PIN) == LOW)
    //     {// ...so do this while it's held down.
    //         // Turn on LED
    //         digitalWrite(ONBOARD_LED_PIN, HIGH);
    //         pixels[0] = makeRGBVal(MAX_COLOR_VAL, MAX_COLOR_VAL, MAX_COLOR_VAL);
    //         ws2812_setColors(NUM_PIXELS, pixels);
    //     }
    //     // And finally do this when it goes back up.
    //     // Turn off LED
    //     Serial.println("");
    //     digitalWrite(ONBOARD_LED_PIN, LOW);
    //     pixels[0] = makeRGBVal(0, 0, 0);
    //     ws2812_setColors(NUM_PIXELS, pixels);
    //     publishData();
    // }

    // pixels[0] = makeRGBVal(MAX_COLOR_VAL,MAX_COLOR_VAL,MAX_COLOR_VAL);
    // pixels[0] = makeRGBVal(random(55)+200, random(155)+100, random(155));
    // ws2812_setColors(NUM_PIXELS, pixels);
    //
    // if (counter >= mqttCounterFreq) {
    //     publishData();
    //     counter = 0;
    // } else counter += 1;
    // delay(ledDelay);
}

void messageCallback(char* topic, byte* payload, unsigned int length) {
    Serial.print("Message arrived [");
    Serial.print(topic);
    Serial.print("] ");
    for (int i=0;i<length;i++) {
        char receivedChar = (char)payload[i];
        Serial.print(receivedChar);
        if (receivedChar == '0')
        {
            // pixels[0] = makeRGBVal(0,0,0);

            // for (int i = 0; i < NUM_PIXELS; i++) {
            //   pixels[i] = makeRGBVal(0, 0, 0);
            // }
            // ws2812_setColors(NUM_PIXELS, pixels);

            fireLit = false;
        }
        else if (receivedChar == '1')
        {
            // pixels[0] = makeRGBVal(MAX_COLOR_VAL, MAX_COLOR_VAL, MAX_COLOR_VAL);

            // for (int i = 0; i < NUM_PIXELS; i++) {
            //   pixels[i] = makeRGBVal(MAX_COLOR_VAL, MAX_COLOR_VAL, MAX_COLOR_VAL);
            // }
            // ws2812_setColors(NUM_PIXELS, pixels);

            fireLit = true;
        }
    }
    Serial.println();
}

void setNextPixelValue(float step) {
    if (fireLit) {
        // getting brighter but not there yet
        if (curColorVal < MAX_COLOR_VAL) {
            // increment by a tiny step and then floor the resulting value to the integer part
            // so if we increment by step 0.1, result is 10 loop-iterations of the same led int value
            curColorVal += step;
            // set the leds to the floored value
            for (int i = 0; i < NUM_PIXELS; i++) {
              pixels[i] = makeRGBVal((int)curColorVal, (int)curColorVal, (int)curColorVal);
            }
            ws2812_setColors(NUM_PIXELS, pixels);
            if ((int)curColorVal == 255) {
                Serial.println("reached peak brightness");
            }
        }
    } else {
        // getting darker but not there yet
        if (curColorVal > 0) {
            curColorVal -= step;
            //set leds to curColorVal
            for (int i = 0; i < NUM_PIXELS; i++) {
                pixels[i] = makeRGBVal((int)curColorVal, (int)curColorVal, (int)curColorVal);
            }
            ws2812_setColors(NUM_PIXELS, pixels);
        }
    }
}

void publishData()
{
    String payload = "{ \"d\" : {\"RSSI\":";
    payload += WiFi.RSSI();
    payload += "}}";

    Serial.print("Sending payload: "); Serial.println(payload);

    // if (MQTTclient.publish(topic, (char*) payload.c_str())) {
    if (MQTTclient.publish(reportTopic, (char*) payload.c_str())) {
        Serial.println("Publish ok");
    } else {
        Serial.println("Publish failed");
    }
}

void connectToWiFi(const char * ssid, const char * pwd)
{
  int ledState = 0;

  Serial.println("Connecting to WiFi network: " + String(ssid));

  WiFi.begin(ssid, pwd);

  while (WiFi.status() != WL_CONNECTED)
  {
    // Blink LED while we're connecting:
    digitalWrite(ONBOARD_LED_PIN, ledState);
    ledState = (ledState + 1) % 2; // Flip ledState
    delay(500);
    Serial.print(".");
  }

  ledState = 0;
  digitalWrite(ONBOARD_LED_PIN, ledState);
  Serial.println();
  Serial.println("WiFi connected!");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}

void reconnectMQTT()
{
    int ledState = 0;
    Serial.print("Reconnecting pubsubclient to "); Serial.println(server);
    // while (!!!MQTTclient.connect(clientId)) {
    if (!!!MQTTclient.connect(clientId)) {
        // Serial.println(MQTTclient.state());
        // Blink LED while we're connecting:
        digitalWrite(ONBOARD_LED_PIN, ledState);
        ledState = (ledState + 1) % 2; // Flip ledState
        delay(1000);
        Serial.print(".");
        delay(100);
    }
    ledState = 0;
    digitalWrite(ONBOARD_LED_PIN, ledState);
    MQTTclient.subscribe("/lights/" DEVICE_ID);
    Serial.println("Connected pubsub client!");
}

// void requestURL(const char * host, uint8_t port)
// {
//   printLine();
//   Serial.println("Connecting to domain: " + String(host));
//
//   // Use WiFiClient class to create TCP connections
//   WiFiClient client;
//   if (!client.connect(host, port))
//   {
//     Serial.println("connection failed");
//     return;
//   }
//   Serial.println("Connected!");
//   printLine();
//
//   // This will send the request to the server
//   client.print((String)"GET / HTTP/1.1\r\n" +
//                "Host: " + String(host) + "\r\n" +
//                "Connection: close\r\n\r\n");
//   unsigned long timeout = millis();
//   while (client.available() == 0)
//   {
//     if (millis() - timeout > 5000)
//     {
//       Serial.println(">>> Client Timeout !");
//       client.stop();
//       return;
//     }
//   }
//
//   // Read all the lines of the reply from server and print them to Serial
//   while (client.available())
//   {
//     String line = client.readStringUntil('\r');
//     Serial.print(line);
//   }
//
//   Serial.println();
//   Serial.println("closing connection");
//   client.stop();
// }
//
// void printLine()
// {
//   Serial.println();
//   for (int i=0; i<30; i++)
//     Serial.print("-");
//   Serial.println();
// }
