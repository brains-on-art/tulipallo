#include "Arduino.h"
#include <WiFi.h>
#include <PubSubClient.h>
#include <ws2812.h>

// FUNCTIONS
void connectToWiFi(const char * ssid, const char * pwd);
void messageCallback(char* topic, byte* payload, unsigned int length);
void publishData();
void requestURL(const char * host, uint8_t port);
void printLine();
void reconnectMQTT();

// WiFi network name and password:
const char * networkName = "";
const char * networkPswd = "";

// Internet domain to request from:
// const char * hostDomain = "example.com";
// const int hostPort = 80;

// On-board and LED stuff
const int BUTTON_PIN = 0;
const int ONBOARD_LED_PIN = 5;
const int LED_DATA_PIN = 14;
const uint16_t NUM_PIXELS = 1;  // <--- modify to suit your configuration
uint8_t MAX_COLOR_VAL = 255;
rgbVal *pixels;

// MQTT stuff
#define ORG "BOA" // your organization or "quickstart"
#define DEVICE_TYPE "esp32" // use this default for quickstart or customize to your registered device type
#define DEVICE_ID "pallo1" // use this default for quickstart or customize to your registered device id

char server[] = "192.168.32.114";
char topic[] = "/laituri";
// char authMethod[] = "use-token-auth";
// char token[] = TOKEN;
char clientId[] = "d:" ORG ":" DEVICE_TYPE ":" DEVICE_ID;

WiFiClient wifiClient;
// PubSubClient client(server, 1883, wifiClient);
PubSubClient client(wifiClient);
// -- end MQTT stuff --

void setup()
{
  // Initilize hardware:
  Serial.begin(115200);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(ONBOARD_LED_PIN, OUTPUT);
  // init led strip
  ws2812_init(LED_DATA_PIN);
  pixels = (rgbVal*)malloc(sizeof(rgbVal) * NUM_PIXELS);

  // Connect to the WiFi network (see function below loop)
  connectToWiFi(networkName, networkPswd);

  client.setServer(server, 1883);
  client.setCallback(messageCallback);
}

void loop()
{
    // reconnect to mqtt broker if necessary
    if (!!!client.connected()) {
        reconnectMQTT();
    }
    client.loop();

    if (digitalRead(BUTTON_PIN) == LOW) {// Button went down...
        while (digitalRead(BUTTON_PIN) == LOW)
        {// ...so do this while it's held down.
            // Turn on LED
            digitalWrite(ONBOARD_LED_PIN, HIGH);
            pixels[0] = makeRGBVal(MAX_COLOR_VAL, MAX_COLOR_VAL, MAX_COLOR_VAL);
            ws2812_setColors(NUM_PIXELS, pixels);
        }
        // And finally do this when it goes back up.
        // Turn off LED
        Serial.println("");
        digitalWrite(ONBOARD_LED_PIN, LOW);
        pixels[0] = makeRGBVal(0, 0, 0);
        ws2812_setColors(NUM_PIXELS, pixels);
        publishData();
    }
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
            digitalWrite(ONBOARD_LED_PIN, LOW);
            pixels[0] = makeRGBVal(0,0,0);
            ws2812_setColors(NUM_PIXELS, pixels);
        }
        else if (receivedChar == '1')
        {
            digitalWrite(ONBOARD_LED_PIN, HIGH);
            pixels[0] = makeRGBVal(MAX_COLOR_VAL, MAX_COLOR_VAL, MAX_COLOR_VAL);
            ws2812_setColors(NUM_PIXELS, pixels);
        }
    }
    Serial.println();
}

void publishData()
{
    String payload = "{ \"d\" : {\"RSSI\":";
    payload += WiFi.RSSI();
    payload += "}}";

    Serial.print("Sending payload: "); Serial.println(payload);

    if (client.publish(topic, (char*) payload.c_str())) {
        Serial.println("Publish ok");
    } else {
        Serial.println("Publish failed");
    }
    // delay(3000);
    }

void connectToWiFi(const char * ssid, const char * pwd)
{
  int ledState = 0;

  printLine();
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

  ledState = 1;
  digitalWrite(ONBOARD_LED_PIN, ledState);
  Serial.println();
  Serial.println("WiFi connected!");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}

void reconnectMQTT()
{
    Serial.print("Reconnecting pubsubclient to "); Serial.println(server);
    while (!!!client.connect(clientId)) {
        Serial.println(client.state());
        delay(500);
    }
    client.subscribe("/" DEVICE_ID);
    Serial.println("Connected pubsub client!");
}

void requestURL(const char * host, uint8_t port)
{
  printLine();
  Serial.println("Connecting to domain: " + String(host));

  // Use WiFiClient class to create TCP connections
  WiFiClient client;
  if (!client.connect(host, port))
  {
    Serial.println("connection failed");
    return;
  }
  Serial.println("Connected!");
  printLine();

  // This will send the request to the server
  client.print((String)"GET / HTTP/1.1\r\n" +
               "Host: " + String(host) + "\r\n" +
               "Connection: close\r\n\r\n");
  unsigned long timeout = millis();
  while (client.available() == 0)
  {
    if (millis() - timeout > 5000)
    {
      Serial.println(">>> Client Timeout !");
      client.stop();
      return;
    }
  }

  // Read all the lines of the reply from server and print them to Serial
  while (client.available())
  {
    String line = client.readStringUntil('\r');
    Serial.print(line);
  }

  Serial.println();
  Serial.println("closing connection");
  client.stop();
}

void printLine()
{
  Serial.println();
  for (int i=0; i<30; i++)
    Serial.print("-");
  Serial.println();
}
