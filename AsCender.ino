#include <WiFi.h>
#include <PubSubClient.h>

#define LED_BUILDIN 2

const char* ssid = "your-ssid";
const char* pass = "your-pass";
const char* mqttServer = "soldier.cloudmqtt.com";
const int mqttPort = 14608;
#define MQTTUSER "wyyyrgco"
#define MQTTPASS "jpOZK7lVUmlR"
#define MQTT_SERIAL_PUBLISH_CH "/AsCender/serial/tx"
#define MQTT_SERIAL_RECEIVER_CH "/AsCender/serial/rx"

const char* payloadCh = "/AsCender/payload";

WiFiClient wifiClient;
PubSubClient client(wifiClient);

#define NO_INTERNET 0
#define INET_CONNECTED 1
#define MQTT_CONNECTED 2

uint8_t STAT = NO_INTERNET;

hw_timer_t* timer = NULL;
volatile SemaphoreHandle_t timerSemaphore;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

volatile uint32_t isrCounter = 0;
volatile uint32_t lastIsrAt = 0;

void IRAM_ATTR onTimer() {
  // Increment the counter and set the time of ISR
  portENTER_CRITICAL_ISR(&timerMux);
  isrCounter++;
  lastIsrAt = millis();
  portEXIT_CRITICAL_ISR(&timerMux);
  // Give a semaphore that we can check in the loop
  xSemaphoreGiveFromISR(timerSemaphore, NULL);
  // It is safe to use digitalRead/Write here if you want to toggle an output
}

void setup_timer() {
  // Create semaphore to inform us when the timer has fired
  timerSemaphore = xSemaphoreCreateBinary();

  // Use 1st timer of 4 (counted from zero).
  // Set 80 divider for prescaler (see ESP32 Technical Reference Manual for more
  // info).
  timer = timerBegin(0, 80, true);

  // Attach onTimer function to our timer.
  timerAttachInterrupt(timer, &onTimer, true);

  // Set alarm to call onTimer function every second (value in microseconds).
  // Repeat the alarm (third parameter)
  timerAlarmWrite(timer, 1000000, true);
}

void connectWiFi() {
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, pass);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    digitalWrite(LED_BUILDIN, HIGH);
    delay(100);
    digitalWrite(LED_BUILDIN, LOW);
    Serial.print('.');
  }
  Serial.println("WiFi connected");
  STAT = INET_CONNECTED;
}

void connectMQTT() {
  // Loop until we're reconnected
  while (WiFi.status() == WL_CONNECTED && !client.connected()) {
    digitalWrite(LED_BUILDIN, HIGH);
    delay(300);
    digitalWrite(LED_BUILDIN, LOW);
    Serial.print("Attempting MQTT connection...");
    // Create a random client ID
    String clientId = "AsC-1";
    //    clientId += String(random(0xffff), HEX);
    // Attempt to connect
    if (client.connect(clientId.c_str(), MQTTUSER, MQTTPASS)) {
      Serial.println("connected");
      //Once connected, publish an announcement...
      {
        const int len = clientId.length() + 1;
        char id[len];
        clientId.toCharArray(id, len);
        client.publish("/client", id);
      }
      // ... and resubscribe
      client.subscribe(MQTT_SERIAL_RECEIVER_CH);
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 2 seconds before retrying
      delay(2000);
    }
  }
  STAT = MQTT_CONNECTED;
}

void callback(char* topic, byte *payload, unsigned int len) {
  Serial.println("-------new message from broker-----");
  Serial.print("channel:");
  Serial.println(topic);
  Serial.print("data:");
  Serial.write(payload, len);
  Serial.println();
}

void publishSerialData(char *serialData) {
  client.publish(MQTT_SERIAL_PUBLISH_CH, serialData);
}

void publishMessage(char* message, char* channel) {
  client.publish(channel, message);
}

char payload[50];
int a, b, c, d, e, f, g, h, j, k = 0, l = 0, m = 0, r = 0;
void update_dummies() {
  if (r >= 5) r = 0;
  a = a + r;
  b = b + 9;
  c = (c + r + 100) / 3;
  d = (d + 10 + c) / 3;
  e = (e + r + 15) / 3;
  f = (f - r + 30) / 2;
  g = (g - r + 150) / 2;
  h = (h - r + 300) / 2;
  j = (j - r + 100) / 3;
  k = k + r;
  l = l + r;
  m = m + r;
  r++;
  sprintf(payload, "%03d %03d %03d %03d %03d %03d %03d %03d %03d %03d %03d %03d %03d", 5, a, b, c, d, e, f, g, h, j, k, l, m);
}

void setup() {
  Serial.begin(115200);
  pinMode(LED_BUILDIN, OUTPUT);
  //  reconnect_wifi();
  client.setServer(mqttServer, mqttPort);
  client.setCallback(callback);
  //  reconnect();
  setup_timer();
  // Start an alarm
  timerAlarmEnable(timer);
}

unsigned long next_blink = millis();

void routine() {
  if (WiFi.status() != WL_CONNECTED)
    STAT = NO_INTERNET;
  else if (!client.connected())
    STAT = INET_CONNECTED;
  else
    STAT = MQTT_CONNECTED;

  switch (STAT) {
    case NO_INTERNET :
      connectWiFi();
      break;
    case INET_CONNECTED :
      connectMQTT();
      break;
    case MQTT_CONNECTED :
      if (xSemaphoreTake(timerSemaphore, 0) == pdTRUE) {
        uint32_t isrCount = 0, isrTime = 0;
        // Read the interrupt count and time
        portENTER_CRITICAL(&timerMux);
        isrCount = isrCounter;
        isrTime = lastIsrAt;
        portEXIT_CRITICAL(&timerMux);

        update_dummies();
        if (STAT == MQTT_CONNECTED) {
          publishMessage(payload, "/AsCender/payload");
        }
      }
      client.loop();
      if (next_blink < millis()) {
        digitalWrite(LED_BUILDIN, HIGH);
        delay(100);
        digitalWrite(LED_BUILDIN, LOW);
        next_blink = millis() + 5000;
      }
  }
}

void loop() {
  routine();
}
