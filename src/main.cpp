#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <DHT.h>
#include <IRremoteESP8266.h>
#include <IRsend.h>
#include <ir_Fujitsu.h>
#include <iostream>

#include "config.h"

// device configuration
#define PIN_IR_LED D1
#define PIN_DHT22 D2
#define PIN_STATUS_LED D4
#define DHT_TYPE DHT22

// seems like secure client must be used to work with mosquitto 2.0 - also need to set wifi.setInsecure()
WiFiClientSecure wifi;
PubSubClient mqtt(wifi);
IRFujitsuAC ac(PIN_IR_LED);
DHT dht(PIN_DHT22, DHT_TYPE);

float temperature, humidity;
int cnt;

bool isInitComplete = false;
// power state must be kept track of separately since library returns power on if cmd is not poweroff
// so power can be off on device and getPower() still could return true
// keeping track this way instead of discarding commands allows for state to sync on retained messages
bool isPowerOn = false;
bool isPowerful = false; // no way to get powerful state

void connectWifi() {
    wifi.setInsecure();
    WiFi.mode(WIFI_OFF);
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

    Serial.println("Connecting to WiFi...");
    while (WiFi.status() != WL_CONNECTED) {
        if (WiFi.status() == WL_CONNECT_FAILED) {
            Serial.println("Failed to connect to WiFi. Invalid credentials.");
            return;
        }
        delay(500);
    }
    Serial.println("WiFi connection established.");
}

// get mode from local fujitsu state and translate to home home assistant strings
void updateModeState() {
    std::string mode;
    if (isPowerOn) {
        switch (ac.getMode()) {
            case kFujitsuAcModeCool:
                mode = "cool";
                break;
            case kFujitsuAcModeFan:
                mode = "fan_only";
                break;
            case kFujitsuAcModeAuto:
                mode = "auto";
                break;
            case kFujitsuAcModeDry:
                mode = "dry";
                break;
            case kFujitsuAcModeHeat:
                mode = "heat";
                break;
        }
    } else {
        mode = "off";
    }

    mqtt.publish(MQTT_TOPIC_BASE "mode/state", mode.c_str(), true);
}

// update the state topic for setpoint
void updateSetpointState() {
    char sp[6];
    dtostrf(ac.getTemp(), 4, 1, sp);
    mqtt.publish(MQTT_TOPIC_BASE "setpoint/state", sp, true);
}

// update the state topic for fan
void updateFanState() {
    std::string mode;
    switch (ac.getFanSpeed()) {
        case kFujitsuAcFanHigh:
            mode = "high";
            break;
        case kFujitsuAcFanMed:
            mode = "medium";
            break;
        case kFujitsuAcFanLow:
            mode = "low";
            break;
        case kFujitsuAcFanAuto:
            mode = "auto";
            break;
        case kFujitsuAcFanQuiet:
            mode = "quiet";
            break;
    }
    mqtt.publish(MQTT_TOPIC_BASE "fan/state", mode.c_str(), true);
}

// update the state topic for swing
void updateSwingState() {
    std::string mode;
    switch (ac.getSwing()) {
        case kFujitsuAcSwingOff:
            mode = "off";
            break;
        case kFujitsuAcSwingVert:
            mode = "on";
            break;
        default:
            mode = "";
            break;
    }
    mqtt.publish(MQTT_TOPIC_BASE "swing/state", mode.c_str(), true);
}

// update state topic for powerful mode
void updatePowerfulState() {
    // can't customize case in HA for this topic, so must use uppercase on/off
    mqtt.publish(MQTT_TOPIC_BASE "powerful/state", isPowerful ? "ON" : "OFF", true);
}

// update all state topics using the above functions
void updateStates() {
    updateModeState();
    updateFanState();
    updateSwingState();
    updateSetpointState();
    updatePowerfulState();
}

void flashPin(unsigned char pin) {
    digitalWrite(pin, LOW);
    delay(500);
    digitalWrite(pin, HIGH);
}

// MQTT incoming message handler
void onMessage(const char *topic, byte *payload, unsigned int length) {
    // get pointer to substring after base topic - ie, mode/set from climate/device/mode/set
    const char *sub = &topic[strlen(MQTT_TOPIC_BASE)];
    // copy message from payload into message - note length + 1 for null termination char
    // note that payload doesn't clear between runs, so without copying would have to strcmp with length
    char message[length + 1];
    strncpy(message, (char *) payload, length);
    message[length] = '\0'; // must manually add null terminator after strncpy

    Serial.print("Received command on '");
    Serial.print(topic);
    Serial.print("': ");
    Serial.println(message);

    // handle incoming set message
    if (strcmp(sub, "mode/set") == 0) {
        // for all mode sets, must set power on manually since using combined power/mode endpoint
        if (strcmp(message, "cool") == 0) {
            ac.setMode(kFujitsuAcModeCool);
            ac.setPower(true);
            isPowerOn = true;
        } else if (strcmp(message, "heat") == 0) {
            ac.setMode(kFujitsuAcModeHeat);
            ac.setPower(true);
            isPowerOn = true;
        } else if (strcmp(message, "auto") == 0) {
            ac.setMode(kFujitsuAcModeAuto);
            ac.setPower(true);
            isPowerOn = true;
        } else if (strcmp(message, "fan_only") == 0) {
            ac.setMode(kFujitsuAcModeFan);
            ac.setPower(true);
            isPowerOn = true;
        } else if (strcmp(message, "dry") == 0) {
            ac.setMode(kFujitsuAcModeDry);
            ac.setPower(true);
            isPowerOn = true;
        } else {
            ac.setPower(false);
            isPowerOn = false;
        }
        ac.send();
    } else if (strcmp(sub, "setpoint/set") == 0) {
        float sp = atof(message);
        // atof returns 0 if can't parse
        // outside of these bounds doesn't work properly
        if (sp >= 16 && sp < 30) {
            ac.setTemp(sp);
        }
        ac.send();
    } else if (strcmp(sub, "fan/set") == 0) {
        if (strcmp(message, "low") == 0) {
            ac.setFanSpeed(kFujitsuAcFanLow);
        } else if (strcmp(message, "medium") == 0) {
            ac.setFanSpeed(kFujitsuAcFanMed);
        } else if (strcmp(message, "high") == 0) {
            ac.setFanSpeed(kFujitsuAcFanHigh);
        } else {
            ac.setFanSpeed(kFujitsuAcFanAuto);
        }
        ac.send();
    } else if (strcmp(sub, "swing/set") == 0) {
        if (strcmp(message, "on") == 0) {
            ac.setSwing(kFujitsuAcSwingVert);
        } else {
            ac.setSwing(kFujitsuAcSwingOff);
        }
        ac.send();
    } else if (strcmp(sub, "powerful/set") == 0) {
        // powerful mode requires special command (unlike other settings above)
        // thus only send powerful mode if device is already powered on
        // also the cmdPowerful is a toggle, so sending it twice will turn on then off
        // ir library doesn't track state for this, so manually track state in isPowerful
        if (isPowerOn) {
            isPowerful = !isPowerful;
            ac.setCmd(kFujitsuAcCmdPowerful);
        }
        ac.send();
    }

    // wait for 20s to pass before updating state so states aren't spammed when getting initial retained messages
    if (isInitComplete) {
        // update all state topics after changing state - probably don't need to do this, but it ensures
        // that all states are accurately reflected in home assistant
        flashPin(PIN_STATUS_LED);
        updateStates();
    }
}

// set mqtt parameters, loop until mqtt connection established, and subscribe to set topics
void connectMQTT() {
    mqtt.setServer(MQTT_HOST, MQTT_PORT);
    mqtt.setKeepAlive(30);
    mqtt.setBufferSize(256);
    mqtt.setCallback(onMessage);
    Serial.println("Connecting to MQTT broker...");
    while (!mqtt.connected()) {
        if (!mqtt.connect(MQTT_CLIENT_ID, MQTT_USER, MQTT_PASSWORD, MQTT_TOPIC_BASE "availability", 1, true, "offline")) {
            Serial.print("Failed with state: ");
            Serial.println(mqtt.state());
            delay(1000);
        }
    }
    Serial.println("MQTT connection established.");
    delay(100);
    mqtt.subscribe(MQTT_TOPIC_BASE "mode/set");
    mqtt.subscribe(MQTT_TOPIC_BASE "setpoint/set");
    mqtt.subscribe(MQTT_TOPIC_BASE "fan/set");
    mqtt.subscribe(MQTT_TOPIC_BASE "swing/set");
    mqtt.subscribe(MQTT_TOPIC_BASE "powerful/set");
    delay(1000);
    if (isInitComplete) {
        mqtt.publish(MQTT_TOPIC_BASE "availability", "online", true);
        digitalWrite(PIN_STATUS_LED, HIGH);
    }
}

// return true if a and b are nearly equal with precision of epsilon
bool nearEqual(float a, float b, float epsilon) {
    return fabs(a - b) < epsilon;
}

// update temperature and humidity readings from DHT - only publish if value changed more than .2/.5
void updateReadings() {
    float newTemperature = dht.readTemperature();
    float newHumidity = dht.readHumidity();
    char result[6];
    if (!isnan(newTemperature) && !nearEqual(temperature, newTemperature, 0.2)) {
        temperature = newTemperature;
        dtostrf(temperature, 4, 1, result);
        mqtt.publish(MQTT_TOPIC_BASE "temperature", result, true);
    }
    if (!isnan(newHumidity) && !nearEqual(humidity, newHumidity, 0.5)) {
        humidity = newHumidity;
        dtostrf(humidity, 4, 1, result);
        mqtt.publish( "humidity", result, true);
    }
}

void setup() {
    Serial.begin(115200);

    pinMode(PIN_STATUS_LED, OUTPUT);
    digitalWrite(PIN_STATUS_LED, LOW);

    delay(1000);

    connectWifi();
    connectMQTT();

    dht.begin();
    ac.begin();

    ac.setModel(ARREB1E);
}

void loop() {
    if (WiFi.status() != WL_CONNECTED) {
        digitalWrite(PIN_STATUS_LED, LOW);
        connectWifi();
    }
    if (!mqtt.connected()) {
        digitalWrite(PIN_STATUS_LED, LOW);
        connectMQTT();
    }

    mqtt.loop();

    updateReadings();

    // runs once 10s after loop has begun running to post online availability and update states
    if (!isInitComplete && cnt > 10) {
        // by the time this runs, all retained messages, etc should be handled
        isInitComplete = true;
        updateStates();
        mqtt.publish(MQTT_TOPIC_BASE "availability", "online", true);
        digitalWrite(PIN_STATUS_LED, HIGH);
        Serial.println("Init complete. Sending updates.");
    }

    cnt++;
    delay(1000);
}
