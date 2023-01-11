#include <Arduino.h>
#include <Preferences.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Adafruit_SSD1306.h>

#include "config.h"

// Persistent eeprom storage
Preferences preferences;

Adafruit_SSD1306 oled(128, 32);

class TemperatureSensor {
    private:
        OneWire *_wire;
        DallasTemperature *_sensor;

    public:
        TemperatureSensor(int pin) {
            _wire = new OneWire(pin);
            _sensor = new DallasTemperature(_wire);
        }

        void begin() {
            _sensor->begin();
        }

        float getTemperature() {
            _sensor->requestTemperatures(); 
            return _sensor->getTempCByIndex(0);
        }
};

TemperatureSensor sensor0(DTM_SENSOR_PIN_0);
TemperatureSensor sensor1(DTM_SENSOR_PIN_1);
TemperatureSensor sensor2(DTM_SENSOR_PIN_2);

template <typename... T>
void serialPrint(const char *message, T... args) {
    int len = snprintf(NULL, 0, message, args...);
    if (len) {
        char buf[len];
        sprintf(buf, message, args...);
        Serial.print(buf);
    }
}

template <typename... T>
void oledPrint(const char *message, T... args) {
    oled.clearDisplay();
    oled.setCursor(0, 0);
    
    int len = snprintf(NULL, 0, message, args...);
    if (len) {
        char buf[len];
        sprintf(buf, message, args...);
        oled.print(buf);
    }
    
    oled.display();
}

int calibrationRequested = 0;

void IRAM_ATTR handleCalibrateButtonPress() {
    calibrationRequested = 1;
}

void setup() {
    Serial.begin(115200);
    while (!Serial);

    Serial.println("Starting up");
    Serial.printf("Core %d, clock %d MHz\n", xPortGetCoreID(), getCpuFrequencyMhz());
    Serial.printf("  XTAL %d MHz, APB %d MHz\n\n", getXtalFrequencyMhz(), getApbFrequency() / 1000000);

    preferences.begin("dtm-v1", false);

    sensor0.begin();

    Wire.setPins(DTM_SDA_PIN, DTM_SCL_PIN);
    oled.begin();
    oled.setTextColor(1);
    oled.setTextSize(1);

    pinMode(DTM_BUTTON_PIN, INPUT);
    attachInterrupt(DTM_BUTTON_PIN, handleCalibrateButtonPress, FALLING);
}

void loop() {
    if (calibrationRequested) {
        for (int i = 0; i < DTM_CALIBRATION_SAMPLES; i++) {
            // Collect a few readings with some delay
            oledPrint("Calibrating %d%%", (int)round(((float)i / DTM_CALIBRATION_SAMPLES) * 100.0));
            delay(DTM_CALIBRATION_INTERVAL);
        }

        calibrationRequested = 0;
    }
    else {
        float temp0 = sensor0.getTemperature();
        float temp1 = sensor0.getTemperature();
        float temp2 = sensor0.getTemperature();

        serialPrint("S0: %.02fC, S1: %.02fC, S2: %.02fC\n", temp0, temp1, temp2);
        oledPrint("S0: %.02fC\nS1: %.02fC\nS2: %.02fC\nD01: %.02fC", temp0, temp1, temp2, abs(temp0 - temp1));
        delay(500);
    }
}
