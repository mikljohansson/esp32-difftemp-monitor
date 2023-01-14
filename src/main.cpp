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
        bool _connected;
        float _bias;
        String _sensorid;

    public:
        TemperatureSensor(int pin) {
            _wire = new OneWire(pin);
            _sensor = new DallasTemperature(_wire);
            _connected = false;
            _sensorid = String("b") + pin;
        }

        void begin() {
            _bias = preferences.getFloat(_sensorid.c_str());
            
            if (isnan(_bias)) {
                _bias = 0.0;
            }
            
            _sensor->begin();
        }

        void setBias(float bias) {
            if (!_connected) {
                bias = 0.0;
            }

            _bias = bias;
            preferences.putFloat(_sensorid.c_str(), bias);
        }

        float getBias() {
            return _bias;
        }

        float getTemperature() {
            float temp = getRawTemperature();
            return temp + _bias;
        }

        float getRawTemperature() {
            _sensor->requestTemperatures(); 
            float temp = _sensor->getTempCByIndex(0);
            _connected = temp > -100.0;
            return temp;
        }

        bool isConnected() {
            return _connected;
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
float calibrationSamples0[DTM_CALIBRATION_SAMPLES];
float calibrationSamples1[DTM_CALIBRATION_SAMPLES];
float calibrationSamples2[DTM_CALIBRATION_SAMPLES];

void IRAM_ATTR handleCalibrateButtonPress() {
    calibrationRequested += 1;
}

void setup() {
    Serial.begin(115200);
    while (!Serial);

    Serial.println("Starting up");
    Serial.printf("Core %d, clock %d MHz\n", xPortGetCoreID(), getCpuFrequencyMhz());
    Serial.printf("  XTAL %d MHz, APB %d MHz\n\n", getXtalFrequencyMhz(), getApbFrequency() / 1000000);

    preferences.begin("dtm-v1", false);

    sensor0.begin();
    sensor1.begin();
    sensor2.begin();

    Wire.setPins(DTM_SDA_PIN, DTM_SCL_PIN);
    oled.begin();
    oled.setTextColor(1);
    oled.setTextSize(1);

    pinMode(DTM_BUTTON_PIN, INPUT);
    attachInterrupt(DTM_BUTTON_PIN, handleCalibrateButtonPress, FALLING);

    calibrationRequested = 0;
}

String pad(int length, const char *message, float value, bool show) {
    String result;
    int spaces;

    if (show) {
        int len = snprintf(NULL, 0, message, value);
        spaces = std::max(length - len, 0);

        if (len) {
            char buf[len];
            sprintf(buf, message, value);
            result = String(buf, len);
        }
    }
    else {
        spaces = length;
    }

    for (int i = 0; i < spaces; i++) {
        result = ' ' + result;
    }

    return result;
}

String pad(int length, const char *message, float value) {
    return pad(length, message, value, value > -100.0);
}

float avg(float samples[], int num_samples) {
    float sum = 0.0;
    for (int i = 0; i < num_samples; i++) {
        sum += samples[i];
    }

    return sum / num_samples;
}

void loop() {
    if (calibrationRequested) {
        delay(500);
        
        if (calibrationRequested == 1) {
            for (int i = 0; i < DTM_CALIBRATION_SAMPLES + 1; i++) {
                // Collect a few readings with some delay
                oledPrint("Calibrating %d%%\n\nClick twice to reset\ncalibration again", (int)round(((float)i / DTM_CALIBRATION_SAMPLES) * 100.0));
                
                if (i < DTM_CALIBRATION_SAMPLES) {
                    calibrationSamples0[i] = sensor0.getTemperature();
                    calibrationSamples1[i] = sensor1.getTemperature();
                    calibrationSamples2[i] = sensor2.getTemperature();
                    delay(DTM_CALIBRATION_INTERVAL);
                }
            }

            float at0 = avg(calibrationSamples0, DTM_CALIBRATION_SAMPLES);
            float at1 = avg(calibrationSamples1, DTM_CALIBRATION_SAMPLES);
            float at2 = avg(calibrationSamples2, DTM_CALIBRATION_SAMPLES);

            float ref = -127.0;
            if (sensor0.isConnected()) {
                ref = at0;
            }
            else if (sensor1.isConnected()) {
                ref = at1;
            }
            else if (sensor2.isConnected()) {
                ref = at2;
            }
            
            if (ref > -100.0) {
                sensor0.setBias(ref - at0);
                sensor1.setBias(ref - at1);
                sensor2.setBias(ref - at2);
            }
        }
        else {
            // Double press to reset
            oledPrint("Resetting...");
            sensor0.setBias(0.0);
            sensor1.setBias(0.0);
            sensor2.setBias(0.0);
            delay(1000);
        }

        calibrationRequested = 0;
    }
    else {
        float t0 = sensor0.getTemperature();
        float t1 = sensor1.getTemperature();
        float t2 = sensor2.getTemperature();

        serialPrint("S1: %.01fC, S2: %.01fC, S3: %.01fC\n", t0, t1, t2);
        oledPrint("S1:%s D12:%s\nS2:%s D23:%s\nS3:%s D31:%s\nCA: %s %s %s", 
            pad(7, "%.01fC", t0).c_str(), pad(6, "%.01fC", abs(t0 - t1), sensor0.isConnected() && sensor1.isConnected()).c_str(), 
            pad(7, "%.01fC", t1).c_str(), pad(6, "%.01fC", abs(t1 - t2), sensor1.isConnected() && sensor2.isConnected()).c_str(),
            pad(7, "%.01fC", t2).c_str(), pad(6, "%.01fC", abs(t2 - t0), sensor2.isConnected() && sensor0.isConnected()).c_str(),
            pad(5, "%.02f", sensor0.getBias()).c_str(),
            pad(5, "%.02f", sensor1.getBias()).c_str(),
            pad(5, "%.02f", sensor2.getBias()).c_str());
        
        delay(500);
    }
}
