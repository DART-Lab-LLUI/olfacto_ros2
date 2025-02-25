#include <Wire.h>
#include "Adafruit_MCP9600.h"

Adafruit_MCP9600 mcp1;
Adafruit_MCP9600 mcp2;

#define HUMIDITY_PIN A0  // HIH-4000 analog output connected to A0

// Humidity sensor calibration constants from the datasheet
const float zeroOffset = 0.826; // Voltage at 0% RH
const float slope = 0.03148;    // Volts per %RH (mV converted to V)
const float temperature = 17.3;    // Temperature in C

void setup() {
    Serial.begin(115200);
    Wire.begin();
    Wire.setClock(400000);  // Increase I2C speed to 400 kHz

    // Initialize first MCP9600 at address 0x67
    if (!mcp1.begin(0x67)) {
        Serial.println("MCP9600 #1 not found! Check wiring.");
        while (1);
    }
    
    // Initialize second MCP9600 at address 0x60
    if (!mcp2.begin(0x60)) {
        Serial.println("MCP9600 #2 not found! Check wiring.");
        while (1);
    }

    Serial.println("MCP9600 Sensors Ready!");

    // Set both sensors to 12-bit for max speed (16ms conversion time)
    mcp1.setThermocoupleType(MCP9600_TYPE_K);
    mcp2.setThermocoupleType(MCP9600_TYPE_K);
    
    mcp1.setADCresolution(MCP9600_ADCRESOLUTION_12);
    mcp2.setADCresolution(MCP9600_ADCRESOLUTION_12);
}

void loop() {
    unsigned long timestamp = millis();

    // Request data from both thermocouple sensors simultaneously
    float hotJunction1 = mcp1.readThermocouple();
    float coldJunction1 = mcp1.readAmbient();
    float hotJunction2 = mcp2.readThermocouple();
    float coldJunction2 = mcp2.readAmbient();

    // Humidity calculation
    int sensorValue = analogRead(HUMIDITY_PIN);     // Read the analog voltage from the humidity sensor
    float voltage = sensorValue * (5.04 / 1023.0);     // Convert the analog reading (0-1023) to a voltage (0-5V)
    float raw_rh = (voltage - (0.16 * 5.04)) / (0.0062 * 5.04);     // Calculate the relative humidity (RH) using the calibration formula
    float true_rh = raw_rh / (1.0546 - (0.00216 * temperature));    // temperature correction

    // Print data in CSV format
    Serial.print(timestamp); Serial.print(",");
    Serial.print(hotJunction1); Serial.print(",");
    Serial.print(coldJunction1); Serial.print(",");
    Serial.print(hotJunction2); Serial.print(",");
    Serial.print(coldJunction2); Serial.print(",");
    Serial.print(true_rh);
    Serial.println();

    delay(1);  // Maximize sampling rate
}
