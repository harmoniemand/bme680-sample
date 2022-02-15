#include <Arduino.h>
#include "bsec.h"
#include <Wire.h>
 
// #define I2C_Freq 100000
#define SDA_0 21
#define SCL_0 22


Bsec iaqSensor;
int errorState = 0;
String output;

int Intervall = 1000;
int lastRun = 0;

void checkIaqSensorStatus(void)
{
  if (iaqSensor.status != BSEC_OK)
  {
    if (iaqSensor.status < BSEC_OK)
    {
      output = "BSEC error code : " + String(iaqSensor.status);
      Serial.println(output);
    }
    else
    {
      output = "BSEC warning code : " + String(iaqSensor.status);
      Serial.println(output);
    }
  }
  else if (iaqSensor.bme680Status != BME680_OK)
  {
    if (iaqSensor.bme680Status < BME680_OK)
    {
      output = "BME680 error code : " + String(iaqSensor.bme680Status);
      Serial.println(output);
    }
    else
    {
      output = "BME680 warning code : " + String(iaqSensor.bme680Status);
      Serial.println(output);
    }
  }
  else
  {
    if (errorState != 0)
    {
      errorState = 0;
    }
  }

}

void setup()
{
  Serial.begin(115200);
  Wire.begin();

  iaqSensor.begin(BME680_I2C_ADDR_SECONDARY, Wire);
  checkIaqSensorStatus();

  bsec_virtual_sensor_t sensorList[2] = {
      // BSEC_OUTPUT_RAW_TEMPERATURE,
      // BSEC_OUTPUT_RAW_PRESSURE,
      // BSEC_OUTPUT_RAW_HUMIDITY,
      // BSEC_OUTPUT_RAW_GAS,
      BSEC_OUTPUT_IAQ,
      BSEC_OUTPUT_STATIC_IAQ,
      // BSEC_OUTPUT_CO2_EQUIVALENT,
      // BSEC_OUTPUT_BREATH_VOC_EQUIVALENT,
      // BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_TEMPERATURE,
      // BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_HUMIDITY,
  };

  iaqSensor.updateSubscription(sensorList, 2, BSEC_SAMPLE_RATE_LP);
  checkIaqSensorStatus();
}

void loop()
{

  if (iaqSensor.run())
  { // If new data is available

    // float rawTemperature = iaqSensor.rawTemperature;
    // float pressure = iaqSensor.pressure;
    // float rawHumidity = iaqSensor.rawHumidity;
    // float gasResistance = iaqSensor.gasResistance;
    float iaq = iaqSensor.iaq;
    float iaqAccuracy = iaqSensor.iaqAccuracy;
    // float temperature = iaqSensor.temperature;
    // float humidity = iaqSensor.humidity;
    // float staticIaq = iaqSensor.staticIaq;
    // float co2Equivalent = iaqSensor.co2Equivalent;
    // float breathVocEquivalent = iaqSensor.breathVocEquivalent;

    if (lastRun < millis() - Intervall)
    {
      lastRun = millis();
      unsigned long time_trigger = millis();

      output = String(time_trigger);
      // output += ", " + String(rawTemperature);
      // output += ", " + String(pressure);
      // output += ", " + String(rawHumidity);
      // output += ", " + String(gasResistance);
      output += ", " + String(iaq);
      output += ", " + String(iaqAccuracy);
      // output += ", " + String(temperature);
      // output += ", " + String(humidity);
      // output += ", " + String(staticIaq);
      // output += ", " + String(co2Equivalent);
      // output += ", " + String(breathVocEquivalent);
      Serial.println(output);
    }
  }
  else
  {
    checkIaqSensorStatus();
  }
}