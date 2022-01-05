#include <Arduino.h>
#include <OneWire.h>
#include <DallasTemperature.h>

/* -------------------------------------------------------------------------- 
* DEFINES
---------------------------------------------------------------------------- */
#define TEMP_SENSOR_PIN         2     // IO pin of the used arduino
#define TEMP_SENSOR_RESOLUTION  12    // [bit] Resolution in bit (9 to 12)
#define TEMP_SENSOR_READ_CYCLE  3000  // [s] Time base for collecting new temperature values

/* -------------------------------------------------------------------------- 
Objects/variables
---------------------------------------------------------------------------- */
// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(TEMP_SENSOR_PIN);

// Pass our oneWire reference to Dallas Temperature Lib
DallasTemperature sensors(&oneWire);
DeviceAddress tempDeviceAddress;

float temperature = 0.0;

/* -------------------------------------------------------------------------- 
* STATIC FUNCTION PROTOTYPES
---------------------------------------------------------------------------- */
static void vTemperatureSensorSetup(void);
static void vReadTemperatureFromSensor(void);

/* -------------------------------------------------------------------------- 
* FUNCTIONS
---------------------------------------------------------------------------- */
void setup(void)
{
  Serial.begin(9600);
  vTemperatureSensorSetup();
}

void loop(void)
{ 
  vReadTemperatureFromSensor();
  delay(10);
}

/* -------------------------------------------------------------------------- 
* STATIC FUNCTIONS
---------------------------------------------------------------------------- */
static void vTemperatureSensorSetup(void)
{
  sensors.begin();
  sensors.getAddress(tempDeviceAddress, 0);
  sensors.setResolution(tempDeviceAddress, TEMP_SENSOR_RESOLUTION);
  
  // Trigger initial temperature request
  sensors.setWaitForConversion(false);
  sensors.requestTemperatures();
}

static void vReadTemperatureFromSensor(void)
{
  static uint32_t u32LastRequest = millis();
  uint32_t u32MinimumPeriod = 0;
  uint32_t u32Period = 0;
  
  // Calculate the needed minimal time the sensor needs at this resolution
  u32MinimumPeriod = 750 / (1 << (12 - TEMP_SENSOR_RESOLUTION));

  // Check if time base is long enough for our resolution
  if(TEMP_SENSOR_READ_CYCLE < u32MinimumPeriod)
  {
    // Not long enough, change to minimum
    u32Period = u32MinimumPeriod;
  }
  else
  {
    u32Period = TEMP_SENSOR_READ_CYCLE;
  }

  // Check if time period is over
  if (millis() - u32LastRequest > u32Period)
  {
    // Collect temperature from sensor
    temperature = sensors.getTempCByIndex(0);

    // Start new temperature read  
    sensors.requestTemperatures(); 
    u32LastRequest = millis(); 

    // Debug output
    Serial.print("Temperature: ");
    Serial.println(temperature);
  }
}