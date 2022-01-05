#include <Arduino.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <KnxTpUart.h>

/* -------------------------------------------------------------------------- 
* DEFINES
---------------------------------------------------------------------------- */
#define DEBUG

#define TEMP_SENSOR_PIN         2     // IO pin of the used arduino
#define TEMP_SENSOR_RESOLUTION  12    // [bit] Resolution in bit (9 to 12)
#define TEMP_SENSOR_READ_CYCLE  3000  // [ms] Time base for collecting new temperature values
#define TEMP_SENSOR_SEND_CYCLE  30000 // [ms] Time base for sending the new temperature to the KNX bus

#define TEMP_SETPOINT_MIN       6     // [°C]  6°C Frost protection
#define TEMP_SETPOINT_MAX       35    // [°C] 35°C should be enough?..

#define KNX_PA                  "1.0.181" // PA of the device
#define KNX_GA_TEMP_CURRENT     "8/2/9"   // GA for the measured temperature
#define KNX_GA_TEMP_SETPOINT    "8/2/10"   // GA for the temperature setpoint

/* -------------------------------------------------------------------------- 
Objects/variables
---------------------------------------------------------------------------- */
// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(TEMP_SENSOR_PIN);

// Pass our oneWire reference to Dallas Temperature Lib
DallasTemperature sensors(&oneWire);
DeviceAddress tempDeviceAddress;

// KNX device
KnxTpUart knx(&Serial1, KNX_PA);

float temperature = 0.0;
float temperatureSetpoint = 0.0;

/* -------------------------------------------------------------------------- 
* STATIC FUNCTION PROTOTYPES
---------------------------------------------------------------------------- */
static void vTemperatureSensorSetup(void);
static void vReadTemperatureFromSensor(void);
static void vInitializeKNX(void);
static void vSendTemperatureToKnx(void);

/* -------------------------------------------------------------------------- 
* FUNCTIONS
---------------------------------------------------------------------------- */
void setup(void)
{
#ifdef DEBUG
  Serial.begin(9600);
#endif

  vInitializeKNX();
  vTemperatureSensorSetup();
}

void loop(void)
{ 
  vReadTemperatureFromSensor();
  vSendTemperatureToKnx();
  delay(1);
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

#ifdef DEBUG
    Serial.print("Temperature: ");
    Serial.println(temperature);
#endif
  }
}

static void vSendTemperatureToKnx(void)
{
  static uint32_t u32LastTime = millis();

  // Check if time period is over
  if (millis() - u32LastTime > TEMP_SENSOR_SEND_CYCLE)
  {
    knx.groupWrite2ByteFloat(KNX_GA_TEMP_CURRENT, temperature);
    u32LastTime = millis(); 
  }
}

static void vInitializeKNX(void) 
{
  // Initialize connection to KNX BCU
  Serial1.begin(19200,SERIAL_8E1); // Even parity;
  while (!Serial1) {
    ; // wait for serial port to connect
  }

  // Reset UART
  if(Serial.available()) {
    knx.uartReset();
  }

  // Register KNX group addresses
  knx.addListenGroupAddress(KNX_GA_TEMP_CURRENT);
  knx.addListenGroupAddress(KNX_GA_TEMP_SETPOINT); 
}

void serialEvent1() 
{
  KnxTpUartSerialEventType eType = knx.serialEvent();

  if(eType == KNX_TELEGRAM) 
  {
    KnxTelegram* telegram = knx.getReceivedTelegram();

    String target = String(0 + telegram->getTargetMainGroup())   + "/" +
                    String(0 + telegram->getTargetMiddleGroup()) + "/" +
                    String(0 + telegram->getTargetSubGroup());

#ifdef DEBUG
  Serial.print("KNX Event on: ");
  Serial.println(target);
#endif

    // Is it a read request?
    switch(telegram->getCommand())
    {
      case KNX_COMMAND_READ:
        // Is the destination address equal to group address KNX_GA_TEMPERATURE
        if(strcmp(target.c_str(), KNX_GA_TEMP_CURRENT) == 0) 
        {
          knx.groupAnswer2ByteFloat(KNX_GA_TEMP_CURRENT, temperature);
        }
        break;

      case KNX_COMMAND_WRITE:
        if (strcmp(target.c_str(), KNX_GA_TEMP_SETPOINT) == 0) 
        {
          temperatureSetpoint = telegram->get2ByteFloatValue();

#ifdef DEBUG
          Serial.print("Temperature setpoint: ");
          Serial.println(temperatureSetpoint);
#endif

          // Limit temperature setpoint to useful values
          temperatureSetpoint = min(temperatureSetpoint, TEMP_SETPOINT_MAX);
          temperatureSetpoint = max(temperatureSetpoint, TEMP_SETPOINT_MIN); 
        }  
        break;

      default:
        break;
    }
  }
}
