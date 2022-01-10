#include <Arduino.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <KnxTpUart.h>

/* -------------------------------------------------------------------------- 
* DEFINES
---------------------------------------------------------------------------- */
//#define DEBUG

#define TEMP_SENSOR_PIN         2     // IO pin of the used arduino
#define TEMP_SENSOR_RESOLUTION  12    // [bit] Resolution in bit (9 to 12)
#define TEMP_SENSOR_READ_CYCLE  5000  // [ms] Time base for collecting new temperature values
#define TEMP_SENSOR_SEND_CYCLE  60000 // [ms] Time base for sending the new temperature to the KNX bus

#define TEMP_SETPOINT_MIN       6     // [°C]  6°C Frost protection
#define TEMP_SETPOINT_MAX       34    // [°C] 33°C on the surface should not be exceeded!
#define TEMP_HYSTERESIS         2     // [K] 2K Hysteresis for 2-point regulator
#define MINIMUM_RELAY_TIME      5     // [min] 5min minimum relay state time (ensure at least ~20y of operation..)

#define KNX_PA                    "1.0.181" // PA of the device
// Group address "Outputs"
#define KNX_GA_TEMP_CONCRETE      "8/2/9"   // GA for the measured concrete temperature
#define KNX_GA_HEATER_ACTUATOR    "1/1/47"  // GA of the actuator the electric heater is connected to

// Group address "Inputs"
#define KNX_GA_TEMP_ROOM          "8/2/10"  // GA for the measured room temperature
#define KNX_GA_TEMP_ROOM_SETPOINT "8/2/11"  // GA for the temperature setpoint
#define KNX_GA_WINDOW_1_STATE     "5/5/2" // GA of a Window sensor (0 Open; 1 Closed)
#define KNX_GA_WINDOW_2_STATE     "5/5/3" // GA of a Window sensor (0 Open; 1 Closed)

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
    knx.groupWrite2ByteFloat(KNX_GA_TEMP_CONCRETE, temperature);
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
  if(Serial1.available()) {
    knx.uartReset();
  }

  // Register KNX group addresses
  knx.addListenGroupAddress(KNX_GA_TEMP_CONCRETE);
  knx.addListenGroupAddress(KNX_GA_TEMP_ROOM_SETPOINT); 
  knx.addListenGroupAddress(KNX_GA_WINDOW_1_STATE);
  knx.addListenGroupAddress(KNX_GA_WINDOW_2_STATE);
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
        if(strcmp(target.c_str(), KNX_GA_TEMP_CONCRETE) == 0) 
        {
          knx.groupAnswer2ByteFloat(KNX_GA_TEMP_CONCRETE, temperature);
        }
        break;

      case KNX_COMMAND_WRITE:
        if (strcmp(target.c_str(), KNX_GA_TEMP_ROOM_SETPOINT) == 0) 
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
