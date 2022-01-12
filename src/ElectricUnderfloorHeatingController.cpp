#include <Arduino.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <KnxTpUart.h>
#include <EEPROM.h>
#include "ElectricUnderfloorHeatingController.h"

/* -------------------------------------------------------------------------- 
* DEFINES
---------------------------------------------------------------------------- */
#define DEBUG

/* -------------------------------------------------------------------------- 
Objects/variables
---------------------------------------------------------------------------- */
// OneWire
OneWire oneWire(TEMP_SENSOR_PIN);
DallasTemperature sensors(&oneWire);
DeviceAddress tempDeviceAddress;

// KNX device
KnxTpUart knx(&Serial1, KNX_PA);

// Electric floor heating variables
ElectricFloorHeatingRegulation Heater;

/* -------------------------------------------------------------------------- 
* STATIC FUNCTION PROTOTYPES
---------------------------------------------------------------------------- */
static void vTemperatureSensorSetup(void);
static void vReadTemperatureFromSensor(void);
static void vInitializeKNX(void);
static void vSendTemperatureToKnx(void);
static void vSendRelayStateToKnx(void);
static void vInitializeEeprom(void);
static void vCalculateInternalSetpoint(void);
static void vTwoPointTemperatureRegulation(void);
static void vSendTemperatureSetpointToKnx(void);
static void vCheckIfConreteTemperaturIsPlausible(void);
static void vSendErrorStateToKnx(void);

/* -------------------------------------------------------------------------- 
* FUNCTIONS
---------------------------------------------------------------------------- */
void setup(void)
{
#ifdef DEBUG
  Serial.begin(9600);
#endif
  vInitializeEeprom();
  vInitializeKNX();
  vTemperatureSensorSetup();
}

void loop(void)
{ 
  // Collect temperature from external floor sensor
  vReadTemperatureFromSensor();

  // Calculate internal working temperature setpoint and relay state
  vCalculateInternalSetpoint();
  vTwoPointTemperatureRegulation();
  vCheckIfConreteTemperaturIsPlausible();

  // Send calculated/measured values to the KNX bus
  vSendTemperatureToKnx();
  vSendRelayStateToKnx();
  vSendTemperatureSetpointToKnx();
  vSendErrorStateToKnx();
}

/* -------------------------------------------------------------------------- 
* STATIC FUNCTIONS
---------------------------------------------------------------------------- */
static void vInitializeEeprom(void)
{
  // Check if Device ID changed
  if(EEPROM.read(EEPROM_ADDRESS_ID) != DEVICE_ID)
  {
#ifdef DEBUG
    Serial.println("Device ID changed, EEPROM will be reinitialized!");
#endif

    // ID changed, reinitialize the EEPROM
    for (uint16_t i = 1 ; i < EEPROM.length() ; i++) 
    {
      EEPROM.update(i, 0xFF);
    }

    // Update initial values
    EEPROM.put(EEPROM_ADDRESS_ID, (uint8_t)DEVICE_ID);
    EEPROM.put(EEPROM_ADDRESS_SUM_WINTER, (uint8_t)Winter); // To start working after initial uploard
    EEPROM.put(EEPROM_ADDRESS_TEMP_SET, (float)TEMP_SETPOINT_DEFAULT);
  }
  else
  {
    // Device ID didn't changed, load values
    EEPROM.get(EEPROM_ADDRESS_TEMP_SET, Heater.dTemperatureSetpoint);
    EEPROM.get(EEPROM_ADDRESS_SUM_WINTER, Heater.fSummerWinterMode);

    // Check if old value is valid, apply limitations!
    Heater.dTemperatureSetpoint = min(Heater.dTemperatureSetpoint, TEMP_SETPOINT_MAX);
    Heater.dTemperatureSetpoint = max(Heater.dTemperatureSetpoint, TEMP_SETPOINT_MIN); 

#ifdef DEBUG
    Serial.print("Device ID didn't changed, read old setpoint from EEPROM: ");
    Serial.println(Heater.dTemperatureSetpoint);
    Serial.print("And old state of summer winter mode:: ");
    Serial.println(Heater.fSummerWinterMode);
#endif
  }
}

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
    Heater.dFloorTemperature = sensors.getTempCByIndex(0);

    // Check if temperature value is valid or not
    if(Heater.dFloorTemperature == DEVICE_DISCONNECTED_C)
    {
      // Use minimum temperature as fallback
      Heater.dFloorTemperature = TEMP_SETPOINT_MIN;

      // Set error to active
      Heater.u8Error = ERROR_SENSOR_NOT_CONNECTED;
    }

    // Start new temperature read  
    sensors.requestTemperatures(); 
    u32LastRequest = millis(); 

#ifdef DEBUG
    Serial.print("Current floor temperature: ");
    Serial.println(Heater.dFloorTemperature);
#endif
  }
}

static void vSendTemperatureToKnx(void)
{
  static uint32_t u32LastTime = millis();

  // Check if time period is over
  if (millis() - u32LastTime > TEMP_SENSOR_SEND_CYCLE)
  {
    knx.groupWrite2ByteFloat(KNX_GA_TEMP_CONCRETE, Heater.dFloorTemperature);
    u32LastTime = millis(); 
  }
}

static void vSendErrorStateToKnx(void)
{
  static uint32_t u32LastTime = millis();
  static uint8_t u8OldErrorState = ERROR_NO_ERROR_ACTICE;

  // Check if time period is over
  if (millis() - u32LastTime > ERROR_SEND_CYCLE)
  {
    knx.groupWrite1ByteInt(KNX_GA_ERROR, Heater.u8Error);
    u32LastTime = millis(); 
  }

  // If the error state changes, also send out the error
  if(u8OldErrorState != Heater.u8Error)
  {
    knx.groupWrite1ByteInt(KNX_GA_ERROR, Heater.u8Error);
  }

  // Save old state of u8Error for change detection
  u8OldErrorState = Heater.u8Error;
}

static void vSendRelayStateToKnx(void)
{
  static uint32_t u32LastTime = millis();
  static bool fLastSendState = true; // Init with true, to force update on startup

    // Check if an update of the relay state is needed
  if(fLastSendState != Heater.fRelayState)
  {
    // Check if time period is over, to ensure minimum relay switch time, or an error occurend (then send immediately)
    if((    (millis() - u32LastTime) > MINIMUM_RELAY_TIME)
         || (Heater.u8Error != ERROR_NO_ERROR_ACTICE) )
    {
      fLastSendState = Heater.fRelayState;
      u32LastTime = millis(); 

      knx.groupWriteBool(KNX_GA_HEATER_ACTUATOR, Heater.fRelayState);

#ifdef DEBUG
      Serial.print("Send relay state to KNX bus: ");
      Serial.println(Heater.fRelayState);
#endif
    }
  }
}

static void vSendTemperatureSetpointToKnx(void)
{
  static uint32_t u32LastTime = millis();
  static float dLastSendValue = 0.0;

  // Check if time period is over
  if (millis() - u32LastTime > TEMP_SETPOINT_SEND_CYCLE)
  {
    knx.groupWrite2ByteFloat(KNX_GA_TEMP_ROOM_SETPOINT, Heater.dTemperatureSetpoint);
    u32LastTime = millis(); 
  }

  // Check if temperature setpoint changed
  if(dLastSendValue != Heater.dTemperatureSetpoint)
  {
#ifdef DEBUG
    Serial.print("Last send value:" );
    Serial.print(dLastSendValue);
    Serial.print("; New value:" );
    Serial.println(Heater.dTemperatureSetpoint);
#endif

    knx.groupWrite2ByteFloat(KNX_GA_TEMP_ROOM_SETPOINT, Heater.dTemperatureSetpoint);
    dLastSendValue = Heater.dTemperatureSetpoint;
  }
}

static void vCheckIfConreteTemperaturIsPlausible(void)
{
  static bool fRelayStateOld = false;
  static uint32_t u32SwitchOnTime = 0;
  static float dSwitchOnTemperature = 0.0;


  // Check if relay state changed from off to on
  if(   (Heater.fRelayState == true)
      &&(fRelayStateOld == false)  )
  {
    // Save timestamp of the heater switch on (and temperature)
    u32SwitchOnTime = millis();
    dSwitchOnTemperature = Heater.dFloorTemperature;
  }

  // Check if heater is on for a long time
  if(   (millis() - u32SwitchOnTime > TEMP_SENSOR_ERROR_TIME)
      &&(Heater.fRelayState == true)  )
  {
    // Check if the temperature hasn't risen by a defined delta temperature
    if(Heater.dFloorTemperature - dSwitchOnTemperature < TEMP_SENSOR_ERROR_DELTA)
    {
      // Set heater error to active
      Heater.u8Error = ERROR_SENSOR_NOT_REACTING;
    }
  }

  // Save old value for signal change detection
  fRelayStateOld = Heater.fRelayState;
}

static void vCalculateInternalSetpoint(void)
{
  // Check if function is disabled in general or disabled through windows
  if(   (Heater.fFrostProtection == true)
      ||(Heater.fWindow1State == WindowOpen)
      ||(Heater.fWindow2State == WindowOpen)  )
  {
    // Heater is deactivated, use minimum setpoint (frost protection)
    Heater.dInternalTemperatureSetpoint = TEMP_SETPOINT_MIN;
  }
  else
  {
    // Heater is active, calculate internal setpoint
    
    // Check if it's night mode
    if(Heater.fDayNight == Night)
    {
      Heater.dInternalTemperatureSetpoint = Heater.dTemperatureSetpoint - TEMP_REDUCTION_NIGHT;
    }
    else
    {
      Heater.dInternalTemperatureSetpoint = Heater.dTemperatureSetpoint;
    }
  }

  // Limit temperature setpoint to useful values
  Heater.dInternalTemperatureSetpoint = min(Heater.dInternalTemperatureSetpoint, TEMP_SETPOINT_MAX);
  Heater.dInternalTemperatureSetpoint = max(Heater.dInternalTemperatureSetpoint, TEMP_SETPOINT_MIN);
}

static void vTwoPointTemperatureRegulation(void)
{
  static bool fMaxTempViolated = false;

  // Check if measured temperature is above temperature setpoint
  if(Heater.dRoomTemperature > Heater.dInternalTemperatureSetpoint)
  {
    Heater.fRelayState = false;
  }
  else if(Heater.dRoomTemperature <= (Heater.dInternalTemperatureSetpoint - TEMP_HYSTERESIS))
  {
    Heater.fRelayState = true;
  }

  // However, even if the room temperature setpoint isn't reached, the floor temperature needs to be watched!
  if(Heater.dFloorTemperature >= TEMP_SETPOINT_MAX)
  {
    fMaxTempViolated = true;
  }
  else if(Heater.dFloorTemperature < (TEMP_SETPOINT_MAX - TEMP_MAX_HYSTERESIS))
  {
    fMaxTempViolated = false;
  }

  // If maximum temperature is violated, 
  // or summer mode is active, or an error occured, set off the heater
  if(   (fMaxTempViolated == true)
      ||(Heater.fSummerWinterMode == Summer)  
      ||(Heater.u8Error != ERROR_NO_ERROR_ACTICE) )
  {
    Heater.fRelayState = false;
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
  knx.addListenGroupAddress(KNX_GA_TEMP_SETPOINT_CORR);
  knx.addListenGroupAddress(KNX_GA_HEATER_ACTUATOR);
  knx.addListenGroupAddress(KNX_GA_TEMP_ROOM);
  knx.addListenGroupAddress(KNX_GA_TEMP_ROOM_SETPOINT);
  knx.addListenGroupAddress(KNX_GA_WINDOW_1_STATE);
  knx.addListenGroupAddress(KNX_GA_WINDOW_2_STATE);
  knx.addListenGroupAddress(KNX_GA_FROST_PROTECTION); 
  knx.addListenGroupAddress(KNX_GA_DAY_NIGHT);
  knx.addListenGroupAddress(KNX_GA_SUMMER_WINTER);
  knx.addListenGroupAddress(KNX_GA_ERROR);
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
          knx.groupAnswer2ByteFloat(KNX_GA_TEMP_CONCRETE, Heater.dFloorTemperature);
        }
        // Current enable/disable state
        else if(strcmp(target.c_str(), KNX_GA_FROST_PROTECTION) == 0) 
        {
          knx.groupAnswerBool(KNX_GA_FROST_PROTECTION, Heater.fFrostProtection);
        }
        // Current room temperature setpoint
        else if(strcmp(target.c_str(), KNX_GA_TEMP_ROOM_SETPOINT) == 0) 
        {
          knx.groupAnswer2ByteFloat(KNX_GA_TEMP_ROOM_SETPOINT, Heater.dTemperatureSetpoint);
        } 
        // Error state request
        else if(strcmp(target.c_str(), KNX_GA_ERROR) == 0) 
        {
          knx.groupAnswer1ByteInt(KNX_GA_ERROR, Heater.u8Error);
        } 
        break;

      case KNX_COMMAND_WRITE:
        // Room temperature setpoint correction
        if(strcmp(target.c_str(), KNX_GA_TEMP_SETPOINT_CORR) == 0) 
        {
          if(telegram->getBool() == SetpointIncrease)
          {
            // Increase internal temperature setpoint by the correction step
            Heater.dTemperatureSetpoint += TEMP_SETPOINT_CORRECTION;
          }
          else
          {
            // Decrease internal temperature setpoint by the correction step
            Heater.dTemperatureSetpoint -= TEMP_SETPOINT_CORRECTION;
          }

          // Limit temperature setpoint to useful values
          Heater.dTemperatureSetpoint = min(Heater.dTemperatureSetpoint, TEMP_SETPOINT_MAX);
          Heater.dTemperatureSetpoint = max(Heater.dTemperatureSetpoint, TEMP_SETPOINT_MIN); 

          // Update EEPROM, if value has changed
          EEPROM.put(EEPROM_ADDRESS_TEMP_SET, (float)Heater.dTemperatureSetpoint);

#ifdef DEBUG
        Serial.print("New room temperature setpoint: ");
        Serial.println(Heater.dTemperatureSetpoint);
#endif
        } 
        // Current room temperature
        else if(strcmp(target.c_str(), KNX_GA_TEMP_ROOM) == 0) 
        {
          Heater.dRoomTemperature = telegram->get2ByteFloatValue();

#ifdef DEBUG
        Serial.print("Current room temperature: ");
        Serial.println(Heater.dRoomTemperature);
#endif
        }
        // Current summer/winter state
        else if(strcmp(target.c_str(), KNX_GA_SUMMER_WINTER) == 0) 
        {
          Heater.fSummerWinterMode = telegram->getBool();
          EEPROM.put(EEPROM_ADDRESS_SUM_WINTER, (uint8_t)Heater.fSummerWinterMode);

#ifdef DEBUG
        Serial.print("Summer/Winter changed to: ");
        Serial.println(Heater.fSummerWinterMode);
#endif
        }
        // Current day/night state
        else if(strcmp(target.c_str(), KNX_GA_DAY_NIGHT) == 0) 
        {
          Heater.fDayNight = telegram->getBool();

#ifdef DEBUG
        Serial.print("Day/Night changed to: ");
        Serial.println(Heater.fDayNight);
#endif
        }
        // Current state of window 1
        else if(strcmp(target.c_str(), KNX_GA_WINDOW_1_STATE) == 0) 
        {
          Heater.fWindow1State = telegram->getBool();

#ifdef DEBUG
        Serial.print("Window 1 changed state to: ");
        Serial.println(Heater.fWindow1State);
#endif
        }
        // Current state of window 2
        else if(strcmp(target.c_str(), KNX_GA_WINDOW_2_STATE) == 0) 
        {
          Heater.fWindow2State = telegram->getBool();

#ifdef DEBUG
        Serial.print("Window 2 changed state to: ");
        Serial.println(Heater.fWindow2State);
#endif
        }
        // Current enable/disable state
        else if(strcmp(target.c_str(), KNX_GA_FROST_PROTECTION) == 0) 
        {
          Heater.fFrostProtection = telegram->getBool();

#ifdef DEBUG
        Serial.print("Heater frost protection active: ");
        Serial.println(Heater.fFrostProtection);
#endif
        }
        break;

      default:
        break;
    }
  }
}
