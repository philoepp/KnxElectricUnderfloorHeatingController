#ifndef ELECTRIC_UNDERFLOOR_HEATING_CONTROLLER_H
#define ELECTRIC_UNDERFLOOR_HEATING_CONTROLLER_H

#include <Arduino.h>

/* -------------------------------------------------------------------------- 
* DEFINES
---------------------------------------------------------------------------- */
// EEPROM
#define EEPROM_ADDRESS_ID           0     // Position 0 is used for a unique ID (1 byte)
#define EEPROM_ADDRESS_SUM_WINTER   1     // Position 1 is used for the summer/winter switch
#define EEPROM_ADDRESS_TEMP_SET     2     // Position 2 is used for the temperature setpoint
#define DEVICE_ID                   1     // If the ID changes the EEPROM gets reinitialized  

// Temperature sensor settings
#define TEMP_SENSOR_PIN             2     // IO pin of the used arduino
#define TEMP_SENSOR_RESOLUTION      12    // [bit] Resolution in bit (9 to 12)
#define TEMP_SENSOR_READ_CYCLE      5000  // [ms] Time base for collecting new temperature values
#define TEMP_SENSOR_SEND_CYCLE      60000 // [ms] Time base for sending the new temperature to the KNX bus

// Error settings
#define TEMP_SENSOR_ERROR_TIME      (60UL * 60UL * 1000UL) // [ms] 60min - Time the temperature of the sensor should increase
#define TEMP_SENSOR_ERROR_DELTA     5     // [K] 5K - Temperature of the sensor should increase this much, when running for the error time
#define ERROR_SEND_CYCLE            (5UL * 60UL * 1000UL) // [ms] 5min - Time base for sending the error state to KNX bus

#define TEMP_SETPOINT_MIN           6     // [°C]  6°C Frost protection
#define TEMP_SETPOINT_DEFAULT       20.0f  // [°C] 20°C default temperature setpoint (float)
#define TEMP_SETPOINT_MAX           33    // [°C] 33°C on the surface should not be exceeded!
#define TEMP_SETPOINT_CORRECTION    0.2   // [K] 0.2K Used as setpoint correction steps
#define TEMP_SETPOINT_SEND_CYCLE    (30UL * 60UL * 1000UL) // [ms] 30min - Resend temperature setpoint every 30min
#define TEMP_MAX_HYSTERESIS         0.5   // [K] 0.5K Hysteresis for maximum temperature switch off
#define TEMP_HYSTERESIS             0.5   // [K] 0.5K Hysteresis for 2-point regulator of room temperature
#define TEMP_REDUCTION_NIGHT        2     // [K] 2K Temperature reduction in night mode
#define MINIMUM_RELAY_TIME          (5UL * 60UL * 1000UL) // [ms] 5min minimum relay state time (ensure at least ~20 years of operation..)

// Define the physical address of the KNX device
#define KNX_PA                      "1.0.181" // PA of the KNX device

// Group address "Outputs"
#define KNX_GA_TEMP_CONCRETE        "8/2/9"   // GA for the measured concrete temperature
#define KNX_GA_HEATER_ACTUATOR      "1/1/47"  // GA of the actuator the electric heater is connected to
#define KNX_GA_TEMP_ROOM_SETPOINT   "3/0/5"   // GA for the current active temperature setpoint
#define KNX_GA_ERROR                "3/4/5"   // GA for sending an error, if something is wrong

// Group address "Inputs"
#define KNX_GA_TEMP_ROOM            "8/2/10"  // GA for the measured room temperature
#define KNX_GA_TEMP_SETPOINT_CORR   "3/2/5"   // GA for the temperature setpoint correction (1 = Increase by x; 0 = decrease by x)
#define KNX_GA_WINDOW_1_STATE       "5/5/2"   // GA of a Window sensor (0 Open; 1 Closed)
#define KNX_GA_WINDOW_2_STATE       "5/5/3"   // GA of a Window sensor (0 Open; 1 Closed)
#define KNX_GA_FROST_PROTECTION     "3/3/5"   // GA to set the heater into frost protection mode (1 frost protection enabled; 0 normal operation)
#define KNX_GA_DAY_NIGHT            "0/4/3"   // GA for the day/night shift (use reduced setpoint at night) (0 night; 1 day)
#define KNX_GA_SUMMER_WINTER        "0/2/0"   // GA for the summer/winter switch (to disable the function in general in summer)

enum Errors
{
  ERROR_NO_ERROR_ACTICE = 0,
  ERROR_SENSOR_NOT_CONNECTED = 1,
  ERROR_SENSOR_NOT_REACTING = 2
};

enum NightDay 
{ 
  Night = 0, 
  Day = 1
};

enum WindowState
{
  WindowClosed = 0,
  WindowOpen = 1
};

enum SetpointCorrection
{
  SetpointDecrease = 0,
  SetpointIncrease = 1
};

enum SummerWinterMode
{
  Winter = 0,
  Summer = 1
};

struct ElectricFloorHeatingRegulation
{
  float dTemperatureSetpoint = 0;
  float dInternalTemperatureSetpoint = 0;
  float dFloorTemperature = 0.0;
  float dRoomTemperature = 0.0;
  bool fRelayState = false;
  bool fDayNight = Day;
  bool fWindow1State = WindowClosed;
  bool fWindow2State = WindowClosed;
  bool fFrostProtection = false;
  bool fSummerWinterMode = Winter;
  uint8_t u8Error = ERROR_NO_ERROR_ACTICE;
};

#endif //ELECTRIC_UNDERFLOOR_HEATING_CONTROLLER_H