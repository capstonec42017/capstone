/* DOESN'T WORK SORRY, WORK IN PROGRESS. MODELED AFTER THE BLINK EXAMPLE */

#include <Sweep.h>

const uint8_t FOV = 180;

Sweep device(Serial);

/* 40 m, the farthest it can see. Used to compare to objects its seen so far */
uint16_t closestDistanceInSpecifiedFOV = 40000;

// Arrays to store attributes of collected scans
bool syncValues[500];         // 1 -> first reading of new scan, 0 otherwise
float angles[500];            // in degrees (accurate to the millidegree)
uint16_t distances[500];      // in cm
uint8_t signalStrengths[500]; // 0:255, higher is better
uint8_t maxSyncValues = 500;

// Finite States for the program sequence
const uint8_t STATE_WAIT_ON_RESET = 0;
const uint8_t STATE_ADJUST_DEVICE_SETTINGS = 1;
const uint8_t STATE_VERIFY_CURRENT_DEVICE_SETTINGS = 2;
const uint8_t STATE_BEGIN_DATA_ACQUISITION = 3;
const uint8_t STATE_GATHER_DATA = 4;
const uint8_t STATE_STOP = 5;
const uint8_t STATE_ERROR = 6;

// Current state in the program sequence
uint8_t currentState = STATE_WAIT_ON_RESET;

void setup() {
  // Initialize serial for the sweep device
  Serial.begin(115200); // sweep device

  // initialize counter variables and reset the current state
  reset();
}

void loop() {
  switch (currentState)
  {
  case STATE_WAIT_ON_RESET:
    currentState = waitOnReset() ? STATE_ADJUST_DEVICE_SETTINGS : STATE_ERROR;
    break;
  case STATE_ADJUST_DEVICE_SETTINGS:
    currentState = adjustDeviceSettings() ? STATE_VERIFY_CURRENT_DEVICE_SETTINGS : STATE_ERROR;
    break;
  case STATE_VERIFY_CURRENT_DEVICE_SETTINGS:
    currentState = verifyCurrentDeviceSettings() ? STATE_BEGIN_DATA_ACQUISITION : STATE_ERROR;
    break;
  case STATE_BEGIN_DATA_ACQUISITION:
    // Attempt to start scanning (will wait for motor speed to stabilize and calibration routine to complete...)
    currentState = device.startScanning() ? STATE_GATHER_DATA : STATE_ERROR;
    break;
  case STATE_GATHER_DATA:
    // returns bool to end scanning
    currentState = STATE_GATHER_DATA;
    if (gatherDistanceInfo())
      // Stops when no more data to gather
      currentState = STATE_STOP;
    break;
  case STATE_ERROR:
    // Need to add error handling
    reset();
    currentState = STATE_WAIT_ON_RESET;
  case STATE_STOP:
    currentState = device.stopScanning();
    break;
  default: // there was some error
    // DO NOTHING
    break;
  }
}

// Wait ~8 seconds for the device to reset and verifies it can communicate
bool waitOnReset()
{
  for(int i = 0; i < 8; i++)
  {
    delay(1000);
  }
  return device.getSampleRate() > 0;
}

// Adjusts default settings
bool adjustDeviceSettings()
{
  // Set the motor speed to 4HZ
  bool bSuccessMotorSpeed = device.setMotorSpeed(MOTOR_SPEED_CODE_4_HZ);

  // Set the sample rate to 1000HZ
  bool bSuccessSampRate = device.setSampleRate(SAMPLE_RATE_CODE_1000_HZ);

  return bSuccessSampRate && bSuccessMotorSpeed;
}

// Main scanning function, perform data analysis algorithms here
bool gatherDistanceInfo()
{
  while(!checkStopScanning())
  {
    // Reads while bool success is false
    ScanPacket reading = device.getReading(false);
  }
  // Return true to get to STATE_STOP
  return true;
}

// See if scan is complete
// Should be based on user input not maxSyncValues
// This implementation is temporary
// True if should stop scanning
// False if otherwise
bool checkStopScanning()
{
  if(syncValues[maxSyncValues - 1] == 1)
  {
    return true;
  }
  else
  {
    return false;
  }
}

void reset()
{
  currentState = STATE_WAIT_ON_RESET;
  closestDistanceInSpecifiedFOV = 4000;
  device.reset();
}
