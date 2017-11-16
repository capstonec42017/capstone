/* DOESN'T WORK SORRY, WORK IN PROGRESS. MODELED AFTER THE BLINK EXAMPLE */

#include <Sweep.h>

const uint8_t FOV = 180;

Sweep device(Serial1);

/* 40 m, the farthest it can see. Used to compare to objects its seen so far */
uint16_t closestDistanceInSpecifiedFOV = 40000;
// Start and end angles for scanning
float beginAngle = 285.0;
float endAngle = 75.0;

// Arrays to store attributes of collected scans
bool syncValues[500];         // 1 -> first reading of new scan, 0 otherwise
float angles[500];            // in degrees (accurate to the millidegree)
uint8_t signalStrengths[500]; // 0:255, higher is better
uint8_t maxSyncValues = 500;

uint16_t distances[3][50];
uint16_t times[3][50];
bool warnState = false;

// Finite States for the program sequence
const uint8_t STATE_WAIT_ON_RESET = 0;
const uint8_t STATE_ADJUST_DEVICE_SETTINGS = 1;
const uint8_t STATE_VERIFY_CURRENT_DEVICE_SETTINGS = 2;
const uint8_t STATE_BEGIN_DATA_ACQUISITION = 3;
const uint8_t STATE_GATHER_DATA = 4;
const uint8_t STATE_ERROR = 5;

// Current state in the program sequence
uint8_t currentState = STATE_WAIT_ON_RESET;

void setup() {
  // Initialize serial for the sweep device
  Serial.begin(9600);    // serial terminal on the computer
  Serial1.begin(115200); // sweep device

  // initialize counter variables and reset the current state
  reset();
}

void loop() {
  switch (currentState)
  {
  case STATE_WAIT_ON_RESET:
    Serial.print("Waiting on reset\n");
    currentState = waitOnReset() ? STATE_ADJUST_DEVICE_SETTINGS : STATE_ERROR;
    break;
  case STATE_ADJUST_DEVICE_SETTINGS:
    Serial.print("Adjusting\n");
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
    gatherDistanceInfo();
    calculateClosingSpeeds();
    /*if (checkForWarn()) {
      if (timer > 1500) {
        increaseBraking();
      }
      else if (!warnState) {
        warnState = true;
        // TODO: Start timer
      }
    }
    else {
      warnState = false;
      decreaseBraking();
    }*/
    break;
  case STATE_ERROR:
    // Need to add error handling
    reset();
    currentState = STATE_WAIT_ON_RESET;
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
  return device.getMotorSpeed() > 0;
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


// Querries the current device settings (motor speed and sample rate)
// and prints them to the console
bool verifyCurrentDeviceSettings()
{
  // Read the current motor speed and sample rate
  int32_t currentMotorSpeed = device.getMotorSpeed();
  if (currentMotorSpeed < 0)
    return false;
  int32_t currentSampleRate = device.getSampleRate();
  if (currentSampleRate < 0)
    return false;
  return true;
}

// Main scanning function, perform data analysis algorithms here
void gatherDistanceInfo()
{
  Serial.print("Gathering\n");
  unsigned long readTime;
  bool success = false;
  String msg;
  ScanPacket curr = device.getReading(success);
  /*while (!(last.getAngleDegrees() <= beginAngle && curr.getAngleDegrees() >= beginAngle)) {
    last = curr;
    curr = device.getReading(success);
    readTime = millis();
  }*/
  int currIndex = 0;
  int readIndex;
  //while (curr.getAngleDegrees() >= beginAngle || curr.getAngleDegrees() <= endAngle) {
  for (int i = 1; i < 100; i++) {
    readIndex = 0;//findIndex(curr.getAngleDegrees());
    //if (readIndex >= currIndex) {
//      distances[2][readIndex] = distances[1][readIndex];
//      distances[1][readIndex] = distances[0][readIndex];
      distances[0][0] = curr.getDistanceCentimeters();
//
//      times[2][readIndex] = times[1][readIndex];
//      times[1][readIndex] = times[0][readIndex];
//      times[0][readIndex] = readTime;

      msg += "Angle: " + String(curr.getAngleDegrees(), 3) + ", Distance: " + String(distances[0][0]) + ", Index: " + String(readIndex) + "\n";
      currIndex = readIndex + 1;
      
    //}
    curr = device.getReading(success);
    //readTime = millis();
  }
  Serial.println(msg);
}

int findIndex(float angle) {
  return ((int) (angle + 75) % 360) / 3;
}
void calculateClosingSpeeds() {
  // Compute velocities, can be simplified to save time, since this is overwriting known values
  for(int i = 0; i < sizeof(velocities)/sizeof(velocities[0]); i++)
  {
    for(int j = 0; j < sizeof(velocities)/sizeof(velocities[0][0]); j++)
    {
      velocities[i][j] = distances[i][j] / times[i][j];
    }
  }
  
}

bool checkForWarn() {
  
}

void increaseBraking() {
  // TODO: send increase braking signal
}

void decreaseBraking() {
  // TODO: send decrease braking signal
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
  memset(times, 0, sizeof(times));
  memset(distances, 0, sizeof(distances));
  Serial.println("Device speed: " + String(device.getMotorSpeed()));
  device.reset();
}
