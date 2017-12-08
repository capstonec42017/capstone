/* 
  Scanse Sweep Arduino Library Examples
  
  MegaDistanceBlink:
      - Example sketch for using the Scanse Sweep with the Arduino Mega 2560.
      - Blinks the onboard LED at a frequency based on the distance of the 
        closest object in the angular window 330-360 degrees (ie: the 30 
        degrees before the azimuth 0 degree mark). For reference, the thin 
        groove and LED on the face of the sweep sensor represent the 0 degree 
        mark. 
      - Expected behavior: 
          1. Resets the device and waits a while to guarantee sweep has time to reset.
             During this time the LED onboard the arduino will blink at ~1Hz
          2. Adjusts device settings and verifies the parameters. During this
             step the LED on the face of the sweep should blink blue.
          3. Waits for the motor speed to stabilize/calibrate and then starts data
             acquisition.
          4. Checks for the closest object within 1 meter, within the angular range
             [330:360] degrees. Stand in front of this angular region and walk back
             and forth from about 25cm to 100cm away from the device. You should see 
             LED onboard the arduino blink rapidly when you are 1m away from the sweep,
             and slowly when you are close to the sweep.
          5. Continues indefinitely, and never stops the data stream. Sensor will have to
             be powered cycled to rerun the script.
      - Although the sweep is not meant to produce reliable readings below 
        1 meter, for this example the bounds of considered range are 0 to 100 cm.
        This is meant to allow the frequency adjustment to be obvious as you 
        move near the sweep.
      - Assumes Sweep sensor is physically connected to Serial #1 (RX1 & TX1)
        - For the sweep's power, ground, RX & TX pins, follow the connector 
          pinouts in the sweep user manual located here: 
          http://scanse.io/downloads
        - Be sure to connect RX_device -> TX_arduino & TX_device -> RX_arduino
      - You should provide dedicated external 5V power to the Sweep... NOT using 
        the arduino's power. Be sure to connect the ground from the power source 
        and the arduino.
  Created by Scanse LLC, June 7th, 2017.
  Released into the public domain.
*/

#include <Sweep.h>

const uint16_t DIST_CM_MIN = 1;   // (cm) Clip range values on the low end
const uint16_t DIST_CM_MAX = 100; // (cm) Clip range values on the high end
const uint16_t DIST_CM_RANGE = DIST_CM_MAX - DIST_CM_MIN;

// the angular range (Field of View). The distance of the shortest valid ranging in the
// angular window between [360-FOV: 0] degrees will be used.
// (0 deg corresponds to the thin groove + LED on the face of the sweep device)
const uint8_t FOV = 5;
float FRICTION_COEFFICIENT = 0.1;
float GRAVITY = 9.806; // units: meters/second^2
const uint8_t BRAKE_STATES = 6;

// Finite States for the program sequence
const uint8_t STATE_WAIT_ON_RESET = 0;
const uint8_t STATE_ADJUST_DEVICE_SETTINGS = 1;
const uint8_t STATE_VERIFY_CURRENT_DEVICE_SETTINGS = 2;
const uint8_t STATE_BEGIN_DATA_ACQUISITION = 3;
const uint8_t STATE_GATHER_DATA = 4;
const uint8_t STATE_UPDATE_BLINK_FREQUENCY = 5;
const uint8_t STATE_RESET = 6;
const uint8_t STATE_ERROR = 7;

// Create a Sweep device using Serial #1 (RX1 & TX1)
Sweep device(Serial1);

uint16_t closestDistanceInSpecifiedFOV = 4000; // the distance (in cm) of the closest object in the specified angular range
uint16_t interval = 1000;                      // interval at which to blink onboard LED (milliseconds)
uint16_t brakeCheckInterval = 500;            // interval at which to check and change braking (milliseconds)
int ledState = LOW;                            // ledState used to set the LED
unsigned long previousMillis = 0;              // time of the last LED update
unsigned long lastBrakeCheckMillis = 0;       // time braking was last increased/decreased

uint16_t distances[3];
unsigned long times[3];
long velocities;
bool danger;
bool warnState;
volatile unsigned int brakeCount;
volatile bool releaseBrake = false;

const int buzzerPin = 2;
const int buttonPin = 3;
const int ledWarnPin = 4;

// Current state in the program sequence
uint8_t currentState = STATE_WAIT_ON_RESET;

void setup()
{
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  pinMode(8, OUTPUT);
  pinMode(10, OUTPUT);
  digitalWrite(8, LOW);
  digitalWrite(10, LOW);
  pinMode(9, OUTPUT);
  digitalWrite(9, LOW);

  // Set up pin for piezo buzzer and warning LEDs
  pinMode(buzzerPin, OUTPUT);
  pinMode(ledWarnPin, OUTPUT);
  pinMode(buttonPin, INPUT);
  // Set up interrupt for button to release brakes
  // Wire for LOW output when button is pushed
  attachInterrupt(digitalPinToInterrupt(buttonPin), releaseBrakes, FALLING);
  //attachInterrupt(digitalPinToInterrupt(buttonPin), endRelease, RISING);
  
  // initialize serial1 for the sweep device
  Serial1.begin(115200); // sweep device
  Serial.begin(9600);
  //Serial2.begin(9600);

  // initialize counter variables and reset the current state
  reset();
}

// Loop functions as an FSM (finite state machine)
void loop()
{
  switch (currentState)
  {
  case STATE_WAIT_ON_RESET:
    currentState = waitOnReset() ? STATE_ADJUST_DEVICE_SETTINGS : STATE_ERROR;
    break;
  case STATE_ADJUST_DEVICE_SETTINGS:
    Serial.println("Adjusting\n");
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
    if (gatherDistanceInfo())
      currentState = STATE_UPDATE_BLINK_FREQUENCY;
    break;
  case STATE_UPDATE_BLINK_FREQUENCY:
    updateBlinkFrequency();
    // reset the closest distance
    closestDistanceInSpecifiedFOV = 4000;
    currentState = STATE_GATHER_DATA;
    break;
  case STATE_RESET:
    reset();
    currentState = STATE_WAIT_ON_RESET;
    break;
  default: // there was some error
    // DO NOTHING
    Serial.println("Error\n");
    interval = 10; // short enough interval to make the LED appear as if it is ON
    break;
  }
  updateLED();
}

// Waits ~8 seconds for the device to reset and verifies it can communicate
bool waitOnReset()
{
  int tempState = LOW;
  for (int i = 0; i < 16; i++)
  {
    delay(500);
    // toggle the LED
    tempState = (tempState == LOW) ? HIGH : LOW;
    digitalWrite(LED_BUILTIN, tempState);
  }

  // to verify communication, check a random command response that doesn't require motor is ready
  return device.getMotorSpeed() > 0;
}

// Adjusts the device settings
bool adjustDeviceSettings()
{
  // Set the motor speed to 2HZ (codes available from 1->10 HZ)
  bool bSuccess = device.setMotorSpeed(MOTOR_SPEED_CODE_2_HZ);

  /* 
   *  Device will always default to 500HZ scan rate when it is powered on.
   *  Snippet below is left for reference.
   */
  // Set the sample rate to 500HZ (codes available for 500, 750 and 1000 HZ)
  //bool bSuccess = device.setSampleRate(SAMPLE_RATE_CODE_500_HZ);
  return bSuccess;
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

// Gathers distance info, for the angular region [360-FOV: 0] degree
// returns true if distances for the complete range [360-FOV: 0] have been collected
bool gatherDistanceInfo()
{
  // attempt to get the next scan packet
  // Note: getReading() will return a ScanPacket object
  bool success = false;
  ScanPacket reading = device.getReading(success);
  unsigned long currTime = millis();
  if (success)
  {

    // consider a Field of View in the angular range [360-FOV: 0]
    if (reading.isSync())
    {
      // only consider valid readings (sensor will report distance of 1 for failed readings)
      uint16_t dist = reading.getDistanceCentimeters();
      if (dist > 1)
      {
        distances[2] = distances[1];
        distances[1] = distances[0];
        distances[0] = dist;

        times[2] = times[1];
        times[1] = times[0];
        times[0] = currTime;
        return true;
      }
    }
  }
  return false;
}

// Updates the stopping distances and set danger flag
void updateBlinkFrequency()
{
  float dist2 = (float) (long)distances[2];
  float dist1 = (float) (long)distances[1];
  float dist0 = (float) (long)distances[0];
  float time2 = (float) (long) times[2];
  float time1 = (float) (long) times[1];
  float time0 = (float) (long) times[0];
  
  float velo1 = 10 * (dist1 - dist0) / ((time0 - time1));
  float velo2 = 10 * (dist2 - dist0) / ((time0 - time2));
  float velo = min(velo1, velo2);
  float stopping_distance = velo*2.0 + sq(velo)/(2 * FRICTION_COEFFICIENT * GRAVITY) + (float)1.5;
  if (stopping_distance * 100 > dist0) {
    danger = true;
  }
  else {
    danger = false;
  }
  
}

// Pull/release brakes, check if the system is still "involved" (either warning signal or actually braking)
void updateLED()
{
  checkInterrupt();
  int state = LOW;
  digitalWrite(8, state);
  digitalWrite(10, state);
  unsigned long currentMillis = millis();
  //Serial.print("Current: " + String(currentMillis) + " previous: " + String(previousMillis) + "\n");
  // enter danger state, start warning
  if (previousMillis == 0 && danger) {
    warnState = true;
    previousMillis = currentMillis;
    ledState = HIGH;
    setWarnAlerts();
  }
  // past end of just warning, start or continue braking
  else if (previousMillis > 0 && currentMillis - previousMillis >= interval)
  {
    if (currentMillis - lastBrakeCheckMillis >= brakeCheckInterval) {
      lastBrakeCheckMillis = currentMillis;
      if (danger) {
        increaseBraking();
      }
      // decreaseBraking returns true on an attempt to decrease braking past 0
      else if (decreaseBraking()) {
        // reset timestamp
        previousMillis = 0;
        // determine the new State
        ledState = LOW;
        // toggle the LED
        setWarnAlerts();
        danger = false;
      }
    }
  }
}

void setWarnAlerts()
{
  if (ledState == HIGH)
    tone(buzzerPin, 440);
  else
    noTone(buzzerPin);
  digitalWrite(ledWarnPin, ledState);
  digitalWrite(LED_BUILTIN, ledState);
}

void increaseBraking()
{
  if (brakeCount < BRAKE_STATES) {
    brakeCount += 1;
    int state = HIGH;
    digitalWrite(8, state);
    //Serial.print("I\n");
    //Serial2.write(1);
  }
}

bool decreaseBraking()
{
  if (brakeCount == 0) {
    return true;
  }
  brakeCount -= 1;
  int state = HIGH;
  digitalWrite(10, state);
  //Serial.print("D\n");
  //Serial2.write(0);
  return false;
}

void releaseBrakes()
{
  brakeCount = 0;
  releaseBrake = true;
  //Serial2.write(2);
}


void checkInterrupt()
{
  if (releaseBrake) {
    //Serial2.write(2);
    digitalWrite(9, HIGH);
    releaseBrake = false;
    delay(2000);
    digitalWrite(9, LOW);
  }
}
// Resets the variables and state so the sequence can be repeated
void reset()
{
  currentState = STATE_WAIT_ON_RESET;
  closestDistanceInSpecifiedFOV = 4000;
  memset(times, 0, sizeof(times));
  memset(distances, 0, sizeof(distances));
  memset(velocities, 0, sizeof(velocities));
  danger = false;
  warnState = false;
  brakeCount = 0;
  ledState = LOW;
  device.reset();
}
