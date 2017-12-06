
/* 
  Scanse Sweep Arduino Library Examples
  
  MegaSerialPrinter:
      - Example sketch for using the Scanse Sweep with the Arduino Mega 2560.
        Collects 3 complete scans, and then prints the sensor readings
      - Assumes Sweep sensor is physically connected to Serial #1 (RX1 & TX1)
        - For the sweep's power, ground, RX & TX pins, follow the connector 
          pinouts in the sweep user manual located here: 
          http://scanse.io/downloads
        - Be sure to connect RX_device -> TX_arduino & TX_device -> RX_arduino
      - For best results, you should provide dedicated external 5V power to the Sweep
        rather than using power from the Arduino. Just be sure to connect the ground 
        from the power source and the arduino. If you are just experimenting, you can
        run the sweep off the 5V power from the Arduino with the Arduino receiving power
        over USB. However this has only been tested with an external powered USB hub. 
        It is possible that using a low power USB port (ex: laptop) to power the 
        arduino & sweep together will result in unexpected behavior. 
      - Note that running off of USB power is not entirely adequate for the sweep, 
        so the quantity and qaulity of sensor readings will drop. This is OK for
        this example, as it is only meant to provide some visual feedback over 
        the serial monitor.
      - In your own projects, be sure to use dedicated power instead of the USB.
  Created by Scanse LLC, February 21, 2017.
  Released into the public domain.
*/

#include <Sweep.h>

// Create a Sweep device using Serial #1 (RX1 & TX1)
Sweep device(Serial1);

// keeps track of how many scans have been collected
uint8_t scanCount = 0;

uint16_t SIG_STRENGTH_MIN = 40; // Set via testing
float BEGIN_ANGLE = 285;
float END_ANGLE = 75;
float FRICTION_COEFFICIENT = 0.7;
float GRAVITY = 980.6; // units: centimeters/second^2


// Arrays to store attributes of collected scans
uint16_t distances[3][50];
uint16_t times[3][50];
long velocities[50];
float angleVec[3][50];
bool warnState = false;
long warnTime = 0;

// Finite States for the program sequence
const uint8_t STATE_WAIT_FOR_USER_INPUT = 0;
const uint8_t STATE_ADJUST_DEVICE_SETTINGS = 1;
const uint8_t STATE_VERIFY_CURRENT_DEVICE_SETTINGS = 2;
const uint8_t STATE_BEGIN_DATA_ACQUISITION = 3;
const uint8_t STATE_GATHER_DATA = 4;
const uint8_t STATE_STOP_DATA_ACQUISITION = 5;
const uint8_t STATE_RESET = 6;
const uint8_t STATE_ERROR = 7;


// Current state in the program sequence
uint8_t currentState;

// String to collect user input over serial
String userInput = "";

// Pin for piezzo buzzer
const int buzzerPin = 2;
const int buttonPin = 3;

void setup()
{
  // Initialize serial
  Serial.begin(9600);    // serial terminal on the computer
  Serial1.begin(115200); // sweep device
  Serial2.begin(9600);   // serial connection for arduino pro mini, verified with Allen

  // reserve space to accumulate user message
  userInput.reserve(50);

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  pinMode(buzzerPin, OUTPUT);
  // interrupt for button, depending on how its wired in, use rise or falling instead of change to avoid buttonFlag

  attachInterrupt(digitalPinToInterrupt(buttonPin), releaseBrakes, HIGH); // set up interrupt for button, add case to send a 2 to Serial 2

  // initialize counter variables and reset the current state
  reset();
}

// Loop functions as an FSM (finite state machine)
void loop()
{
  switch (currentState)
  {
  case STATE_WAIT_FOR_USER_INPUT:
    if (listenForUserInput())
      currentState = STATE_ADJUST_DEVICE_SETTINGS;
    break;
  case STATE_ADJUST_DEVICE_SETTINGS:
    currentState = adjustDeviceSettings() ? STATE_VERIFY_CURRENT_DEVICE_SETTINGS : STATE_ERROR;
    break;
  case STATE_VERIFY_CURRENT_DEVICE_SETTINGS:
    currentState = verifyCurrentDeviceSettings() ? STATE_BEGIN_DATA_ACQUISITION : STATE_ERROR;
    break;
  case STATE_BEGIN_DATA_ACQUISITION:
    currentState = beginDataCollectionPhase() ? STATE_GATHER_DATA : STATE_ERROR;
    break;
  case STATE_GATHER_DATA:
    gatherSensorReading();
    currentState = scanCount > 10 ? STATE_STOP_DATA_ACQUISITION : STATE_GATHER_DATA;
    break;
  case STATE_STOP_DATA_ACQUISITION:
    currentState = stopDataCollectionPhase() ? STATE_RESET : STATE_ERROR;
    break;
  case STATE_RESET:
    Serial.println("\n\nAttempting to reset and run the program again...");
    reset();
    currentState = STATE_WAIT_FOR_USER_INPUT;
    break;
  default: // there was some error
    Serial.println("\n\nAn error occured. Attempting to reset and run program again...");
    reset();
    currentState = STATE_WAIT_FOR_USER_INPUT;
    break;
  }
}

// checks if the user has communicated anything over serial
// looks for the user to send "start"
bool listenForUserInput()
{
  while (Serial.available())
  {
    userInput += (char)Serial.read();
  }
  if (userInput.indexOf("start") != -1)
  {
    Serial.println("Registered user start.");
    return true;
  }
  return false;
}

// Adjusts the device settings
bool adjustDeviceSettings()
{
  // Set the motor speed to 5HZ (codes available from 1->10 HZ)
  bool bSuccess = device.setMotorSpeed(MOTOR_SPEED_CODE_3_HZ);
  Serial.println(bSuccess ? "\nSuccessfully set motor speed." : "\nFailed to set motor speed");

  bSuccess = device.setSampleRate(SAMPLE_RATE_CODE_500_HZ);

  return bSuccess;
}

// Querries the current device settings (motor speed and sample rate)
// and prints them to the console
bool verifyCurrentDeviceSettings()
{
  // Read the current motor speed and sample rate
  int32_t currentMotorSpeed = device.getMotorSpeed();
  if (currentMotorSpeed < 0)
  {
    Serial.println("\nFailed to get current motor speed");
    return false;
  }
  int32_t currentSampleRate = device.getSampleRate();
  if (currentSampleRate < 0)
  {
    Serial.println("\nFailed to get current sample rate");
    return false;
  }

  // Report the motor speed and sample rate to the computer terminal
  Serial.println("\nMotor Speed Setting: " + String(currentMotorSpeed) + " HZ");
  Serial.println("Sample Rate Setting: " + String(currentSampleRate) + " HZ");

  return true;
}

// Initiates the data collection phase (begins scanning)
bool beginDataCollectionPhase()
{
  // Attempt to start scanning
  Serial.println("\nWaiting for motor speed to stabilize and calibration routine to complete...");
  bool bSuccess = device.startScanning();
  Serial.println(bSuccess ? "\nSuccessfully initiated scanning..." : "\nFailed to start scanning.");
  if (bSuccess)
    Serial.println("\nGathering 3 scans...");
  return bSuccess;
}

// Gathers individual sensor readings until 3 complete scans have been collected
void gatherSensorReading()
{
  Serial.println("gathering sensor reading");
  // attempt to get the next scan packet
  // Note: getReading() will write values into the "reading" variable
  unsigned long readTime;
  bool success = false;
  // For testing
  scanCount += 1;
  Serial1.flush();
  ScanPacket reading = device.getReading(success);
  while (reading.getAngleDegrees() < BEGIN_ANGLE && reading.getAngleDegrees() > END_ANGLE) {
    reading = device.getReading(success);
    readTime = millis();
  }
  int currIndex = 0;
  int readIndex;
  while (reading.getAngleDegrees() >= BEGIN_ANGLE || reading.getAngleDegrees() <= END_ANGLE) {
    readIndex = findIndex(reading.getAngleDegrees());
    if (readIndex >= currIndex && reading.getSignalStrength() >= SIG_STRENGTH_MIN && reading.getAngleDegrees() <= 360) {
//      Serial.println("Angle: " + String(reading.getAngleDegrees()) + ", Read index: " + String(readIndex));
      distances[2][readIndex] = distances[1][readIndex];
      distances[1][readIndex] = distances[0][readIndex];
      distances[0][readIndex] = reading.getDistanceCentimeters();

      times[2][readIndex] = times[1][readIndex];
      times[1][readIndex] = times[0][readIndex];
      times[0][readIndex] = readTime;

      angleVec[2][readIndex] = angleVec[1][readIndex];
      angleVec[1][readIndex] = angleVec[0][readIndex];
      angleVec[0][readIndex] = reading.getAngleDegrees();

      currIndex = readIndex + 1;
    }
    reading = device.getReading(success);
    readTime = millis();
  }

  analyze();
}

int findIndex(float angle) {
  return (((int) angle + 75) % 360) / 3;
}

void analyze()
{
  calculateClosingSpeeds();
  if(checkForWarn())
  {
    digitalWrite(LED_BUILTIN, HIGH);
    // Only increase breaking if warning signals have gone on for more than 1.5ms
    if(warnState && ((millis() - warnTime) >= 1500) && warnTime != 0)
    {
      increaseBraking();
    }
    else if((millis() - warnTime) < 1500)
    {
      // Make sure warn state is true and do nothing
      warnState = true;

      // Sound piezzo buzzer, 262 Hz for 10 ms
      tone(buzzerPin, 262, 10);
      
    }
    // First sign of warning start timer and set warn state
    else
    {
      warnTime = millis();
      warnState = true;
    }
  }
  else
  {
    warnState = false;
    warnTime = 0;
    decreaseBraking();
  }
}
void calculateClosingSpeeds() {
  for (int i = 0; i < 50; i++) {
    velocities[i] = /*max((distances[2][i] - distances[0][i]) / (times[0][i] - times[2][i]),*/
            (distances[1][i] - distances[0][i]) / (times[0][i] - times[1][i]);
  }
}

bool checkForWarn()
{
  int stoppingDistance;
  for (int i = 0; i < 50; i++) {
    stoppingDistance = velocities[i]*1.5 + sq(velocities[i])/(2 * FRICTION_COEFFICIENT * GRAVITY) + 200;
    if (stoppingDistance >= distances[0][i]) {
      Serial.println("Found warning\n");
      return true;
    }
  }
  return false;
}

void increaseBraking() {
  // Send interrupt to Pro Mini
  Serial2.write(1);
}

void decreaseBraking() {
  // Send interrupt to Pro Mini
  Serial2.write(0);
}

void releaseBrakes() {
  // Send interrupt to Pro mini
  Serial2.write(2);
}

// Terminates the data collection phase (stops scanning)
bool stopDataCollectionPhase()
{
  // Attempt to stop scanning
  bool bSuccess = device.stopScanning();

  Serial.println(bSuccess ? "\nSuccessfully stopped scanning." : "\nFailed to stop scanning.");

  for (int i = 0; i < 3; i++) {
    Serial.println("\nScan " + String(i) + ":\n");
    for (int j = 0; j < 50; j++) {
      Serial.println("Index: " + String(j) + ", Distance: " + String(distances[i][j]) + ", Time: " + String(times[i][j]) 
                      + ", Velocity: " + String(velocities[j]) + ", Angle: "  + String(angleVec[i][j], 3) + "\n");
    }
  }
  return bSuccess;
}

// Resets the variables and state so the sequence can be repeated
void reset()
{
  scanCount = 0;
  // reset the sensor
  device.reset();
  delay(50);
  Serial.flush();
  userInput = "";
  memset(times, 0, sizeof(times));
  memset(distances, 0, sizeof(distances));
  memset(velocities, 0, sizeof(velocities));
  memset(angleVec, 0, sizeof(angleVec));
  Serial.println("\n\nWhenever you are ready, type \"start\" to to begin the sequence...");
  currentState = 0;
}
