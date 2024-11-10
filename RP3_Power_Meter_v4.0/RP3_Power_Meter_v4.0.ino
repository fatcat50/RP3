#include "Arduino.h"
#include "esp_timer.h"
#include <ArduinoBLE.h>
#define BLE_DEVICE_NAME "RP3 Power Meter"  // Device name which can be scene in BLE scanning software.
#define BLE_LOCAL_NAME "RP3 Power Meter"   // Local name which should pop up when scanning for BLE devices.
#define PULSE_PIN 2                        // GPIO pin for pulse sensor
#define PI 3.1415926535897932384626433832795

// Model S configuration
int magnetCount = 4;
int cogTeethCount = 11;
int elBAndTension = 21;
float elBandConstant = 10.7;
float momOfInertia = 0.10880;
float cogWheelRad = 0.014195;
float linkLength = 0.008;
double initK = 0.0001;
double dragFactor = 0.0001;
double A = magnetCount / (2 * PI);
double angularVelocity = 0;
double angularVelocitySum = 0; 
int drivePulseCount = 0;
double dragFactorSum = 0;  
int recoverPulseCount = 0; 
int validReadingsCount = 0;

volatile int64_t prevT = 0;
volatile int64_t t = 0;
volatile int64_t dt = 0;
volatile int64_t pulseInTimeEnd = 0;
volatile int64_t pulseInTimeEnd_old = 0;
volatile int pulseInNumber = 0;
volatile bool newPulseDurationAvailable = false;
const int debounceTime = 6000;
const int upperLimit = 53366;
double driveSum = 0;
double recoverSum = 0;
int transitionPulseCount = 0;
unsigned long previousMillis = 0;
const long updateInterval = 1000;

const int numReadings = 4;
long readings[numReadings];
double avArray[numReadings];
int readIndex = 0;
int avArrayIndex = 0;
float total = 0;
double average = 0;
double averagePrev = 0;
double avInSeconds = 0;
double averagePrevInSeconds = 0;
double strokeInSeconds = 0;
float strokesPerMinute = 0;

enum State { RECOVER,
             DRIVE };
State currentState = DRIVE;
State previousState = DRIVE;

BLEService CyclePowerService("1818");
BLECharacteristic CyclePowerFeature("2A65", BLERead, 4);
BLECharacteristic CyclePowerMeasurement("2A63", BLENotify, 8);
BLECharacteristic CyclePowerSensorLocation("2A5D", BLERead, 1);

unsigned char bleBuffer[8];
unsigned char slBuffer[1];
unsigned char fBuffer[4];

float power = 0;
unsigned short revolutions = 0;
unsigned short prevRevolutions = 0;
unsigned short timestamp = 1;
unsigned short prevTimestamp = 1;
unsigned short timeStampDiff = 0;
unsigned short flags = 0x20;
byte sensorlocation = 0x0D;
bool updateValues = false;

// Interrupt function triggered by the pulse sensor
void buttonPinInterrupt() {
  t = esp_timer_get_time();
  dt = t - prevT;

  if (dt < debounceTime) {
    return;
  }

  // Update pulse data
  prevT = t;
  pulseInNumber++;
  pulseInTimeEnd_old = pulseInTimeEnd;
  pulseInTimeEnd = t;
  newPulseDurationAvailable = true;
}

// Determine if the current phase is DRIVE or RECOVER
bool detectState(double prev, double cur) {
  return (prev / cur) < 1;
}

// Count Av-Array-Entries for correct average calculation
int countValidReadings(long readings[], int arraySize) {
  int count = 0;
  for (int i = 0; i < arraySize; i++) {
    if (readings[i] != 0) {
      count++;
    }
  }

  return count;
}
// Calculations based on RP3-README
double calculateAngularVelocity(double pulseTime) {
  return (2.0 * PI) / (pulseTime * magnetCount);
}

double calculateDragFactor(double currentPulse, double previousPulse) {
  return A * momOfInertia * (1 - (previousPulse / currentPulse));
}

double calculatePower(double dragFactor, double angularVelocity) {
  return dragFactor * pow(angularVelocity, 3);
}

void setup() {
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(PULSE_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(PULSE_PIN), buttonPinInterrupt, FALLING);

  for (int i = 0; i < numReadings; i++) {
    readings[i] = 0;
  }

  // Setup BLE Service
  if (!BLE.begin()) {
    while (1);
  } else {
    BLE.setDeviceName(BLE_DEVICE_NAME);
    BLE.setLocalName(BLE_LOCAL_NAME);
    BLE.setAdvertisedService(CyclePowerService);
    CyclePowerService.addCharacteristic(CyclePowerFeature);
    CyclePowerService.addCharacteristic(CyclePowerMeasurement);
    CyclePowerService.addCharacteristic(CyclePowerSensorLocation);

    BLE.addService(CyclePowerService);
    BLE.advertise();
  }

  slBuffer[0] = sensorlocation & 0xff;

  fBuffer[0] = 0x00;
  fBuffer[1] = 0x00;
  fBuffer[2] = 0x00;
  fBuffer[3] = 0x08;

  CyclePowerFeature.writeValue(fBuffer, 4);
  CyclePowerSensorLocation.writeValue(slBuffer, 1);
}

void loop() {
  BLEDevice central = BLE.central();

  digitalWrite(LED_BUILTIN, LOW);
  delay(300);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(300);

  if (central) {
    digitalWrite(LED_BUILTIN, HIGH);
    Serial.print("Verbunden mit: ");
    Serial.println(central.address());

    while (central.connected()) {
      unsigned long currentMillis = millis();
      if (newPulseDurationAvailable) {
        noInterrupts();

        // Update the readings array
        total = total - readings[readIndex];
        readings[readIndex] = abs(pulseInTimeEnd - pulseInTimeEnd_old);
        total = total + readings[readIndex];
        readIndex++;
        if (readIndex >= numReadings) {
          readIndex = 0;
        }

        validReadingsCount = countValidReadings(readings, numReadings);

        // Calculate average
        averagePrev = average;
        averagePrevInSeconds = averagePrev / 1000000.0;
        average = total / validReadingsCount;
        avInSeconds = average / 1000000.0;

        if (average <= upperLimit) {
          avArray[avArrayIndex] = avInSeconds;
          avArrayIndex++;

          if (avArrayIndex >= numReadings) {
            avArrayIndex = 0;
          }

          // Determine current state
          State detectedState;
          if (detectState(averagePrev, average)) {
            detectedState = RECOVER;
          } else {
            detectedState = DRIVE;
          }

          // State transition from DRIVE to RECOVER
          // Stay in DRIVE State for the first 6 "RECOVER-Pulses"
          if (previousState == DRIVE && detectedState == RECOVER) {
            if (transitionPulseCount < 6) {
              transitionPulseCount++;
              currentState = DRIVE;
            } else {
              currentState = RECOVER;
              transitionPulseCount = 0;
            }
          } else {
            currentState = detectedState;
          }

          // Process DRIVE state
          // State transition from RECOVER to DRIVE: handle last 2 RECOVER-Pulses as DRIVE-Pulses
          if (currentState == DRIVE) {
            if (previousState == RECOVER) {
              drivePulseCount += 2;
              angularVelocitySum += calculateAngularVelocity(avArray[(avArrayIndex - 2 + numReadings) % numReadings]);
              driveSum += avArray[(avArrayIndex - 2 + numReadings) % numReadings];
              angularVelocitySum += calculateAngularVelocity(avArray[(avArrayIndex - 3 + numReadings) % numReadings]);
              driveSum += avArray[(avArrayIndex - 3 + numReadings) % numReadings];

              recoverSum -= avArray[(avArrayIndex - 2 + numReadings) % numReadings];
              recoverSum -= avArray[(avArrayIndex - 3 + numReadings) % numReadings];

              dragFactor = (dragFactorSum / (recoverPulseCount - magnetCount));  // Durchschnitt berechnen

              dragFactorSum = 0;
              recoverPulseCount = 0;
            }

            drivePulseCount++;
            angularVelocitySum += calculateAngularVelocity(avInSeconds);
            driveSum += avInSeconds;
          }

          // Process RECOVER State
          // State transition from DRIVE to RECOVER: calculate power and the timestamp for strokes per minute for this stroke
          if (currentState == RECOVER) {
            if (previousState == DRIVE) {
              angularVelocity = angularVelocitySum / drivePulseCount;
              power = (float)calculatePower(dragFactor, angularVelocity);

              strokeInSeconds = driveSum + recoverSum;
              strokesPerMinute = 60 / (float)strokeInSeconds;

              timestamp += short(strokeInSeconds * 1024.0);

              angularVelocitySum = 0;
              drivePulseCount = 0;
              driveSum = 0;
              recoverSum = 0;
              updateValues = true;
            }

            recoverPulseCount++;
            recoverSum += avInSeconds;

            if (recoverPulseCount >= magnetCount) {  // first a(magnetcount) pulses ignored, siehe doc
              dragFactorSum += calculateDragFactor(avInSeconds, averagePrevInSeconds);
            }
          }
          previousState = currentState;
        }
        newPulseDurationAvailable = false;
        interrupts();
      }

      // BLE Datatranser: sends data every second, duplicate power/strokes per minute values are exptected and handled
      if (currentMillis - previousMillis >= updateInterval) {
        previousMillis = currentMillis;

        if (!updateValues) {
          timestamp += timeStampDiff;

          if (revolutions > 0) {
            revolutions++;
          }
        } else {
          revolutions++;
        }

        bleBuffer[0] = flags & 0xff;
        bleBuffer[1] = (flags >> 8) & 0xff;
        bleBuffer[2] = (short)round(power) & 0xff;
        bleBuffer[3] = ((short)round(power) >> 8) & 0xff;
        bleBuffer[4] = (unsigned short)revolutions & 0xff;
        bleBuffer[5] = ((unsigned short)revolutions >> 8) & 0xff;
        bleBuffer[6] = (unsigned short)timestamp & 0xff;
        bleBuffer[7] = ((unsigned short)timestamp >> 8) & 0xff;

        CyclePowerMeasurement.writeValue(bleBuffer, 8);

        timeStampDiff = timestamp - prevTimestamp;
        prevTimestamp = timestamp;

        updateValues = false;
      }
    }
  }
}