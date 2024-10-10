#include "Arduino.h"
#include "esp_timer.h"
/* For the bluetooth funcionality */
#include <ArduinoBLE.h>
/* Device name which can be scene in BLE scanning software. */
#define BLE_DEVICE_NAME "RP3 Power Meter"
/* Local name which should pop up when scanning for BLE devices. */
#define BLE_LOCAL_NAME "RP3 Power Meter"
#define PULSE_PIN 2
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
double angularVelocitySum = 0;  // Summe der Winkelgeschwindigkeiten während des Drives
int drivePulseCount = 0;
double dragFactorSum = 0;   // Summe des Drag-Faktors während der Recover-Phase
int recoverPulseCount = 0;  // Anzahl der Pulse in der Recover-Phase

volatile int64_t prevT = 0;
volatile int64_t t = 0;
volatile int64_t dt = 0;
volatile int64_t pulseInTimeEnd = 0;
volatile int64_t pulseInTimeEnd_old = 0;
volatile int pulseInNumber = 0;
volatile bool newPulseDurationAvailable = false;
int debounceTime = 4000;
int subtractFromDebounce = 500;
double driveSum = 0;
double recoverSum = 0;
unsigned long stateStartTime = 0;
unsigned long minStateDuration = 100000;

const int numReadings = 4;
long readings[numReadings];
double avArray[numReadings];
int readIndex = 0;
int avArrayIndex = 0;
double total = 0;
double average = 0;
double averagePrev = 0;
double avInSeconds = 0;
double averagePrevInSeconds = 0;
int numOfAvs = 0;
double strokeInSeconds = 0;
double strokesPerMinute = 0;

enum State { RECOVER,
             DRIVE };
State currentState = DRIVE;
State previousState = DRIVE;

//Nano33BLEMagneticData magneticData;

BLEService CyclePowerService("1818");
BLECharacteristic CyclePowerFeature("2A65", BLERead, 4);
BLECharacteristic CyclePowerMeasurement("2A63", BLERead | BLENotify, 8);
BLECharacteristic CyclePowerSensorLocation("2A5D", BLERead, 1);

unsigned char bleBuffer[8];
unsigned char slBuffer[1];
unsigned char fBuffer[4];

short power;
long random_power;
unsigned short revolutions = 0;
unsigned short timestamp = 0;
unsigned short flags = 0x20;
byte sensorlocation = 0x0D;

double i_diff = 0;

//Configurable values
float mag_power_calib = 100;
double mag_samps_per_sec = 16;
short cap_power = 400;
float decay_factor = 0.5;
float noise_factor = 3;

void buttonPinInterrupt() {
  // Start measuring
  t = esp_timer_get_time();
  dt = t - prevT;

  //Serial.println(dt);

  // Prüfen, ob dt in den erwarteten Bereich fällt
  if (dt < 6000) {
    return;
  }
  prevT = t;

  pulseInNumber++;
  pulseInTimeEnd_old = pulseInTimeEnd;
  pulseInTimeEnd = t;
  newPulseDurationAvailable = true;
}

bool detectDrive(double prev, double cur) {
  // Erkennung, ob der Wert von Recover auf Drive gewechselt hat
  return (prev / cur) < 1;
}

double calculateAngularVelocity(double pulseTime) {
  return (2.0 * PI) / (pulseTime * magnetCount);
}

double calculateDragFactor(double currentPulse, double previousPulse) {
  if (currentPulse < 0.00001 || previousPulse < 0.00001) {
    return initK;
  }
  double deltaDragFactor = A * momOfInertia * (1 - (previousPulse / currentPulse));
  return deltaDragFactor;
}

double calculatePower(double dragFactor, double angularVelocity) {  // Verhindere Division durch Null
  return dragFactor * pow(angularVelocity, 3);                      // Power basierend auf Durchschnitt berechnen
}

void setup() {
  // put your setup code here, to run once:
  /* BLE Setup. For information, search for the many ArduinoBLE examples.*/
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);

  pinMode(PULSE_PIN, INPUT);
  for (int i = 0; i < numReadings; i++) {
    readings[i] = 0;
  }

  attachInterrupt(digitalPinToInterrupt(PULSE_PIN), buttonPinInterrupt, FALLING);

  if (!BLE.begin()) {
    while (1)
      ;
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

      if (newPulseDurationAvailable) {
        numOfAvs++;
        noInterrupts();

        // Update the readings array
        total = total - readings[readIndex];
        readings[readIndex] = abs(pulseInTimeEnd - pulseInTimeEnd_old);
        total = total + readings[readIndex];
        readIndex++;
        if (readIndex >= numReadings) {
          readIndex = 0;
        }

        //interrupts();

        // Calculate average
        averagePrev = average;
        averagePrevInSeconds = averagePrev / 1000000.0;
        average = total / numReadings;
        avInSeconds = average / 1000000.0;

        //interrupts();

        uint64_t currentTime = t;

        if (average <= 53366) {

          avArray[avArrayIndex] = avInSeconds;
          //Serial.print(avArray[avArrayIndex], 6);
          avArrayIndex++;

          if (avArrayIndex >= numReadings) {
            avArrayIndex = 0;
          }

          // Determine current state
          State detectedState;
          if (detectDrive(averagePrev, average)) {
            detectedState = RECOVER;
          } else {
            detectedState = DRIVE;
          }

          // Check if we have been in the same state long enough
          if (detectedState != currentState) {
            // If the state has changed, check if we have been in the current state long enough
            if (currentTime - stateStartTime >= minStateDuration) {
              // We have been in the state long enough to consider it valid
              previousState = currentState;
              currentState = detectedState;
              stateStartTime = currentTime;  // Reset the state start time for the new state
            }
          } else {
            // If the state has not changed, update the start time
            stateStartTime = currentTime;
          }

          // Process the state

          if (currentState == DRIVE) {
            if (previousState == RECOVER && currentState == DRIVE) {
              drivePulseCount += 2;
              angularVelocitySum += calculateAngularVelocity(avArray[(avArrayIndex - 2 + numReadings) % numReadings]);
              angularVelocitySum += calculateAngularVelocity(avArray[(avArrayIndex - 3 + numReadings) % numReadings]);
            }
            drivePulseCount++;
            angularVelocitySum += calculateAngularVelocity(avInSeconds);
            driveSum += avInSeconds;
          }
          if (currentState == RECOVER) {
            recoverPulseCount++;
            if (recoverPulseCount >= magnetCount) {
              dragFactorSum += calculateDragFactor(avInSeconds, averagePrevInSeconds);
              //Serial.print(dragFactorSum, 8);
            }
            recoverSum += avInSeconds;
          }

          if (previousState == RECOVER && currentState == DRIVE) {
            // Sicherstellen, dass wir Pulse haben
            Serial.print(dragFactorSum, 8);
            Serial.print("rec pulse");
            Serial.print(recoverPulseCount);
            dragFactor = (dragFactorSum / (recoverPulseCount - magnetCount));  // Durchschnitt berechnen

            // Ausgabe des neuen Drag-Faktors
            Serial.print("Neuer Drag-Faktor (nach Recover): ");
            Serial.println(dragFactor, 6);

            // Reset der Variablen für den nächsten Recover
            dragFactorSum = 0;
            recoverPulseCount = 0;
          }

          // Check for state change from Recover to Drive
          if (previousState == DRIVE && currentState == RECOVER) {
            angularVelocity = angularVelocitySum / drivePulseCount;
            power = calculatePower(dragFactor, angularVelocity);  // Leistung berechnen
            revolutions++;
            strokeInSeconds = driveSum + recoverSum;
            strokesPerMinute = 60 / strokeInSeconds;
            Serial.print("drivesum: ");
            Serial.print(driveSum);
            Serial.print("rec sum: ");
            Serial.print(recoverSum);
            Serial.print("stroke in sec: ");
            Serial.print(strokeInSeconds);
            Serial.print("strokes per minute: ");
            Serial.print(strokesPerMinute);
            // Ausgabe der berechneten Werte
            Serial.println("Stroke finished!");
            Serial.print("Schläge: ");
            Serial.print(revolutions);
            Serial.print("; Durchschnittliche Winkelgeschwindigkeit: ");
            Serial.print(angularVelocity);
            Serial.print(" ; DragFactor: ");
            Serial.print(dragFactor, 6);
            Serial.print(" ; Power: ");
            Serial.println(power);

            bleBuffer[0] = flags & 0xff;
            bleBuffer[1] = (flags >> 8) & 0xff;
            bleBuffer[2] = power & 0xff;
            bleBuffer[3] = (power >> 8) & 0xff;
            bleBuffer[4] = revolutions & 0xff;
            bleBuffer[5] = (revolutions >> 8) & 0xff;
            bleBuffer[6] = timestamp & 0xff;
            bleBuffer[7] = (timestamp >> 8) & 0xff;

            slBuffer[0] = sensorlocation & 0xff;

            fBuffer[0] = 0x00;
            fBuffer[1] = 0x00;
            fBuffer[2] = 0x00;
            fBuffer[3] = 0x08;

            CyclePowerFeature.writeValue(fBuffer, 4);
            CyclePowerMeasurement.writeValue(bleBuffer, 8);
            CyclePowerSensorLocation.writeValue(slBuffer, 1);

            // Variablen für den nächsten Drive-Zyklus zurücksetzen
            angularVelocitySum = 0;
            drivePulseCount = 0;
            driveSum = 0;
            recoverSum = 0;
          }

          // Print current average value
          Serial.print(currentState == DRIVE ? "Drive:   " : "Recover: ");
          Serial.print("Pulse: ");
          //Serial.print(pulseInNumber);
          Serial.print(numOfAvs);
          Serial.print("; Zeit (s): ");
          Serial.println(avInSeconds, 8);

          previousState = currentState;
        }
        newPulseDurationAvailable = false;

        interrupts();
      }
    }
  }
}