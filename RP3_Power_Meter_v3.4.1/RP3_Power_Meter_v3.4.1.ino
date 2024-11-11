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
const uint64_t maxTimeBetweenValidPulses = 1000000;  // Maximale Zeit zwischen zwei validen Pulsen (800 ms)
volatile double lastValidDt = 0;
double driveSum = 0;
double recoverSum = 0;
unsigned long stateStartTime = 0;
unsigned long minStateDuration = 200000;
int transitionPulseCount = 0;  // Zählt die Pulse nach Zustandwechsel von Drive zu Recover
unsigned long previousMillis = 0;
const long updateInterval = 1000;
unsigned long lastMeasurementMillis = 0;
unsigned short lastCrankEventTime = 0;
unsigned long currentTimeMicroSeconds = 0;
unsigned long randomTime = random(2000, 3000);
unsigned long prevMillisRandom = 0;

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
int numOfAvs = 0;
double strokeInSeconds = 0;
float strokesPerMinute = 0;

enum State { RECOVER,
             DRIVE };
State currentState = DRIVE;
State previousState = DRIVE;

//Nano33BLEMagneticData magneticData;

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
unsigned short prevTimestamp = 0;
unsigned short timeStampDiff = 0;
unsigned short flags = 0x20;//0b0010000000000000; //0x2000 
// 0x20 stimmt, Zwift erkennt Cadence Sensor
byte sensorlocation = 0x0D;
bool updateValues = false;

void buttonPinInterrupt() {
  t = esp_timer_get_time();
  dt = t - prevT;

  //Serial.println(dt);

  if (dt < debounceTime) {
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

int countValidReadings(long readings[], int arraySize) {
  int count = 0;
  for (int i = 0; i < arraySize; i++) {
    if (readings[i] != 0) {
      count++;
    }
  }
  return count;
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
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(PULSE_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(PULSE_PIN), buttonPinInterrupt, FALLING);

  for (int i = 0; i < numReadings; i++) {
    readings[i] = 0;
  }

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
  slBuffer[0] = sensorlocation & 0xff;

  //fBuffer[0] = 0x00;
  //fBuffer[1] = 0x00;
  //fBuffer[2] = 0x00;
  fBuffer[3] |= 0x08;

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

        uint64_t currentTime = t;

        /*if (average > upperLimit) {
          Serial.println("Upper limit reached");
        }
        */

        //interrupts();

        if (average <= upperLimit) {
          numOfAvs++;

          avArray[avArrayIndex] = avInSeconds;
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

          /*
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
          */
          // Handle state transition logic
          if (previousState == DRIVE && detectedState == RECOVER) {
            if (transitionPulseCount < 6) {
              // Bleibe in Drive, aber zähle weiter Pulse
              transitionPulseCount++;
              currentState = DRIVE;  // Halte den Zustand auf Drive
            } else {
              // Nach 6 Pulsen wechsle endgültig zu Recover
              currentState = RECOVER;
              transitionPulseCount = 0;  // Setze den Zähler zurück
            }
          } else {
            // Normale Zustandsänderung ohne Verzögerung
            currentState = detectedState;
          }

          // Process the state
          if (currentState == DRIVE) {
            if (previousState == RECOVER) {
              /*if (revolutions < 1) {
                drivePulseCount++;
                angularVelocitySum += calculateAngularVelocity(averagePrevInSeconds);
                driveSum += averagePrevInSeconds;
              } else {*/
              drivePulseCount += 2;
              angularVelocitySum += calculateAngularVelocity(avArray[(avArrayIndex - 2 + numReadings) % numReadings]);
              driveSum += avArray[(avArrayIndex - 2 + numReadings) % numReadings];
              angularVelocitySum += calculateAngularVelocity(avArray[(avArrayIndex - 3 + numReadings) % numReadings]);
              driveSum += avArray[(avArrayIndex - 3 + numReadings) % numReadings];
              //}
              /*Serial.print(dragFactorSum, 8);
            Serial.print("rec pulse");
            Serial.print(recoverPulseCount);
            */
              //recoverPulseCount -= 2;
              recoverSum -= avArray[(avArrayIndex - 2 + numReadings) % numReadings];
              recoverSum -= avArray[(avArrayIndex - 3 + numReadings) % numReadings];
              /*
              dragFactorSum -= calculateDragFactor(avArray[(avArrayIndex - 2 + numReadings) % numReadings], avArray[(avArrayIndex - 3 + numReadings) % numReadings]);
              dragFactorSum -= calculateDragFactor(avArray[(avArrayIndex - 3 + numReadings) % numReadings], avArray[(avArrayIndex - 4 + numReadings) % numReadings]);

              */
              dragFactor = (dragFactorSum / (recoverPulseCount - magnetCount));  // Durchschnitt berechnen

              //Serial.print("Neuer Drag-Faktor (nach Recover): ");
              //Serial.println(dragFactor, 6);

              dragFactorSum = 0;
              recoverPulseCount = 0;
            }
            drivePulseCount++;
            angularVelocitySum += calculateAngularVelocity(avInSeconds);
            driveSum += avInSeconds;
          }

          if (currentState == RECOVER) {
            if (previousState == DRIVE) {
              angularVelocity = angularVelocitySum / drivePulseCount;
              power = (float)calculatePower(dragFactor, angularVelocity);  // Leistungsberechnung
              //revolutions++;
              strokeInSeconds = driveSum + recoverSum;
              strokesPerMinute = 60 / (float)strokeInSeconds;

              Serial.print("drivesum: ");
              Serial.print(driveSum, 3);
              Serial.print(" ; rec sum: ");
              Serial.print(recoverSum, 3);
              Serial.print(" ; stroke in sec: ");
              Serial.print(strokeInSeconds, 3);
              Serial.print(" ; strokes per minute: ");
              Serial.print(round(strokesPerMinute));
              //Serial.println("Stroke finished!");
              Serial.print(" ; Schläge: ");
              Serial.print(revolutions);
              //Serial.print("; Durchschnittliche Winkelgeschwindigkeit: ");
              //Serial.print(angularVelocity);
              //Serial.print(" ; DragFactor: ");
              //Serial.print(dragFactor, 6);
              Serial.print(" ; Power: ");
              Serial.print(round(power));

              // Kopiere den float-Wert für Power in den BLE-Puffer
              //memcpy(&bleBuffer[0], &power, sizeof(float));  // 4 Bytes für Power

              // Kopiere den float-Wert für Strokes per Minute in den BLE-Puffer
              //memcpy(&bleBuffer[4], &strokesPerMinute, sizeof(float));  // 4 Bytes für Strokes per Minute
              //timestamp = timestamp + (unsigned short)(i_diff * (1024 / mag_samps_per_sec));
              currentTimeMicroSeconds = esp_timer_get_time();
              //lastCrankEventTime = (currentTimeMicroSeconds / 1000) * 1.024;
              //lastCrankEventTime = (currentTimeMicroSeconds / 1000000) * 1024;
              timestamp += short(strokeInSeconds * 1024.0);

              Serial.print("timestamp: ");
              Serial.println(timestamp);

              // Variablen für den nächsten Drive-Zyklus zurücksetzen
              angularVelocitySum = 0;
              drivePulseCount = 0;
              driveSum = 0;
              recoverSum = 0;
              updateValues = true;
            }
            recoverPulseCount++;
            if (recoverPulseCount >= magnetCount) {  //first 4 pulses ignored, siehe doc
              dragFactorSum += calculateDragFactor(avInSeconds, averagePrevInSeconds);
            }
            recoverSum += avInSeconds;
          }
          /*
          Serial.print(currentState == DRIVE ? "Drive:   " : "Recover: ");
          Serial.print("Pulse: ");
          //Serial.print(pulseInNumber);
          Serial.print(numOfAvs);
          Serial.print("; Zeit (s): ");
          Serial.println(avInSeconds, 8);

*/
          previousState = currentState;
        }
        newPulseDurationAvailable = false;

        interrupts();
      }
      //RANDOMIZED TEST

      if (currentMillis - prevMillisRandom >= randomTime) {
        //revolutions++;
        timestamp += short(randomTime * 1.024);
        randomTime = random(3000, 3500);
        prevMillisRandom = currentMillis;
        power = random(90, 100);
        updateValues = true;
      }
      
      /*
      Serial.print(revolutions);
      Serial.print("; ");
      Serial.println(timestamp);
      */
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
        bleBuffer[4] = revolutions & 0xff;
        bleBuffer[5] = (revolutions >> 8) & 0xff;
        bleBuffer[6] = timestamp & 0xff;
        bleBuffer[7] = (timestamp >> 8) & 0xff;

        CyclePowerMeasurement.writeValue(bleBuffer, 8);
        Serial.print("power: ");
        Serial.print(power);
        Serial.print(" ; revolutions: ");
        Serial.print(revolutions);
        Serial.print(" ; PREVtimestamp: ");
        Serial.print(prevTimestamp);
        Serial.print(" ; timestamp: ");
        Serial.print(timestamp);
        Serial.print("; updateState: ");
        Serial.println(updateValues);

        timeStampDiff = timestamp - prevTimestamp;
        prevTimestamp = timestamp;

        updateValues = false;
      }
    }
  }
}
