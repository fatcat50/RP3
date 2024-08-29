#include "Arduino.h"
//#include "esp_timer.h"
/* For the bluetooth funcionality */
#include <ArduinoBLE.h>
/* Device name which can be scene in BLE scanning software. */
#define BLE_DEVICE_NAME "RP3 Power Meter"
/* Local name which should pop up when scanning for BLE devices. */
#define BLE_LOCAL_NAME "RP3 Power Meter"
#define PULSE_PIN 2

// Model S configuration
int magnetCount = 4;
int cogTeethCount = 11;
int elBAndTension = 21;
float elBandConstant = 10.7;
float momOfInertia = 0.10880;
float cogWheelRad = 0.014195;
float linkLength = 0.008;
double initK = 0.0001;
double dragFactor = initK;
double A = magnetCount / 2 * PI;
double angularVelocity = 0;

volatile unsigned long pulseInTimeEnd = 0;
volatile unsigned long pulseInTimeEnd_old = 0;
volatile unsigned long pulseInNumber = 0;
volatile bool newPulseDurationAvailable = false;
volatile double debounceTime = 4000;
int subtractFromDebounce = 500;
double driveSum = 0;

const int numReadings = 1;
int readings[numReadings];
int readIndex = 0;
double total = 0;
double average = 0;
double averagePrev = 0;
double avInSeconds = 0;

enum State { RECOVER,
             DRIVE };
State currentState = RECOVER;
State previousState = RECOVER;

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
  // start measuring
  pulseInNumber++;
  pulseInTimeEnd_old = pulseInTimeEnd;
  pulseInTimeEnd = esp_timer_get_time();
  newPulseDurationAvailable = true;
}

bool detectDrive(double prev, double cur) {
  // Erkennung, ob der Wert von Recover auf Drive gewechselt hat
  return (prev / cur) < 1;
}

double calculateAngularVelocity(double pulseTime) {
  return (2.0 * PI) / (pulseTime * magnetCount);
}

double calculateDragFactor(double previousDragFactor, double currentPulse, double previousPulse, int pulseCount) {
  double deltaDragFactor = A * momOfInertia * (1 - previousPulse / currentPulse);
  return (previousDragFactor * (pulseCount - 1) + deltaDragFactor) / pulseCount;
}

double calculatePower(double dragFactor, double angularVelocity) {
  return dragFactor * pow(angularVelocity, 3);
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

  attachInterrupt(digitalPinToInterrupt(PULSE_PIN),
                  buttonPinInterrupt,
                  FALLING);

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

    //Magnetic.begin();
  }
}

void loop() {
  // put your main code here, to run repeatedly:
  unsigned long pulseEnd, pulseEndOld;
  BLEDevice central = BLE.central();

  digitalWrite(LED_BUILTIN, LOW);
  delay(300);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(300);

  if (central) {
    /* 
    If a BLE device is connected, magnetic data will start being read, 
    and the data will be processed
    */
    digitalWrite(LED_BUILTIN, HIGH);
    Serial.print("Verbunden mit: ");
    Serial.println(central.address());
    while (central.connected()) {
      if (newPulseDurationAvailable) {
        noInterrupts();
        pulseEnd = pulseInTimeEnd;
        pulseEndOld = pulseInTimeEnd_old;
        interrupts();

        // Update the readings array
        total = total - readings[readIndex];
        readings[readIndex] = pulseEnd - pulseEndOld;
        total = total + readings[readIndex];
        readIndex++;
        if (readIndex >= numReadings) {
          readIndex = 0;
        }

        // Calculate average
        averagePrev = average;
        average = total / numReadings;
        avInSeconds = average / 1000000.0;

        // Determine current state
        if (detectDrive(averagePrev, average)) {
          currentState = RECOVER;
        } else {
          currentState = DRIVE;
        }

        if (currentState == DRIVE) {
          driveSum += average;
          //Serial.print("Aktuelle Drive-Summe: ");
          //Serial.println(driveSum);
        }

        // Check for state change from Recover to Drive
        if (previousState == DRIVE && currentState == RECOVER) {
          Serial.print("Stroke finished! DriveSum = ");
          Serial.println(driveSum);

          angularVelocity = calculateAngularVelocity(average);
          dragFactor = calculateDragFactor(dragFactor, average, averagePrev, pulseInNumber);
          power = calculatePower(dragFactor, angularVelocity);

          revolutions = revolutions + 1;
          timestamp = timestamp + (unsigned short)(i_diff * (1024 / mag_samps_per_sec));

          Serial.print("Schläge: ");
          Serial.print(revolutions);
          Serial.print("; angularVel: ");
          Serial.print(angularVelocity);
          Serial.print(" ; DragFactor: ");
          Serial.print(dragFactor);
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
        }

        // Print current average value
        //Serial.print("Aktueller Zustand: ");
        if (currentState == DRIVE) {
          Serial.print("Drive:   ");
        } else {
          Serial.print("Recover: ");
        }
        Serial.print("Pulse: ");
        Serial.print(pulseInNumber);
        Serial.print("; Zeit (s): ");
        Serial.println(avInSeconds, 8);

        // Update previous state
        previousState = currentState;

        newPulseDurationAvailable = false;
      }
    }
  }
}