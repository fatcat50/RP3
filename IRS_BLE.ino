#define PULSE_PIN 2
#include "Arduino.h"
/* For the bluetooth funcionality */
#include <ArduinoBLE.h>
/* For the use of the IMU sensor */
//#include "Nano33BLEMagnetic.h"

/* Device name which can be scene in BLE scanning software. */
#define BLE_DEVICE_NAME "RP3 Power Meter"
/* Local name which should pop up when scanning for BLE devices. */
#define BLE_LOCAL_NAME "RP3 Power Meter"

#define CYCLING_POWER_SERVICE_UUID "1826"
#define CYCLING_POWER_MEASUREMENT_UUID "2A63"
#define CYCLING_POWER_CONTROL_POINT_UUID "2A66"
#define CYCLING_POWER_FEATURE_UUID "2A65"
#define CYCLING_POWER_SENSOR_LOCATION_UUID "2A5D"

static uint32_t prevT = 0;
volatile unsigned long pulseInTimeEnd = 0;
volatile unsigned long pulseInTimeEnd_old = 0;
volatile unsigned long pulseInNumber = 0;
volatile bool newPulseDurationAvailable = false;

const int numReadings = 4;
int readings[numReadings];
int readIndex = 0;
int total = 0;
int average = 0;

//Nano33BLEMagneticData magneticData;

BLEService cyclingPowerService(CYCLING_POWER_SERVICE_UUID);
BLECharacteristic powerMeasurementCharacteristic(CYCLING_POWER_MEASUREMENT_UUID, BLERead | BLENotify, 8);
BLECharacteristic powerFeatureCharacteristic(CYCLING_POWER_FEATURE_UUID, BLERead, 4);
BLECharacteristic powerControlPointCharacteristic(CYCLING_POWER_CONTROL_POINT_UUID, BLEWrite, 1);
BLECharacteristic sensorLocationCharacteristic(CYCLING_POWER_SENSOR_LOCATION_UUID, BLERead, 1);

unsigned char bleBuffer[8];
unsigned char slBuffer[1];
unsigned char fBuffer[4];

short power;
long random_power;
unsigned short revolutions = 0;
unsigned short timestamp = 0;
unsigned short flags = 0x20;
byte sensorlocation = 0x0D;

float tm2 = 0;
float tm1 = 0;
float tm0 = 0;

float min_m2 = 100;
float max_m2 = 0;
float min_m1 = 100;
float max_m1 = 0;
float curr_min = 100;
float curr_max = 0;
bool is_static = true;

double i_prev = 0;
double i_curr = 0;
double i_diff = 0;
bool point = false;
double counter = 0;

//Configurable values
float mag_power_calib = 100;
double mag_samps_per_sec = 16;
short cap_power = 400;
float decay_factor = 0.5;
float noise_factor = 3;

// Variablen für Timing
unsigned long lastSendTime = 0;
const unsigned long interval = 2000;  // Intervall von 2 Sekunden

void buttonPinInterrupt() {
  uint32_t t = micros();
  uint32_t dt = t - prevT;
  if (dt < 4000) return;
  prevT = t;

  if (digitalRead(PULSE_PIN) == LOW) {
    // start measuring
    pulseInNumber++;
    pulseInTimeEnd_old = pulseInTimeEnd;
    pulseInTimeEnd = micros();
    newPulseDurationAvailable = true;
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(PULSE_PIN, INPUT);
  pinMode(LED_BUILTIN, OUTPUT);

  if (!BLE.begin()) {
    while (1)
      ;
  } else {
    BLE.setDeviceName(BLE_DEVICE_NAME);
    BLE.setLocalName(BLE_LOCAL_NAME);
    BLE.setAdvertisedService(cyclingPowerService);
    cyclingPowerService.addCharacteristic(powerMeasurementCharacteristic);
    cyclingPowerService.addCharacteristic(powerFeatureCharacteristic);
    cyclingPowerService.addCharacteristic(powerControlPointCharacteristic);
    cyclingPowerService.addCharacteristic(sensorLocationCharacteristic);

    BLE.addService(cyclingPowerService);
    BLE.advertise();

    //Magnetic.begin();
  }
  for (int i = 0; i < numReadings; i++) {
    readings[i] = 0;
  }

  attachInterrupt(digitalPinToInterrupt(PULSE_PIN),
                  buttonPinInterrupt,
                  FALLING);
}

void loop() {
  unsigned long pulseEnd, pulseEndOld;

  BLEDevice central = BLE.central();
  digitalWrite(LED_BUILTIN, HIGH);
  delay(300);
  digitalWrite(LED_BUILTIN, LOW);
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
      unsigned long currentMillis = millis();

      if (newPulseDurationAvailable) {
        noInterrupts();
        pulseEnd = pulseInTimeEnd;
        pulseEndOld = pulseInTimeEnd_old;
        interrupts();
        total = total - readings[readIndex];
        readings[readIndex] = pulseEnd - pulseEndOld;
        total = total + readings[readIndex];
        readIndex++;
        if (readIndex >= numReadings) {
          readIndex = 0;
        }
        average = total / numReadings;

        Serial.print(pulseInNumber);
        Serial.print(",");
        Serial.println(average);

        newPulseDurationAvailable = false;
      }

      if (currentMillis - lastSendTime >= interval) {
        // Speichern des aktuellen Zeitstempels
        lastSendTime = currentMillis;

        revolutions++;
        timestamp = timestamp + (unsigned short)(i_diff * (1024 / mag_samps_per_sec));
        i_prev = i_curr;

        //int sensorValue = analogRead(A0);
        power = average;

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

        powerFeatureCharacteristic.writeValue(fBuffer, 4);
        powerMeasurementCharacteristic.writeValue(bleBuffer, 8);
        sensorLocationCharacteristic.writeValue(slBuffer, 1);

        Serial.print("Leistung gesendet: ");
        Serial.print(power);
        Serial.print(" Watt, Schlagzahl gesendet: ");
        Serial.println(revolutions);
      }
    }
  }
}