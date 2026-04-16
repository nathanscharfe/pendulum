#include <Wire.h>
#include "AS5600.h"

// Pendulum AS5600 angle reader.
//
// Serial commands:
//   z  zero the current encoder position
//   r  reset wrap tracking without changing the zero
//   h  print the CSV header

AS5600 encoder;

const unsigned long SERIAL_BAUD = 115200;
const unsigned long SAMPLE_PERIOD_US = 10000;  // 100 Hz
const unsigned long I2C_CLOCK_HZ = 400000;

const int AS5600_COUNTS_PER_REV = 4096;
const int AS5600_HALF_REV_COUNTS = AS5600_COUNTS_PER_REV / 2;
const float COUNTS_TO_DEGREES = 360.0f / AS5600_COUNTS_PER_REV;
const float COUNTS_TO_RADIANS = 6.28318530718f / AS5600_COUNTS_PER_REV;

// Set this to the measured mechanical zero if you want a fixed startup zero.
const long STARTUP_ZERO_COUNT = 0;
const bool AUTO_ZERO_ON_START = false;

unsigned long nextSampleUs = 0;
unsigned long lastSampleUs = 0;

int previousRawCount = 0;
long unwrappedCount = 0;
long zeroCount = STARTUP_ZERO_COUNT;
bool havePreviousSample = false;

float previousThetaRad = 0.0f;
float angularVelocityRadPerSec = 0.0f;

void printHeader()
{
  Serial.println("time_ms,connected,magnet_detected,magnet_too_weak,magnet_too_strong,agc,magnitude,raw_count,unwrapped_count,theta_deg,theta_rad,omega_rad_s");
}

int wrapDelta(int currentCount, int previousCount)
{
  int delta = currentCount - previousCount;

  if (delta > AS5600_HALF_REV_COUNTS)
  {
    delta -= AS5600_COUNTS_PER_REV;
  }
  else if (delta < -AS5600_HALF_REV_COUNTS)
  {
    delta += AS5600_COUNTS_PER_REV;
  }

  return delta;
}

void resetTrackingToCurrent()
{
  previousRawCount = encoder.rawAngle();
  unwrappedCount = previousRawCount;
  havePreviousSample = true;
  previousThetaRad = (unwrappedCount - zeroCount) * COUNTS_TO_RADIANS;
  angularVelocityRadPerSec = 0.0f;
}

void zeroCurrentPosition()
{
  if (!havePreviousSample)
  {
    resetTrackingToCurrent();
  }

  zeroCount = unwrappedCount;
  previousThetaRad = 0.0f;
  angularVelocityRadPerSec = 0.0f;
}

void handleSerialCommands()
{
  while (Serial.available() > 0)
  {
    char command = Serial.read();

    if (command == 'z' || command == 'Z')
    {
      zeroCurrentPosition();
      Serial.println("# zeroed current encoder position");
      printHeader();
    }
    else if (command == 'r' || command == 'R')
    {
      resetTrackingToCurrent();
      Serial.println("# reset wrap tracking");
      printHeader();
    }
    else if (command == 'h' || command == 'H')
    {
      printHeader();
    }
  }
}

void setup()
{
  Serial.begin(SERIAL_BAUD);
  Wire.begin();
  Wire.setClock(I2C_CLOCK_HZ);

  encoder.begin();
  encoder.setDirection(AS5600_CLOCK_WISE);

  delay(100);

  int connected = encoder.isConnected();
  Serial.println("# pendulum_angle_reader");
  Serial.print("# AS5600 connected: ");
  Serial.println(connected);
  Serial.println("# Send 'z' to zero the current angle, 'r' to reset wrap tracking, or 'h' to print the header.");

  resetTrackingToCurrent();

  if (AUTO_ZERO_ON_START)
  {
    zeroCurrentPosition();
    Serial.println("# auto-zeroed current startup position");
  }

  printHeader();

  unsigned long nowUs = micros();
  lastSampleUs = nowUs;
  nextSampleUs = nowUs;
}

void loop()
{
  handleSerialCommands();

  unsigned long nowUs = micros();
  if ((long)(nowUs - nextSampleUs) < 0)
  {
    return;
  }

  nextSampleUs += SAMPLE_PERIOD_US;

  int connected = encoder.isConnected();
  int magnetDetected = encoder.detectMagnet();
  int magnetTooWeak = encoder.magnetTooWeak();
  int magnetTooStrong = encoder.magnetTooStrong();
  int agc = encoder.readAGC();
  int magnitude = encoder.readMagnitude();
  int rawCount = encoder.rawAngle();

  if (!havePreviousSample)
  {
    previousRawCount = rawCount;
    unwrappedCount = rawCount;
    havePreviousSample = true;
  }
  else
  {
    unwrappedCount += wrapDelta(rawCount, previousRawCount);
    previousRawCount = rawCount;
  }

  long relativeCount = unwrappedCount - zeroCount;
  float thetaDeg = relativeCount * COUNTS_TO_DEGREES;
  float thetaRad = relativeCount * COUNTS_TO_RADIANS;

  float dtSec = (nowUs - lastSampleUs) / 1000000.0f;
  if (dtSec > 0.0f)
  {
    angularVelocityRadPerSec = (thetaRad - previousThetaRad) / dtSec;
  }

  previousThetaRad = thetaRad;
  lastSampleUs = nowUs;

  Serial.print(millis());
  Serial.print(",");
  Serial.print(connected);
  Serial.print(",");
  Serial.print(magnetDetected);
  Serial.print(",");
  Serial.print(magnetTooWeak);
  Serial.print(",");
  Serial.print(magnetTooStrong);
  Serial.print(",");
  Serial.print(agc);
  Serial.print(",");
  Serial.print(magnitude);
  Serial.print(",");
  Serial.print(rawCount);
  Serial.print(",");
  Serial.print(unwrappedCount);
  Serial.print(",");
  Serial.print(thetaDeg, 4);
  Serial.print(",");
  Serial.print(thetaRad, 6);
  Serial.print(",");
  Serial.println(angularVelocityRadPerSec, 6);
}
