#include <math.h>
#include <stdlib.h>
#include <string.h>

// Arduino #1 linear actuator stepper controller.
//
// Driver wiring:
//   STEP -> Arduino D10
//   DIR  -> Arduino D11
//   ENA  -> Arduino D12
//
// Serial commands end with a newline:
//   h                         print help
//   ?                         print status
//   e 1                       enable driver
//   e 0                       disable driver
//   x                         stop motion immediately
//   p <steps>                 set current position estimate
//   m <steps> <delay_us>      relative move by step count
//   r <steps_per_second>      continuous signed step-rate command

const byte STEP_PIN = 10;
const byte DIR_PIN = 11;
const byte ENA_PIN = 12;

const byte ENABLE_ACTIVE_LEVEL = LOW;
const byte ENABLE_INACTIVE_LEVEL = HIGH;
const byte DIR_POSITIVE_LEVEL = HIGH;
const byte DIR_NEGATIVE_LEVEL = LOW;

const unsigned long SERIAL_BAUD = 115200;
const unsigned long STATUS_PERIOD_MS = 20;
const unsigned int MIN_STEP_DELAY_US = 10;
const unsigned int MAX_STEP_DELAY_US = 10000;

enum MotionMode
{
  MOTION_IDLE,
  MOTION_RELATIVE_MOVE,
  MOTION_CONTINUOUS
};

MotionMode motionMode = MOTION_IDLE;

bool driverEnabled = false;
bool stepPinHigh = false;
bool completeMoveAfterPulse = false;
int motionDirection = 1;

long positionSteps = 0;
long remainingMoveSteps = 0;

unsigned int stepDelayUs = 1000;
unsigned long lastStepToggleUs = 0;
unsigned long lastStatusMs = 0;

char commandBuffer[96];
byte commandIndex = 0;

void setDriverEnabled(bool enabled)
{
  driverEnabled = enabled;
  digitalWrite(ENA_PIN, enabled ? ENABLE_ACTIVE_LEVEL : ENABLE_INACTIVE_LEVEL);
}

void setDirection(int direction)
{
  motionDirection = (direction >= 0) ? 1 : -1;
  digitalWrite(DIR_PIN, motionDirection > 0 ? DIR_POSITIVE_LEVEL : DIR_NEGATIVE_LEVEL);
}

void forceStepLow()
{
  stepPinHigh = false;
  digitalWrite(STEP_PIN, LOW);
}

void stopMotion()
{
  motionMode = MOTION_IDLE;
  remainingMoveSteps = 0;
  completeMoveAfterPulse = false;
  forceStepLow();
}

unsigned int clampDelayUs(long delayUs)
{
  if (delayUs < MIN_STEP_DELAY_US)
  {
    return MIN_STEP_DELAY_US;
  }

  if (delayUs > MAX_STEP_DELAY_US)
  {
    return MAX_STEP_DELAY_US;
  }

  return (unsigned int)delayUs;
}

unsigned int delayFromStepRate(float stepsPerSecond)
{
  float absRate = fabs(stepsPerSecond);

  if (absRate <= 0.0f)
  {
    return MAX_STEP_DELAY_US;
  }

  float delayUs = 500000.0f / absRate;
  return clampDelayUs((long)(delayUs + 0.5f));
}

void beginRelativeMove(long steps, unsigned int delayUs)
{
  if (steps == 0)
  {
    stopMotion();
    return;
  }

  setDirection(steps > 0 ? 1 : -1);
  stepDelayUs = clampDelayUs(delayUs);
  remainingMoveSteps = labs(steps);
  completeMoveAfterPulse = false;
  motionMode = MOTION_RELATIVE_MOVE;
  forceStepLow();
  lastStepToggleUs = micros();
  setDriverEnabled(true);
}

void beginContinuousStepRate(float stepsPerSecond)
{
  if (stepsPerSecond == 0.0f)
  {
    stopMotion();
    return;
  }

  setDirection(stepsPerSecond > 0.0f ? 1 : -1);
  stepDelayUs = delayFromStepRate(stepsPerSecond);
  completeMoveAfterPulse = false;
  motionMode = MOTION_CONTINUOUS;
  forceStepLow();
  lastStepToggleUs = micros();
  setDriverEnabled(true);
}

void printHelp()
{
  Serial.println("# linear_actuator_controller");
  Serial.println("# Commands:");
  Serial.println("# h                         print help");
  Serial.println("# ?                         print status");
  Serial.println("# e 1                       enable driver");
  Serial.println("# e 0                       disable driver");
  Serial.println("# x                         stop motion immediately");
  Serial.println("# p <steps>                 set current position estimate");
  Serial.println("# m <steps> <delay_us>      relative move by step count");
  Serial.println("# r <steps_per_second>      continuous signed step-rate command");
}

void printStatus()
{
  Serial.print("status,");
  Serial.print(millis());
  Serial.print(",");
  Serial.print(driverEnabled);
  Serial.print(",");
  Serial.print(motionMode);
  Serial.print(",");
  Serial.print(positionSteps);
  Serial.print(",");
  Serial.print(remainingMoveSteps);
  Serial.print(",");
  Serial.print(motionDirection);
  Serial.print(",");
  Serial.println(stepDelayUs);
}

void printStatusHeader()
{
  Serial.println("status,time_ms,enabled,motion_mode,position_steps,remaining_move_steps,direction,step_delay_us");
}

void printAck(const char *message)
{
  Serial.print("ack,");
  Serial.println(message);
}

void printError(const char *message)
{
  Serial.print("error,");
  Serial.println(message);
}

void executeCommand(char *line)
{
  char *command = strtok(line, " ,\t\r\n");

  if (command == NULL)
  {
    return;
  }

  if (strcmp(command, "h") == 0 || strcmp(command, "H") == 0)
  {
    printHelp();
    return;
  }

  if (strcmp(command, "?") == 0)
  {
    printStatusHeader();
    printStatus();
    return;
  }

  if (strcmp(command, "e") == 0 || strcmp(command, "E") == 0)
  {
    char *enabledArg = strtok(NULL, " ,\t\r\n");
    if (enabledArg == NULL)
    {
      printError("missing_enable_value");
      return;
    }

    bool enabled = atoi(enabledArg) != 0;
    if (!enabled)
    {
      stopMotion();
    }

    setDriverEnabled(enabled);
    printAck(enabled ? "enabled" : "disabled");
    return;
  }

  if (strcmp(command, "x") == 0 || strcmp(command, "X") == 0)
  {
    stopMotion();
    printAck("stopped");
    return;
  }

  if (strcmp(command, "p") == 0 || strcmp(command, "P") == 0)
  {
    char *positionArg = strtok(NULL, " ,\t\r\n");
    if (positionArg == NULL)
    {
      printError("missing_position_steps");
      return;
    }

    positionSteps = atol(positionArg);
    printAck("position_set");
    return;
  }

  if (strcmp(command, "m") == 0 || strcmp(command, "M") == 0)
  {
    char *stepsArg = strtok(NULL, " ,\t\r\n");
    char *delayArg = strtok(NULL, " ,\t\r\n");
    if (stepsArg == NULL || delayArg == NULL)
    {
      printError("usage_m_steps_delay_us");
      return;
    }

    beginRelativeMove(atol(stepsArg), clampDelayUs(atol(delayArg)));
    printAck("move_started");
    return;
  }

  if (strcmp(command, "r") == 0 || strcmp(command, "R") == 0)
  {
    char *rateArg = strtok(NULL, " ,\t\r\n");
    if (rateArg == NULL)
    {
      printError("missing_steps_per_second");
      return;
    }

    beginContinuousStepRate(atof(rateArg));
    printAck("step_rate_started");
    return;
  }

  printError("unknown_command");
}

void readSerialCommands()
{
  while (Serial.available() > 0)
  {
    char c = Serial.read();

    if (c == '\n' || c == '\r')
    {
      if (commandIndex > 0)
      {
        commandBuffer[commandIndex] = '\0';
        executeCommand(commandBuffer);
        commandIndex = 0;
      }
      continue;
    }

    if (commandIndex < sizeof(commandBuffer) - 1)
    {
      commandBuffer[commandIndex++] = c;
    }
    else
    {
      commandIndex = 0;
      printError("command_too_long");
    }
  }
}

void updateMotion()
{
  if (motionMode == MOTION_IDLE || !driverEnabled)
  {
    return;
  }

  unsigned long nowUs = micros();
  if ((unsigned long)(nowUs - lastStepToggleUs) < stepDelayUs)
  {
    return;
  }

  lastStepToggleUs = nowUs;
  stepPinHigh = !stepPinHigh;
  digitalWrite(STEP_PIN, stepPinHigh ? HIGH : LOW);

  if (stepPinHigh)
  {
    positionSteps += motionDirection;

    if (motionMode == MOTION_RELATIVE_MOVE)
    {
      remainingMoveSteps--;

      if (remainingMoveSteps <= 0)
      {
        completeMoveAfterPulse = true;
      }
    }
  }
  else if (completeMoveAfterPulse)
  {
    stopMotion();
    printAck("move_complete");
  }
}

void setup()
{
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(ENA_PIN, OUTPUT);

  forceStepLow();
  setDirection(1);
  setDriverEnabled(false);

  Serial.begin(SERIAL_BAUD);
  printHelp();
  printStatusHeader();
}

void loop()
{
  readSerialCommands();
  updateMotion();

  unsigned long nowMs = millis();
  if ((nowMs - lastStatusMs) >= STATUS_PERIOD_MS)
  {
    lastStatusMs = nowMs;
    printStatus();
  }
}
