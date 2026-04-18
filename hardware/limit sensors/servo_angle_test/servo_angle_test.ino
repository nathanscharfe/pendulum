#include <Servo.h>

// Simple servo angle test for Arduino #2.
//
// Wiring:
//   servo signal -> Arduino D7
//   servo power  -> external 5V supply recommended
//   servo ground -> Arduino GND and power-supply ground
//
// Serial commands (newline terminated):
//   h           print help
//   ?           print current angle
//   a <deg>     move to angle in degrees (0 to 180)
//   c           move to center (90 deg)
//   l           move to low end (0 deg)
//   u           move to high end (180 deg)
//
// For a gentler manual range search, start near center and then try
// angles like 70, 80, 90, 100, 110 while watching for mechanical binding.

const unsigned long SERIAL_BAUD = 115200;
const byte SERVO_PIN = 7;
const int DEFAULT_ANGLE_DEG = 90;

Servo testServo;
int currentAngleDeg = DEFAULT_ANGLE_DEG;

char commandBuffer[48];
byte commandIndex = 0;

int clampAngleDeg(int angleDeg)
{
  if (angleDeg < 0)
  {
    return 0;
  }

  if (angleDeg > 180)
  {
    return 180;
  }

  return angleDeg;
}

void moveServoTo(int angleDeg)
{
  currentAngleDeg = clampAngleDeg(angleDeg);
  testServo.write(currentAngleDeg);

  Serial.print("ack,angle_deg,");
  Serial.println(currentAngleDeg);
}

void printHelp()
{
  Serial.println("# servo_angle_test");
  Serial.println("# Commands:");
  Serial.println("# h           print help");
  Serial.println("# ?           print current angle");
  Serial.println("# a <deg>     move to angle in degrees (0 to 180)");
  Serial.println("# c           move to center (90 deg)");
  Serial.println("# l           move to low end (0 deg)");
  Serial.println("# u           move to high end (180 deg)");
}

void printStatus()
{
  Serial.print("status,angle_deg,");
  Serial.println(currentAngleDeg);
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
    printStatus();
    return;
  }

  if (strcmp(command, "c") == 0 || strcmp(command, "C") == 0)
  {
    moveServoTo(90);
    return;
  }

  if (strcmp(command, "l") == 0 || strcmp(command, "L") == 0)
  {
    moveServoTo(0);
    return;
  }

  if (strcmp(command, "u") == 0 || strcmp(command, "U") == 0)
  {
    moveServoTo(180);
    return;
  }

  if (strcmp(command, "a") == 0 || strcmp(command, "A") == 0)
  {
    char *angleArg = strtok(NULL, " ,\t\r\n");
    if (angleArg == NULL)
    {
      printError("missing_angle_deg");
      return;
    }

    moveServoTo(atoi(angleArg));
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

void setup()
{
  Serial.begin(SERIAL_BAUD);
  testServo.attach(SERVO_PIN);
  moveServoTo(DEFAULT_ANGLE_DEG);
  printHelp();
  printStatus();
}

void loop()
{
  readSerialCommands();
}
