// Linear actuator optical limit sensor reader.
//
// Serial commands:
//   h  print the CSV header
//   s  print current sensor state immediately
//
// Wiring for the first sensor:
//   brown  -> Arduino 5V
//   blue   -> Arduino GND
//   black  -> Arduino D2
//   white  -> disconnected for now
//
// Wiring for the second sensor:
//   brown  -> Arduino 5V
//   blue   -> Arduino GND
//   black  -> Arduino D3
//   white  -> disconnected for now

const unsigned long SERIAL_BAUD = 115200;
const unsigned long SAMPLE_PERIOD_MS = 20;   // 50 Hz status stream
const unsigned long DEBOUNCE_MS = 5;

// Black signal wires.
const byte LEFT_LIMIT_PIN = 2;
const byte RIGHT_LIMIT_PIN = 3;

// White signal wires. These are disabled for initial testing.
const byte LEFT_LIMIT_COMPLEMENT_PIN = 255;
const byte RIGHT_LIMIT_COMPLEMENT_PIN = 255;

// The tested sensors work with the Arduino internal pullups.
const byte LIMIT_INPUT_MODE = INPUT_PULLUP;

// With the tested sensors, a blocked beam reads active.
const byte LIMIT_ACTIVE_LEVEL = LOW;

const byte UNUSED_PIN = 255;

struct DebouncedInput
{
  byte pin;
  bool rawActive;
  bool stableActive;
  bool previousStableActive;
  unsigned long lastRawChangeMs;
};

DebouncedInput leftLimit = {LEFT_LIMIT_PIN, false, false, false, 0};
DebouncedInput rightLimit = {RIGHT_LIMIT_PIN, false, false, false, 0};
DebouncedInput leftComplement = {LEFT_LIMIT_COMPLEMENT_PIN, false, false, false, 0};
DebouncedInput rightComplement = {RIGHT_LIMIT_COMPLEMENT_PIN, false, false, false, 0};

unsigned long lastStatusMs = 0;

bool pinIsUsed(byte pin)
{
  return pin != UNUSED_PIN;
}

bool readActive(byte pin)
{
  if (!pinIsUsed(pin))
  {
    return false;
  }

  return digitalRead(pin) == LIMIT_ACTIVE_LEVEL;
}

void beginInput(DebouncedInput &input)
{
  if (!pinIsUsed(input.pin))
  {
    return;
  }

  pinMode(input.pin, LIMIT_INPUT_MODE);
  input.rawActive = readActive(input.pin);
  input.stableActive = input.rawActive;
  input.previousStableActive = input.stableActive;
  input.lastRawChangeMs = millis();
}

bool updateInput(DebouncedInput &input, unsigned long nowMs)
{
  if (!pinIsUsed(input.pin))
  {
    return false;
  }

  bool rawActive = readActive(input.pin);
  if (rawActive != input.rawActive)
  {
    input.rawActive = rawActive;
    input.lastRawChangeMs = nowMs;
  }

  input.previousStableActive = input.stableActive;

  if ((nowMs - input.lastRawChangeMs) >= DEBOUNCE_MS)
  {
    input.stableActive = input.rawActive;
  }

  return input.stableActive != input.previousStableActive;
}

bool isComplementConsistent(const DebouncedInput &mainInput, const DebouncedInput &complementInput)
{
  if (!pinIsUsed(complementInput.pin))
  {
    return true;
  }

  return mainInput.stableActive != complementInput.stableActive;
}

void printHeader()
{
  Serial.println("time_ms,left_limit,right_limit,left_complement,right_complement,left_pair_ok,right_pair_ok,any_limit");
}

void printState()
{
  bool leftPairOk = isComplementConsistent(leftLimit, leftComplement);
  bool rightPairOk = isComplementConsistent(rightLimit, rightComplement);
  bool anyLimit = leftLimit.stableActive || rightLimit.stableActive;

  Serial.print(millis());
  Serial.print(",");
  Serial.print(leftLimit.stableActive);
  Serial.print(",");
  Serial.print(rightLimit.stableActive);
  Serial.print(",");
  Serial.print(pinIsUsed(leftComplement.pin) ? leftComplement.stableActive : 0);
  Serial.print(",");
  Serial.print(pinIsUsed(rightComplement.pin) ? rightComplement.stableActive : 0);
  Serial.print(",");
  Serial.print(leftPairOk);
  Serial.print(",");
  Serial.print(rightPairOk);
  Serial.print(",");
  Serial.println(anyLimit);
}

void printEvent(const char *name, bool active)
{
  Serial.print("# event,");
  Serial.print(millis());
  Serial.print(",");
  Serial.print(name);
  Serial.print(",");
  Serial.println(active ? "active" : "inactive");
}

void handleSerialCommands()
{
  while (Serial.available() > 0)
  {
    char command = Serial.read();

    if (command == 'h' || command == 'H')
    {
      printHeader();
    }
    else if (command == 's' || command == 'S')
    {
      printState();
    }
  }
}

void setup()
{
  Serial.begin(SERIAL_BAUD);

  beginInput(leftLimit);
  beginInput(rightLimit);
  beginInput(leftComplement);
  beginInput(rightComplement);

  Serial.println("# limit_sensor_reader");
  Serial.println("# Send 'h' to print the CSV header or 's' to print the current state.");
  printHeader();
}

void loop()
{
  unsigned long nowMs = millis();

  handleSerialCommands();

  if (updateInput(leftLimit, nowMs))
  {
    printEvent("left_limit", leftLimit.stableActive);
  }

  if (updateInput(rightLimit, nowMs))
  {
    printEvent("right_limit", rightLimit.stableActive);
  }

  updateInput(leftComplement, nowMs);
  updateInput(rightComplement, nowMs);

  if ((nowMs - lastStatusMs) >= SAMPLE_PERIOD_MS)
  {
    lastStatusMs = nowMs;
    printState();
  }
}
