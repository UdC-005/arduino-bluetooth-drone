#include <SoftwareSerial.h>

// HC-05
SoftwareSerial bt(2, 3);   // RX, TX

// Motor PWM pins
#define FL 11
#define FR 5
#define BL 9
#define BR 6

// Current throttle
int speedValue = 0;

// Amount speed changes every 0.2 sec
const int speedStep = 20;

// 200 ms = 0.2 sec
const unsigned long updateInterval = 200;

unsigned long lastUpdate = 0;

// Last Bluetooth command
char lastCmd = 'S';


void setup()
{
  // Motor pins
  pinMode(FL, OUTPUT);
  pinMode(FR, OUTPUT);
  pinMode(BL, OUTPUT);
  pinMode(BR, OUTPUT);

  // Bluetooth
  bt.begin(9600);

  // Start with motors OFF
  stopMotors();
}


void loop()
{
  // -----------------------------
  // Read Bluetooth command
  // -----------------------------

  if (bt.available())
  {
    lastCmd = bt.read();

    Serial.println(lastCmd);
  }
  // -----------------------------
  // Change throttle every 200 ms
  // -----------------------------

  if (millis() - lastUpdate >= updateInterval)
  {
    lastUpdate = millis();

    // Hold FORWARD → increase speed
    if (lastCmd == 'F')
    {
      speedValue += speedStep;

      if (speedValue > 255)
        speedValue = 255;
    }

    // Hold BACKWARD → decrease speed
    if (lastCmd == 'B')
    {
      speedValue -= speedStep;

      if (speedValue < 0)
        speedValue = 0;
    }
  }
  // -----------------------------
  // Normal throttle
  // -----------------------------
  if (lastCmd == 'F' || lastCmd == 'B')
  {
    setMotors(
      speedValue,
      speedValue,
      speedValue,
      speedValue
    );
  }
  // -----------------------------
  // TURN LEFT
  // -----------------------------

  else if (lastCmd == 'L')
  {
    setMotors(
      0,          // FL
      speedValue, // FR
      0,          // BL
      speedValue  // BR
    );
  }
  // -----------------------------
  // TURN RIGHT
  // -----------------------------
  else if (lastCmd == 'R')
  {
    setMotors(
      speedValue, // FL
      0,          // FR
      speedValue, // BL
      0           // BR
    );
  }
  // -----------------------------
  // STOP
  // -----------------------------

  else if (lastCmd == 'S')
  {
    stopMotors();
  }
}
// =================================
// MOTOR FUNCTION
// =================================
void setMotors(int fl, int fr, int bl, int br)
{
  fl = constrain(fl, 0, 255);
  fr = constrain(fr, 0, 255);
  bl = constrain(bl, 0, 255);
  br = constrain(br, 0, 255);

  analogWrite(FL, fl);
  analogWrite(FR, fr);
  analogWrite(BL, bl);
  analogWrite(BR, br);
}
// =================================
// STOP FUNCTION
// =================================
void stopMotors()
{
  analogWrite(FL, 0);
  analogWrite(FR, 0);
  analogWrite(BL, 0);
  analogWrite(BR, 0);
}
