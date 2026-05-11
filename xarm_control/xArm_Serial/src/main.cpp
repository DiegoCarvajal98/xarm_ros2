#include <Arduino.h>
#include "LX16A-bus.h"

LX16A motor1(1, Serial2); // 0-16800 (0-168 degrees)
LX16A motor2(2, Serial2); // 0-24000 (0-240 degrees)
LX16A motor3(3, Serial2); // 0-24000 (0-240 degrees)
LX16A motor4(4, Serial2); // 0-24000 (0-240 degrees)
LX16A motor5(5, Serial2); // 0-24000 (0-240 degrees)
LX16A motor6(6, Serial2); // 0-24000 (0-240 degrees)

#define READ_ENCODERS 'e'
#define MOTOR_SPEEDS 'm'

/* Variable initialization */

// A pair of varibles to help parse serial commands (thanks Fergs)
int arg = 0;
int arg_index = 0;

// Variable to hold an input character
char chr;

// Variable to hold the current single-character command
char cmd;

// Character arrays to hold the first and second arguments
char argv1[16];
char argv2[16];
char argv3[16];
char argv4[16];
char argv5[16];
char argv6[16];
char argv7[16];

// The arguments converted to doubles
double argi[7];

int pos_cmd[7]; // Servo pos commands

double pos[6] = {0, 0, 0, 0, 0, 0}; // servo positions

void runCommand()
{
  // if (servoBus.available())
  // {
    argi[0] = atof(argv1);
    argi[1] = atof(argv2);
    argi[2] = atof(argv3);
    argi[3] = atof(argv4);
    argi[4] = atof(argv5);
    argi[5] = atof(argv6);
    argi[6] = atof(argv7);

    switch (cmd)
    {
    case READ_ENCODERS:
      pos[0] = motor1.getPhysicalAngle();
      pos[1] = motor2.getPhysicalAngle();
      pos[2] = motor3.getPhysicalAngle();
      pos[3] = motor4.getPhysicalAngle();
      pos[4] = motor5.getPhysicalAngle();
      pos[5] = motor6.getPhysicalAngle();

      Serial.print(pos[5]);
      Serial.print(" ");
      Serial.print(pos[4]);
      Serial.print(" ");
      Serial.print(pos[3]);
      Serial.print(" ");
      Serial.print(pos[2]);
      Serial.print(" ");
      Serial.print(pos[1]);
      Serial.print(" ");
      Serial.print(pos[0]);
      Serial.println("\r");
      break;
    case MOTOR_SPEEDS:
      motor1.move(argi[5], argi[6]);
      motor2.move(argi[4], argi[6]);
      motor3.move(argi[3], argi[6]);
      motor4.move(argi[2], argi[6]);
      motor5.move(argi[1], argi[6]);
      motor6.move(argi[0], argi[6]);

      Serial.println("\r");
      break;
    default:
      // Serial.println("InvalidCommand\r");
      break;
    }
  // }
  // else
  // {
  //   // Serial.println("ServoNotConnected\r");
  // }
}

/* Clear the current command parameters */
void resetCommand()
{
  cmd = NULL;
  memset(argv1, 0, sizeof(argv1));
  memset(argv2, 0, sizeof(argv2));
  memset(argv3, 0, sizeof(argv3));
  memset(argv4, 0, sizeof(argv4));
  memset(argv5, 0, sizeof(argv5));
  memset(argv6, 0, sizeof(argv6));

  memset(argi, 0, sizeof(argi));
  arg = 0;
  arg_index = 0;
}

void setup()
{
  Serial.begin(57600);
  Serial2.begin(115200, SERIAL_8N1, 16, 17);

  // Initialize servos
  motor1.initialize();
  motor1.enableTorque();
  motor1.setServoMode();

  motor1.initialize();
  motor1.enableTorque();
  motor1.setServoMode();

  motor3.initialize();
  motor3.enableTorque();
  motor3.setServoMode();

  motor4.initialize();
  motor4.enableTorque();
  motor4.setServoMode();

  motor5.initialize();
  motor5.enableTorque();
  motor5.setServoMode();

  motor6.initialize();
  motor6.enableTorque();
  motor6.setServoMode();

  // Reset the servo positions
  motor1.move(46, 500); // Home: 46
  motor2.move(110, 500); // Home: 110
  motor3.move(120, 500); // Home: 120
  motor4.move(120, 500); // Home: 120
  motor5.move(120, 500); // Home: 120
  motor6.move(120, 500); // Home: 120
  delay(2000);

  motor1.move(50, 500);
  delay(500);
  motor1.move(170, 500);
  delay(500);
  motor1.move(50, 500);
  delay(500);

  // Read in servo positions
  pos[0] = motor1.getPhysicalAngle();
  delay(1000);
}

void loop()
{
  // put your main code here, to run repeatedly:
  if (Serial.available() > 0)
  {
    while (Serial.available() > 0)
    {

      // Read the next character
      chr = Serial.read();

      // Terminate a command with a CR
      if (chr == 13)
      {
        if (arg == 7)
          argv7[arg_index] = NULL;

        runCommand();
        resetCommand();
      }
      // Use spaces to delimit parts of the command
      else if (chr == ' ')
      {
        // Step through the arguments
        switch (arg)
        {
        case 0:
          arg = 1;
          break;

        case 1:
          argv1[arg_index] = NULL;
          arg++;
          arg_index = 0;
          break;

        case 2:
          argv2[arg_index] = NULL;
          arg++;
          arg_index = 0;
          break;

        case 3:
          argv3[arg_index] = NULL;
          arg++;
          arg_index = 0;
          break;

        case 4:
          argv4[arg_index] = NULL;
          arg++;
          arg_index = 0;
          break;

        case 5:
          argv5[arg_index] = NULL;
          arg++;
          arg_index = 0;
          break;

        case 6:
          argv6[arg_index] = NULL;
          arg++;
          arg_index = 0;
          break;

        default:
          break;
        }
        continue;
      }
      else
      {
        switch (arg)
        {
        case 0:
          // First argument is always a single letter command
          cmd = chr;
          break;

        case 1:
          // Subsequent arguments can be more than one character
          argv1[arg_index] = chr;
          arg_index++;
          break;

        case 2:
          argv2[arg_index] = chr;
          arg_index++;
          break;

        case 3:
          argv3[arg_index] = chr;
          arg_index++;
          break;

        case 4:
          argv4[arg_index] = chr;
          arg_index++;
          break;

        case 5:
          argv5[arg_index] = chr;
          arg_index++;
          break;

        case 6:
          argv6[arg_index] = chr;
          arg_index++;
          break;

        case 7:
          argv7[arg_index] = chr;
          arg_index++;
          break;

        default:
          break;
        }
      }
    }
  }
}
