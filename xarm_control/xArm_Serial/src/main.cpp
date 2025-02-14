#include <Arduino.h>
#include <lx16a-servo.h>

LX16ABus servoBus;
LX16AServo servo1(&servoBus, 1); // 0-16800 (0-168 degrees)
LX16AServo servo2(&servoBus, 2); // 0-24000 (0-240 degrees)
LX16AServo servo3(&servoBus, 3); // 0-24000 (0-240 degrees)
LX16AServo servo4(&servoBus, 4); // 0-24000 (0-240 degrees)
LX16AServo servo5(&servoBus, 5); // 0-24000 (0-240 degrees)
LX16AServo servo6(&servoBus, 6); // 0-24000 (0-240 degrees)

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

// The arguments converted to integers
long argi[7];

int pos_cmd[7]; // Servo pos commands

double pos[6] = {0, 0, 0, 0, 0, 0}; // servo positions

void runCommand()
{
  // if (servoBus.available())
  // {
    argi[0] = atoi(argv1);
    argi[1] = atoi(argv2);
    argi[2] = atoi(argv3);
    argi[3] = atoi(argv4);
    argi[4] = atoi(argv5);
    argi[5] = atoi(argv6);
    argi[6] = atoi(argv7);

    switch (cmd)
    {
    case READ_ENCODERS:
      pos[0] = servo1.pos_read();
      pos[1] = servo2.pos_read();
      pos[2] = servo3.pos_read();
      pos[3] = servo4.pos_read();
      pos[4] = servo5.pos_read();
      pos[5] = servo6.pos_read();

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
      servo1.move_time(argi[5], argi[6]);
      servo2.move_time(argi[4], argi[6]);
      servo3.move_time(argi[3], argi[6]);
      servo4.move_time(argi[2], argi[6]);
      servo5.move_time(argi[1], argi[6]);
      servo6.move_time(argi[0], argi[6]);

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
  Serial.begin(230400);
  // put your setup code here, to run once:
  pinMode(LED_BUILTIN, OUTPUT);

  // Serial.println("Beginning Servo Example");
  servoBus.beginOnePinMode(&Serial2, 33);
  servoBus.debug(false);
  servoBus.retry = 3;

  // Reset the servo positions
  servo1.move_time(16000, 1000); // Home: 16000
  servo2.move_time(11000, 1250); // Home: 11000
  servo3.move_time(12000, 1500); // Home: 12000
  servo4.move_time(11750, 2000); // Home: 11750
  servo5.move_time(12250, 2000); // Home: 12250
  servo6.move_time(11750, 2000); // Home: 11750
  delay(2000);

  servo1.move_time(8000, 500);
  delay(500);
  servo1.move_time(16000, 500);
  delay(500);
  servo1.move_time(11000, 500);
  delay(500);

  // Read in servo positions
  pos[0] = servo1.pos_read();
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
