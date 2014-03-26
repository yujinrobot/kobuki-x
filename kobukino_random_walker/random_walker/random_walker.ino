#define BUMPER_L_LED  12 // the pin for the left bumper LED
#define BUMPER_C_LED  9  // the pin for the centre bumper LED
#define BUMPER_R_LED  4  // the pin for the right bumper LED
#define SPEED_LED1    8  // the pin for the LED
#define SPEED_LED2    7  // the pin for the LED
#define SPEED_LED3    6  // the pin for the LED
#define SPEED_LED4    5  // the pin for the LED
#define BUMPER_R      0
#define BUMPER_C      1
#define BUMPER_L      2
#define MOTOR_L       10 // the pin for the left motor power
#define MOTOR_R       11 // the pin for the right motor power

int speed_change_dir = 0;
int max_speed = 255;
int min_speed = 0;
int zero_speed = 127;
int speed_step = 42; // how much the speed in/decrease per each button press
int current_speed = zero_speed;

void setup()
{
  pinMode(BUMPER_L_LED, OUTPUT); // tell Arduino LED is an output
  pinMode(BUMPER_C_LED, OUTPUT); // tell Arduino LED is an output
  pinMode(BUMPER_R_LED, OUTPUT); // tell Arduino LED is an output
  pinMode(SPEED_LED1, OUTPUT); // tell Arduino LED is an output
  pinMode(SPEED_LED2, OUTPUT); // tell Arduino LED is an output
  pinMode(SPEED_LED3, OUTPUT); // tell Arduino LED is an output
  pinMode(SPEED_LED4, OUTPUT); // tell Arduino LED is an output
  pinMode(BUMPER_R, INPUT);
  pinMode(BUMPER_C, INPUT);
  pinMode(BUMPER_L, INPUT);
  pinMode(MOTOR_L, OUTPUT); // tell Arduino LED is an output
  pinMode(MOTOR_R, OUTPUT); // tell Arduino LED is an output
  attachInterrupt(0, change_speed, FALLING);
  Serial.begin(9600);
  speed_change_dir = 1;
  randomSeed(analogRead(5));
  digitalWrite(SPEED_LED1, HIGH); // always on
}

int check_bumpers()
{
  int bumps = 0;
  if(analogRead(BUMPER_R) > 100)
  {
    digitalWrite(BUMPER_R_LED, HIGH);
    bumps = bumps + 1;
  }
  else
  {
    digitalWrite(BUMPER_R_LED, LOW);
  }
  if(analogRead(BUMPER_C) > 100)
  {
    digitalWrite(BUMPER_C_LED, HIGH);
    bumps = bumps + 2;
  }
  else
  {
    digitalWrite(BUMPER_C_LED, LOW);
  }
  if(analogRead(BUMPER_L) > 100)
  {
    digitalWrite(BUMPER_L_LED, HIGH);
    bumps = bumps + 4;
  }
  else
  {
    digitalWrite(BUMPER_L_LED, LOW);
  }
  return bumps;
}

/**
 * speed: 0 ... 127 ... 255 = full speed backward ... stop ... full speed forward
 */
void change_speed()
{
  if (current_speed <= (max_speed - speed_step))
  {
    current_speed = current_speed + speed_step;
  }
  else
  {
    current_speed = zero_speed;
  }
  digitalWrite(SPEED_LED1, HIGH); // always on
  if (current_speed >= (zero_speed + speed_step))
  {
    digitalWrite(SPEED_LED2, HIGH);
  }
  else
  {
    digitalWrite(SPEED_LED2, LOW);
  }
  if (current_speed >= (zero_speed + 2 * speed_step))
  {
    digitalWrite(SPEED_LED3, HIGH);
  }
  else
  {
    digitalWrite(SPEED_LED3, LOW);
  }
  if (current_speed >= (max_speed - speed_step))
  {
    digitalWrite(SPEED_LED4, HIGH);
  }
  else
  {
    digitalWrite(SPEED_LED4, LOW);
  } 
}

void loop()
{
  Serial.println("------");
  int bumps = 0;
  bumps = check_bumpers();
  if (bumps > 0)
  {
    Serial.print("Bumps: ");
    Serial.println(bumps);
    // first go backwards a bit
    analogWrite(MOTOR_L, (zero_speed - speed_step));
    analogWrite(MOTOR_R, (zero_speed - speed_step));
    delay(300);
    // then turn
    switch (bumps)
    {
      case 1: // bumper right hit -> turn left a bit
        analogWrite(MOTOR_L, -current_speed);
        analogWrite(MOTOR_R, current_speed);
        delay(random(100, 300));
        break;
      case 2: // bumper middle hit -> turn left or right a bit
        if (random(-1,2) < 0)
        {
          analogWrite(MOTOR_L, -current_speed);
          analogWrite(MOTOR_R, current_speed);
        }
        else
        {
        analogWrite(MOTOR_L, current_speed);
        analogWrite(MOTOR_R, -current_speed);
        }
        delay(random(100, 300));
        break;
      case 3: // bumper right & middle hit -> turn left a bit more
        analogWrite(MOTOR_L, -current_speed);
        analogWrite(MOTOR_R, current_speed);
        delay(random(300, 600));
        break;
      case 4: // bumper left hit -> turn right a bit
        analogWrite(MOTOR_L, current_speed);
        analogWrite(MOTOR_R, -current_speed);
        delay(random(100, 300));
        break;
      case 5: // bumper left & right hit -> turn left or right a lot
        if (random(-1,2) < 0)
        {
          analogWrite(MOTOR_L, -current_speed);
          analogWrite(MOTOR_R, current_speed);
        }
        else
        {
        analogWrite(MOTOR_L, current_speed);
        analogWrite(MOTOR_R, -current_speed);
        }
        delay(random(600, 900));
        break;
      case 6: // bumper left & middle hit -> turn right a bit more 
        analogWrite(MOTOR_L, current_speed);
        analogWrite(MOTOR_R, -current_speed);
        delay(random(300, 600));
        break;
      case 7: // all bumpers hit -> turn left or right a lot
        if (random(-1,2) < 0)
        {
          analogWrite(MOTOR_L, -current_speed);
          analogWrite(MOTOR_R, current_speed);
        }
        else
        {
        analogWrite(MOTOR_L, current_speed);
        analogWrite(MOTOR_R, -current_speed);
        }
        delay(random(600, 900));
        break;
    }
  }
  else
  {
    analogWrite(MOTOR_L, current_speed);
    analogWrite(MOTOR_R, current_speed);
  }
  Serial.print("Speed: ");
  Serial.println(current_speed);
  delay(10);
}

