// SELF BALANCE ARDUINO ROBOT WITH STEPPER MOTORS
// License: GPL v2
// Control functions (PID controls, Steppers control...)

// PD controller implementation(Proportional, derivative). DT in seconds
float stabilityPDControl(float DT, float theta, float setTheta,  float Kp, float Kd)
{
  float error;
  float output;

  int16_t data = 0;
  int8_t data_w[2];

  error = setTheta - theta;

  float setThetaLim = constrain((setTheta - setThetaOld), -8, 8);
  output = Kp * error + (Kd * setThetaLim - Kd * (theta - thetaOld)) / DT;

  thetaOld = theta;
  setThetaOld = setTheta;

  /*                                DEBUG                                     */
  //Encode data
  data = int16_t(theta*1000);
  // Prepare data
  data_w[0] = data;
  data_w[1] = data >> 8;
  // Send data - Simulink waints 2 Bytes (int16)
  Serial.write(data_w[0]);
  Serial.write(data_w[1]);

  //Encode data
  data = int16_t(output*1000);
  // Prepare data
  data_w[0] = data;
  data_w[1] = data >> 8;
  // Send data - Simulink waints 2 Bytes (int16)
  Serial.write(data_w[0]);
  Serial.write(data_w[1]);

  // Waits for the transmission of outgoing serial data to complete
  //Serial.flush();
  /*                                DEBUG                                     */

  return (output);
}

// Stability Control with Simulink for testing/debugging. DT in seconds
float stabilityControlWithSimulink(float DT, float input, float setPoint)
{
  int16_t data = 0;
  int8_t data_w[2];
  int8_t incomingByte[4];
  float output = 0.0;

  // We are wating for 2 Bytes (int16) from Simulink
  int i = 0;
  while (Serial.available() > 0) { // if
    // read the incoming byte:
    incomingByte[i] = Serial.read();
    i++;
  }
  // Simulink int16 2 Bytes: 2º 0000 0001 1º 1100 1000 = 456
  data = (incomingByte[1]&0xFF) << 8 | (incomingByte[0]&0xFF);

  // Decode data
  output = float(data*0.001);

  //Encode data
  data = int16_t(input*1000);
  // Prepare data
  data_w[0] = data;
  data_w[1] = data >> 8;
  // Send data - Simulink waints 2 Bytes (int16)
  Serial.write(data_w[0]);
  Serial.write(data_w[1]);

  //Encode data
  data = int16_t(setPoint*1000);
  // Prepare data
  data_w[0] = data;
  data_w[1] = data >> 8;
  // Send data - Simulink waints 2 Bytes (int16)
  Serial.write(data_w[0]);
  Serial.write(data_w[1]);

  // Waits for the transmission of outgoing serial data to complete
  //Serial.flush();

  return (output);
}

// PI controller implementation (Proportional, integral). DT in seconds
float speedPIControl(float DT, int16_t speed, int16_t setSpeed,  float Kp, float Ki)
{
  int16_t error;
  float output;

  error = setSpeed - speed;
  errorSum += constrain(error, -ITERM_MAX_ERROR, ITERM_MAX_ERROR);
  errorSum = constrain(errorSum, -ITERM_MAX, ITERM_MAX);

  output = Kp * error + Ki * errorSum * DT; // DT is in miliseconds...
  return (output);
}


float positionPDControl(long actualPos, long setPointPos, float Kpp, float Kdp, int16_t speedM)
{
  float output;
  float P;

  P = constrain(Kpp * float(setPointPos - actualPos), -115, 115); // Limit command
  output = P + Kdp * float(speedM);
  return (output);
}

// TIMER 1 : STEPPER MOTOR1 SPEED CONTROL
ISR(TIMER1_COMPA_vect)
{
  if (dir_M1 == 0) // If we are not moving we dont generate a pulse
    return;
  // We generate 1us STEP pulse
  SET(PORTE, 6); // STEP MOTOR 1
  //delay_1us();
  if (dir_M1 > 0)
    steps1--;
  else
    steps1++;
  CLR(PORTE, 6);
}
// TIMER 3 : STEPPER MOTOR2 SPEED CONTROL
ISR(TIMER3_COMPA_vect)
{
  if (dir_M2 == 0) // If we are not moving we dont generate a pulse
    return;
  // We generate 1us STEP pulse
  SET(PORTB, 7); // STEP MOTOR 2
  //delay_1us();
  if (dir_M2 > 0)
    steps2--;
  else
    steps2++;
  CLR(PORTB, 7);
}


// Set speed of Stepper Motor1
// tspeed could be positive or negative (reverse)
void setMotorSpeedM1(int16_t tspeed)
{
  long timer_period;
  int16_t speed;

  // Limit max speed?

  // WE LIMIT MAX ACCELERATION of the motors
  if ((speed_M1 - tspeed) > MAX_ACCEL)
    speed_M1 -= MAX_ACCEL;
  else if ((speed_M1 - tspeed) < -MAX_ACCEL)
    speed_M1 += MAX_ACCEL;
  else
    speed_M1 = tspeed;

#if MICROSTEPPING==16
  speed = speed_M1 * 50; // Adjust factor from control output speed to real motor speed in steps/second
#else
  speed = speed_M1 * 25; // 1/8 Microstepping
#endif

  if (speed == 0)
  {
    timer_period = ZERO_SPEED;
    dir_M1 = 0;
  }
  else if (speed > 0)
  {
    timer_period = 2000000 / speed; // 2Mhz timer
    dir_M1 = 1;
    SET(PORTB, 4); // DIR Motor 1 (Forward)
  }
  else
  {
    timer_period = 2000000 / -speed;
    dir_M1 = -1;
    CLR(PORTB, 4); // Dir Motor 1
  }
  if (timer_period > 65535)   // Check for minimun speed (maximun period without overflow)
    timer_period = ZERO_SPEED;

  OCR1A = timer_period;
  // Check  if we need to reset the timer...
  if (TCNT1 > OCR1A)
    TCNT1 = 0;
}

// Set speed of Stepper Motor2
// tspeed could be positive or negative (reverse)
void setMotorSpeedM2(int16_t tspeed)
{
  long timer_period;
  int16_t speed;

  // Limit max speed?

  // WE LIMIT MAX ACCELERATION of the motors
  if ((speed_M2 - tspeed) > MAX_ACCEL)
    speed_M2 -= MAX_ACCEL;
  else if ((speed_M2 - tspeed) < -MAX_ACCEL)
    speed_M2 += MAX_ACCEL;
  else
    speed_M2 = tspeed;

#if MICROSTEPPING==16
  speed = speed_M2 * 50; // Adjust factor from control output speed to real motor speed in steps/second
#else
  speed = speed_M2 * 25; // 1/8 Microstepping
#endif

  if (speed == 0)
  {
    timer_period = ZERO_SPEED;
    dir_M2 = 0;
  }
  else if (speed > 0)
  {
    timer_period = 2000000 / speed; // 2Mhz timer
    dir_M2 = 1;
    CLR(PORTD, 6);   // Dir Motor2 (Forward)
  }
  else
  {
    timer_period = 2000000 / -speed;
    dir_M2 = -1;
    SET(PORTD, 6);  // DIR Motor 2
  }
  if (timer_period > 65535)   // Check for minimun speed (maximun period without overflow)
    timer_period = ZERO_SPEED;

  OCR3A = timer_period;
  // Check  if we need to reset the timer...
  if (TCNT3 > OCR3A)
    TCNT3 = 0;
}
