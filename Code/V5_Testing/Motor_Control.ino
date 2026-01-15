void update_motors()
{
  if(((esp_timer_get_time() - motorRsent) > 100) && (esp_timer_get_time() - motorLsent) > 100 && motorRsend == true)
  {
    escR.sendThrottle3D(motorR);
    motorRsend = false;
    motorRsent = esp_timer_get_time();
  }
  if((esp_timer_get_time() - motorLsent) > 100 && (esp_timer_get_time() - motorRsent) > 100 && motorRsend == false)
  {
    escL.sendThrottle3D(motorL);
    motorRsend = true;
    motorLsent = esp_timer_get_time();
  }
}

void spin()
{
  motor_on = false;
  if(reversed == true)
  {
    //pinspeed = map(duty[3], 0, 100, 0, -1000);
    spinspeed = max(map(duty[3], 0, 100, 0, -1000), long(-((acc_rate * rpm) + 200)));
  }
  else
  {
    //spinspeed = map(duty[3], 0, 100, 0, 1000);
    spinspeed = min(map(duty[3], 0, 100, 0, 1000), long(acc_rate * rpm) + 200);
  }
  motorR = spinspeed;
  motorL = spinspeed;
}
void translate()
{
  unsigned long long currentime;
  unsigned long long duration; //defines variable for decel duration in microseconds
  int transpeed; //variable to store movement speed 0-100 from ch2 duty
  currentime = esp_timer_get_time();
  duration = spintimefunct() * float(percentdecel) * 0.01; //duration of total decel pulse
  dtime = currentime - startime; //calculates time it's been since start of decel, will need a way to start the timer when decel initiates
  if(reversed == true)
  {
    spinspeed = max(map(duty[3], 0, 100, 0, -1000), long(-((acc_rate * rpm) + 200)));
    transpeed = abs(map(duty[2], 0, 100, -100, 100));  //-100 to 100 and due to formula, - will automatically switch motor direction without needing separate if statements
  }
  else
  {
    spinspeed = min(map(duty[3], 0, 100, 0, 1000), long(acc_rate * rpm) + 200);
    transpeed = abs(map(duty[2], 0, 100, 100, -100));  //-100 to 100 and due to formula, - will automatically switch motor direction without needing separate if statements
  }
  if(angle > 180)
  {
    if(angle > 180 && angle < 250 && startimeset == false) //resets start time when 300 degrees is hit
    {
      startime = currentime;
      dtime = 0;
      startimeset = true;
    }
    if(angle > 250)
    {
      startimeset = false;
    }
    if(dtime < duration)
    {
      if(reversed == true)
      {
        motorR = map(transpeed, 0, 100, spinspeed, 0);
        motorL = map(transpeed, 0, 100, spinspeed, -1000);
      }
      else
      {
        motorR = map(transpeed, 0, 100, spinspeed, 0);
        motorL = map(transpeed, 0, 100, spinspeed, 1000);
      }
      motor_on = true;
    }
    else
    {
      spin();
    }
  }
  else
  {
    if(angle < 80 && startimeset == false) //resets start time when 120 degrees is hit
    {
      startime = currentime;
      dtime = 0;
      startimeset = true;
    }
    if(angle > 80)
    {
      startimeset = false;
    }
    if(dtime < duration)
    {
      if(reversed == true)
      {
        motorL = map(transpeed, 0, 100, spinspeed, 0);
        motorR = map(transpeed, 0, 100, spinspeed, -1000);
      }
      else
      {
        motorL = map(transpeed, 0, 100, spinspeed, 0);
        motorR = map(transpeed, 0, 100, spinspeed, 1000);
      }
      motor_on = true;
    }
    else
    {
      spin();
    }
  }
}