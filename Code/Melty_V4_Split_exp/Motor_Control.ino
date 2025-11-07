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
  if(reversed == true)
  {
    spinspeed = map(duty[3], 0, 100, 0, -1000);
  }
  else
  {
    spinspeed = map(duty[3], 0, 100, 0, 1000);
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
  duration = spintimefunct() * float(percentdecel *.01); //duration of total decel pulse
  dtime = currentime - startime; //calculates time it's been since start of decel, will need a way to start the timer when decel initiates
  if(reversed == true)
  {
    spinspeed = map(duty[3], 0, 100, 0, -1000);
    transpeed = map(duty[2], 0, 100, -100, 100);  //-100 to 100 and due to formula, - will automatically switch motor direction without needing separate if statements
  }
  else
  {
    spinspeed = map(duty[3], 0, 100, 0, 1000);
    transpeed = map(duty[2], 0, 100, 100, -100);  //-100 to 100 and due to formula, - will automatically switch motor direction without needing separate if statements
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
            motorR = spinspeed * float(1 - float(1 - float(float(cos(2* PI * (float(dtime) / duration)) + 1)/2.0)) * float(transpeed * 0.02)); //creates sine wave motor pulsing
            motorL = spinspeed * float(1 - float(1 - float(float(cos(2* PI * (float(dtime) / duration)) + 1)/2.0)) * float(transpeed * -0.02)); //creates inverse sine wave motor pulsing
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
            motorL = spinspeed * float(1 - float(1 - float(float(cos(2* PI * (float(dtime) / duration)) + 1)/2.0)) * float(transpeed * 0.02)); //creates sine wave motor pulsing
            motorR = spinspeed * float(1 - float(1 - float(float(cos(2* PI * (float(dtime) / duration)) + 1)/2.0)) * float(transpeed * -0.02)); //creates inverse sine wave motor pulsing
          }
          else
          {
            spin();
          }
        }
}