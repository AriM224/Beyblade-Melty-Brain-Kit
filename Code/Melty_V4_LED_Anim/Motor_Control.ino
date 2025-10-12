void update_motors()
{
  if(((esp_timer_get_time() - motor1sent) > 100) && (esp_timer_get_time() - motor2sent) > 100 && motor1send == true)
  {
    esc1.sendThrottle3D(motor1);
    motor1send = false;
    motor1sent = esp_timer_get_time();
  }
  if((esp_timer_get_time() - motor2sent) > 100 && (esp_timer_get_time() - motor1sent) > 100 && motor1send == false)
  {
    esc2.sendThrottle3D(motor2);
    motor1send = true;
    motor2sent = esp_timer_get_time();
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
  motor1 = spinspeed;
  motor2 = spinspeed;
}
void translate()
{
  currentime = esp_timer_get_time();
  duration = spintime * float(percentdecel *.01); //duration of total decel pulse
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
            motor1 = spinspeed * float(1 - float(1 - float(float(cos(2* PI * (float(dtime) / duration)) + 1)/2.0)) * float(transpeed * 0.01)); //creates sine wave motor pulsing
            motor2 = spinspeed * float(1 - float(1 - float(float(cos(2* PI * (float(dtime) / duration)) + 1)/2.0)) * float(transpeed * -0.01)); //creates inverse sine wave motor pulsing
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
            motor2 = spinspeed * float(1 - float(1 - float(float(cos(2* PI * (float(dtime) / duration)) + 1)/2.0)) * float(transpeed * 0.01)); //creates sine wave motor pulsing
            motor1 = spinspeed * float(1 - float(1 - float(float(cos(2* PI * (float(dtime) / duration)) + 1)/2.0)) * float(transpeed * -0.01)); //creates inverse sine wave motor pulsing
          }
          else
          {
            spin();
          }
        }
}