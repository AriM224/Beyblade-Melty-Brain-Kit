void update_channels()
{
  esp_task_wdt_reset();  // feeds the watchdog
  if((millis() - chanstart) > 10)
  {
    IBus.loop(); //triggers instead of interrupts
  for (int channel = 1; channel <= NUM_CHANNELS; channel++) 
  {
    pwm[channel] = IBus.readChannel(channel - 1);
    duty[channel] = map(pwm[channel], 1000, 2000, 0, 100); //channels can be accessed by calling duty[0] for channel 1 etc
  }
  if(IBus.cnt_rec == last_rec)  //failsafe
  {
    rec_gap = millis() - rec_last;
    if(rec_gap > 500)
    {
      duty[1] = 50;
      duty[2] = 50;
      duty[3] = 0;
      duty[4] = 50;
      duty[5] = 100;
      duty[6] = 0;
      rc_status = false;
    }
  }
  else
  {
    last_rec = IBus.cnt_rec;
    rec_last = millis();
    rc_status = true;
  }
  chanstart = millis();
  }
  
}