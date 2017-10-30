
/*
class Timer
{
  private:
    unsigned long _lastTime;
    unsigned int _rate;

  public:
    Timer(unsigned int rate);
    void setRate(unsigned int rate);
    unsigned int getRate(void);
    unsigned long timeLapsed();
    void timeReset(void);
    bool timerDone();
    bool timerDoneReset();

};


Timer::Timer(unsigned int rate)
{
  _rate = rate;
  //timeReset();
}

void Timer::setRate(unsigned int rate)
{
  _rate = rate;
}

unsigned int Timer::getRate(void)
{
  return _rate;
}

void Timer::timeReset(void)
{
  _lastTime = millis();
}

unsigned long Timer::timeLapsed()
{
  return millis() - _lastTime;
}

bool Timer::timerDone()
{
  if (timeLapsed() > getRate())
    return true;
  else
    return false;

}

bool Timer::timerDoneReset()
{
  if (timerDone())
  {
    timeReset();
    return true;
  }
  else
    return false;
}

*/
