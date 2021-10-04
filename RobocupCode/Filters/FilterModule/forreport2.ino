class mySense
{
  private:
    byte pin;
    
    int buffCount = 0;
    int buff[CIRCBUFFSIZE];
    
  public:
    mySense(byte pin) {
      pinMode(pin, INPUT);
      buff[0] = analogRead(pin);
    }
    
    long runSum = 0;
    long avg;
    int val;
    
    void poll() {
      val = analogRead(pin);
      
      runSum += val - buff[buffCount];
      
      buff[buffCount] = val;
      buffCount++;
      buffCount = buffCount % CIRCBUFFSIZE;
      
      
      avg = ((runSum*1000)/CIRCBUFFSIZE) / 1000;
    }
    
};