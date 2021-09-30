#define CIRCBUFFSIZE 30

int rightsensePin = A0;

long samples = 10;
long val; // assigning val as int

long sum = 0;
int cirCount = 0;
int medCount = 0;
long mAvg;
long circBuff[CIRCBUFFSIZE];
bool SENDFLAG = false;

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

mySense rsensr(rightsensePin);

void setup() {
  pinMode(49, OUTPUT);                 //Pin 49 is used to enable IO power
  digitalWrite(49, 1);                 //Enable IO power on main CPU board
  

  Serial.begin(9600); // setting the baud rate
  delay(100);
}

void loop() {
  // put your main code here, to run repeatedly:
  rsensr.poll();

  Serial.print(rsensr.avg);
  Serial.print(',');
  Serial.print(rsensr.val);
  
  Serial.write(13);
  Serial.write(10);
  delay(10);
}
