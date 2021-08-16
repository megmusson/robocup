#define CIRCBUFFSIZE 35

int leftsensepin = A0;
int rightsensepin = A1;
int frontleftsensepin = A5;
int frontrightsensepin = A7;

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
    int pin;
    long runSum = 0;
    int buffCount = 0;
    int buff[CIRCBUFFSIZE];
  public:
    mySense(int pin) {
      this->pin = pin;
      pinMode(pin, INPUT);
      buff[0] = analogRead(pin);
    }
    
    
    long avg;
    int val;
    
    void poll() {
      val = analogRead(pin);
      
      runSum += val - buff[buffCount];
      
      buff[buffCount] = val;
      buffCount++;
      buffCount = buffCount % CIRCBUFFSIZE;

      avg = ((runSum*1000)/CIRCBUFFSIZE) / 1000;
      Serial.print(avg);
    }
    
};

mySense lsensr(leftsensepin);
mySense rsensr(rightsensepin);

mySense flsensr(frontleftsensepin);
mySense frsensr(frontrightsensepin);

void setup() {
  pinMode(49, OUTPUT);                 //Pin 49 is used to enable IO power
  digitalWrite(49, 1);                 //Enable IO power on main CPU board
  

  Serial.begin(9600); // setting the baud rate
  delay(100);
}

void loop() {
  lsensr.poll();
  Serial.print(",");
  rsensr.poll();
  Serial.print("\n");
//  Serial.println(" mV");
  
  delay(10);
}
