#define CIRCBUFFSIZE 50

int analogPin = A1; // setting the read pin
int switchPin1 = 43;
long samples = 10;
long val; // assigning val as int
long sum = 0;
int cirCount = 0;
int medCount = 0;
long mAvg;
long circBuff[CIRCBUFFSIZE];
bool SENDFLAG = false;

void setup() {
  pinMode(49, OUTPUT);                 //Pin 49 is used to enable IO power
  digitalWrite(49, 1);                 //Enable IO power on main CPU board
  pinMode(switchPin1, INPUT);
  pinMode(analogPin, INPUT);

  Serial.begin(9600); // setting the baud rate
  val = analogRead(analogPin);
  delay(100);
}

void loop() {
  // put your main code here, to run repeatedly:
  val = analogRead(analogPin);

  circBuff[cirCount] = val;

  mAvg = moveAvg(circBuff, CIRCBUFFSIZE);

  Serial.print(mAvg);
  Serial.write(13);
  Serial.write(10);

  cirCount++;
  cirCount = cirCount % CIRCBUFFSIZE;
  delay(10);
}



long moveAvg(long* buff, int siz) {
  long avg;
  long sumboi = 0;
  for (int i = 0; i < siz; i++) {
    sumboi += buff[i];
  }
  avg = (100 * sumboi) / siz;
  return avg / 100;
}
