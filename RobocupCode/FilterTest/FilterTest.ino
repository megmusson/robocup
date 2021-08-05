#define CIRCBUFFSIZE 200
#define MEDBUFFSIZE 10

int analogPin = A1; // setting the read pin to A0
int switchPin1 = 43;
long filtered;
long samples = 10;
long val; // assigning val as int
long sum = 0;
int cirCount = 0;
int medCount = 0;
long mAvg;
long circBuff[CIRCBUFFSIZE];
long medBuff[MEDBUFFSIZE];
long medVal;
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

  medBuff[medCount] = val;
  medCount++;
  medCount = medCount % MEDBUFFSIZE;
  
  circBuff[cirCount] = medVal;
  cirCount++;
  cirCount = cirCount % CIRCBUFFSIZE;

  mAvg = moveAvg(circBuff, CIRCBUFFSIZE);

  filtered = mAvg;
  if (cirCount == CIRCBUFFSIZE - 1) {
    SENDFLAG = true;
  }

  if (SENDFLAG) {
  writeFiltered(filtered);
  }
  delay(10);
}

void swap(long *p, long *q) {
  long t;

  t = *p;
  *p = *q;
  *q = t;
}

void sort(long a[], long n) {
  long i, j, temp;

  for (i = 0; i < n - 1; i++) {
    for (j = 0; j < n - i - 1; j++) {
      if (a[j] > a[j + 1])
        swap(&a[j], &a[j + 1]);
    }
  }
}

long moveAvg(long* buff, int siz) {
  long avg;
  long sumboi = 0;
  for (int i = 0; i < siz; i++) {
    sumboi = sumboi + buff[i];
  }
  avg = sumboi / siz;
  return avg;
}

long findMedian(long* buff, int siz) {
  int boi;
  sort(buff, siz);
  boi = (siz+1) / 2 - 1;
  return buff[boi];
}

void writeFiltered(long value) {
  Serial.print(val);
  Serial.write(13);
  Serial.write(10);
}
