#define AVERAGE false
#define BUFFSIZE 10

int analogPin = A1; // setting the read pin to A0
int switchPin1 = 43;
long filtered;
long samples = 10;
long val; // assigning val as int
long sum = 0;
int circCount = 0;

long buffer[BUFFSIZE];

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

  if circCount > BUFFSIZE {
    
    cirCount = 0;
  } else {
    cirCount++;
  }

  filtered = 
  
  writeFiltered(filtered)
  delayMicroseconds(10); 
}



void writeFiltered(int value) {
  Serial.print(val);
  Serial.write(13);
  Serial.write(10); 
}
