

#define AVERAGE false

int analogPin = A0; // setting the read pin to A0
int switchPin1 = 43;
long samples = 10;
long val; // assigning val as int
long sum = 0;

void setup() {
  pinMode(49, OUTPUT);                 //Pin 49 is used to enable IO power
  digitalWrite(49, 1);                 //Enable IO power on main CPU board
  pinMode(switchPin1, INPUT);
  pinMode(analogPin, INPUT);

  Serial.begin(9600); // setting the baud rate
  val = analogRead(analogPin);
  delay(1000);
}

void loop() {
  // put your main code here, to run repeatedly:
  val = analogRead(analogPin);
  Serial.print(val);
  Serial.write(13);
  Serial.write(10);
  delay(5);
}
