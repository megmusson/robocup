#ifndef IRfilter
#define IRfilter

#define CIRCBUFFSIZE 5

class mySense
{
private:
  int pin;
  long runSum = 0;
  int buffCount = 0;
  int buff[CIRCBUFFSIZE];
 float k1;
 float k2;
public:
  mySense(int pin, float k1, float k2);
  long avg;
  int val;

  void poll();

  int voltage();
  float Distance();
};

#endif
