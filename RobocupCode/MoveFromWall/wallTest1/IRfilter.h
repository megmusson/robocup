#ifndef IRfilter
#define IRfilter

#define CIRCBUFFSIZE 10

class mySense
{
private:
	int pin;
	long runSum = 0;
	int buffCount = 0;
	int buff[CIRCBUFFSIZE];
public:
	mySense(int pin);
	long avg;
	int val;

	void poll();
};

#endif
