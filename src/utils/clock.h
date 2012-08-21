#ifndef CLOCK_H_
#define CLOCK_H_

void StartClock();
double GetClock();

#define TIC() double TIC_TIME = GetClock()
#define TIC1() (TIC_TIME = GetClock())
#define TOC() (GetClock() -TIC_TIME)


#endif /* CLOCK_H_ */
