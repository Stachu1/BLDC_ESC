#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdio.h>


// A_HIGH: PD2
// B_HIGH: PD4
// C_HIGH: PD7

// A_LOW: PD3
// B_LOW: PD5
// C_LOW: PD6

#define A_H 2
#define B_H 4
#define C_H 7

#define A_L 3
#define B_L 5
#define C_L 6



volatile uint32_t millis = 0;
volatile uint8_t step = 1;
volatile uint8_t speed_div = 32;
uint8_t target_speed_div = 4;


void step_1() {
  DDRD = (1 << B_H) | (1 << A_L);
  PORTD = (1 << A_L);
}

void step_2() {
  DDRD = (1 << B_H) | (1 << C_L);
  PORTD = (1 << C_L);
}

void step_3() {
  DDRD = (1 << A_H) | (1 << C_L);
  PORTD = (1 << C_L);
}

void step_4() {
  DDRD = (1 << A_H) | (1 << B_L);
  PORTD = (1 << B_L);
}

void step_5() {
  DDRD = (1 << C_H) | (1 << B_L);
  PORTD = (1 << B_L);
}

void step_6() {
  DDRD = (1 << C_H) | (1 << A_L);
  PORTD = (1 << A_L);
}


ISR (TIMER0_OVF_vect) {
  millis++;
  if (millis % speed_div == 0) {
    step++;
    if (step == 7) step = 1;
  }
}


int main(void) {
  cli();
  TCCR0B |= 0b00000011;
  TIMSK0 |= 0b00000001;
  sei();

  uint8_t last_step;
  uint32_t last_millis = 0;

  while(1)  {
    if (millis - last_millis > 100) {
      last_millis = millis;
      if (speed_div != target_speed_div) speed_div--;
    }


    if (last_step != step) {
      last_step = step;
      switch (step) {
      case 1:
        step_1();
        break;
      case 2:
        step_2();
        break;
      case 3:
        step_3();
        break;
      case 4:
        step_4();
        break;
      case 5:
        step_5();
        break;
      case 6:
        step_6();
        break;
    }
    }
  }
  return 0;
}