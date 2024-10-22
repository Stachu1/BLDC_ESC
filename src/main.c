#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdio.h>


// A_HIGH: PD2
// B_HIGH: PD1
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



volatile uint32_t ticks = 0;
volatile uint8_t step = 1;
volatile uint8_t speed_div = 20;


void step_1() {
  PORTD = (1 << B_H) | (1 << A_L);
  // PORTD = (1 << A_L);
}

void step_2() {
  PORTD = (1 << B_H) | (1 << C_L);
  // PORTD = (1 << C_L);
}

void step_3() {
  PORTD = (1 << A_H) | (1 << C_L);
  // PORTD = (1 << C_L);
}

void step_4() {
  PORTD = (1 << A_H) | (1 << B_L);
  // PORTD = (1 << B_L);
}

void step_5() {
  PORTD = (1 << C_H) | (1 << B_L);
  // PORTD = (1 << B_L);
}

void step_6() {
  PORTD = (1 << C_H) | (1 << A_L);
  // PORTD = (1 << A_L);
}


ISR (TIMER1_COMPA_vect) {
  ticks++;
  if (ticks % speed_div == 0) {
    step++;
    if (step == 7) step = 1;
  }
}


int main(void) {
  cli();
  TCCR1B |= (1 << WGM12);
  OCR1A = 255;
  TIMSK1 |= (1 << OCIE1A);
  TCCR1B |= (1 << CS11);
  sei();

  DDRD = 0xFF;

  uint8_t last_step;
  while(1)  {
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