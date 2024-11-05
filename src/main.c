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
volatile uint8_t step_ticks = 20;
volatile uint8_t power_on_ticks = 10;
volatile float duty_cycle = 0.3;


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
  if (ticks % step_ticks == 0) {
    step++;
    if (step == 7) step = 1;
  }
  if (ticks % power_on_ticks == 0) {
    PORTD = 0x00;
  }
}


uint16_t ADC_read(uint8_t channel) {
    // Select the ADC channel (0 to 7)
    ADMUX = (ADMUX & 0xF8) | (channel & 0x07);
    
    // Start the ADC conversion
    ADCSRA |= (1 << ADSC);
    
    // Wait for the conversion to complete
    while (ADCSRA & (1 << ADSC));
    
    // Return the ADC result
    return ADC;
}

void set_div() {
  uint16_t val = ADC_read(0);
  step_ticks = (uint8_t)(val / 10 + 1);
  power_on_ticks = (uint8_t)(step_ticks*duty_cycle);
}


int main(void) {
  cli();
  TCCR1B |= (1 << WGM12);
  OCR1A = 200;
  TIMSK1 |= (1 << OCIE1A);
  TCCR1B |= (1 << CS11);
  sei();

  DDRD = 0xFF;

  // Set the reference voltage to AVcc with external capacitor at AREF pin
  ADMUX |= (1 << REFS0);
  
  // Set the ADC prescaler to 128 for 16 MHz clock (to get 125 kHz ADC clock)
  ADCSRA |= (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
  
  // Enable the ADC
  ADCSRA |= (1 << ADEN);


  uint8_t last_step;
  while(1)  {
    if (last_step != step) {
      last_step = step;
      switch (step) {
      case 1:
        step_1();
        set_div();
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