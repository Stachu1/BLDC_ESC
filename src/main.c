#include <stdlib.h>
#include <stdio.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "usart.h"

#define MIN_TARGET 40
#define START_DUTY_CYCLE 100
#define DEBOUNCE 10
#define MAX_DELTA_RPS 100

// Define inverter pins
#define AH PD5
#define BH PD4
#define CH PD3

#define AL PB3
#define BL PB2
#define CL PB1

#define Kp 0.2f   // Proportional gain as a float macro
#define Kd 0.5f   // Derivative gain as a float macro

volatile uint8_t step = 0;
volatile uint16_t steps_count = 0;
volatile uint64_t millis_count = 0;
float duty_cycle = 0;
float rps = 0;
float delta_rps = 0;
uint16_t rps_target = 0;
int8_t spinning = 0;

int8_t update_speed_target(void);
void bldc_move(void);
void update_duty_cycle(void);
void start(void);
void stop(void);

void BEMF_A_RISING(void);
void BEMF_A_FALLING(void);
void BEMF_B_RISING(void);
void BEMF_B_FALLING(void);
void BEMF_C_RISING(void);
void BEMF_C_FALLING(void);
void AH_BL(void);
void AH_CL(void);
void BH_CL(void);
void BH_AL(void);
void CH_AL(void);
void CH_BL(void);

void set_pwm(float duty);

int main(void) {
  uart_init();
  io_redirect();

  // Set inverter pins as outputs
  DDRB |= (1 << AL) | (1 << BL) | (1 << CL);
  DDRB |= (1 << AH) | (1 << BH) | (1 << CH);

  // Timer1: set clock source to clkI/O / 1 (no prescaling)
  TCCR1A = 0;
  TCCR1B = (1 << CS10);

  // Timer2: set clock source to clkI/O / 1 (no prescaling)
  TCCR2A = 0;
  TCCR2B = (1 << CS20);

  // Timer0: 1ms interrupt
  TCCR0A = (1 << WGM01);                    // Set CTC mode
  TCCR0B = (1 << CS01) | (1 << CS00);       // Prescaler 64
  OCR0A = 249;                              // Compare match for 1ms
  TIMSK0 |= (1 << OCIE0A);                  // Enable Timer0 compare interrupt
  
  // Analog comparator setting
  ACSR &= ~(1 << ACIE);                     // Disable the Analog Comparator interrupt
  ACSR |= (1 << ACI);                       // Clear the Analog Comparator interrupt flag
  sei();                                    // Enable global interrupts

  while(1) {
    if (update_speed_target()) {
      if (spinning && rps_target == 0) {
        stop();
        spinning = 0;
      }
      else if (!spinning && rps_target > 0) {
        start();
        spinning = 1;
      }
    }

    if (millis_count % 100 == 0) {    // 100ms interval
      delta_rps = rps;
      rps = (steps_count / 4.2);      // Calculate RPS
      delta_rps = rps - delta_rps;    // Calculate delta RPS
      steps_count = 0;

      if (spinning) {
        update_duty_cycle();
        set_pwm(duty_cycle);
      }
    }

    if (delta_rps > MAX_DELTA_RPS) {
      start();
      spinning = 1;
      millis_count = 0;
      steps_count = 0;
    }
  }
}


// Timer0 ISR - triggers every 1ms
ISR(TIMER0_COMPA_vect) {
  millis_count++;
}


// Analog comparator ISR
ISR (ANALOG_COMP_vect) {
  for (int8_t i = 0; i < DEBOUNCE; i++) {            // BEMF debounce
    if (step & 1) {
      if (!(ACSR & (1 << ACO))) i -= 1;
    } else {
      if ((ACSR & (1 << ACO)))  i -= 1;
    }
  }
  bldc_move();
  step++;
  step %= 6;
  steps_count++;
}


int8_t update_speed_target(void) {
  static char buff[6];                      // buff to hold the incoming string (max 5 digits + null terminator)
  static uint8_t index = 0;

  if (UCSR0A & (1 << RXC0)) {               // Check if data is available in the UART buff
    char c = uart_getchar(NULL);
    if (c == '\n') {
      buff[index] = '\0';                   // Null-terminate the string
      rps_target = (uint16_t)atoi(buff);    // Convert string to uint16_t
      if (rps_target < MIN_TARGET) rps_target = MIN_TARGET;
      index = 0;                            // Reset buff index
      return 1;
    } else if (index < sizeof(buff) - 1) {
      buff[index] = c;                      // Add character to buff
      index++;                              // Increment buff index
    }
  }
  return 0;
}


void bldc_move(void) {        // BLDC motor commutation function
  switch (step) {
    case 0: AH_BL(); BEMF_C_RISING(); break;
    case 1: AH_CL(); BEMF_B_FALLING(); break;
    case 2: BH_CL(); BEMF_A_RISING(); break;
    case 3: BH_AL(); BEMF_C_FALLING(); break;
    case 4: CH_AL(); BEMF_B_RISING(); break;
    case 5: CH_BL(); BEMF_A_FALLING(); break;
  }
}


void update_duty_cycle(void) {
  static float previous_error = 0;

  float error = (float)rps_target - rps;

  float Pout = Kp * error;

  float derivative = error - previous_error;
  float Dout = Kd * derivative;

  previous_error = error;

  float pid_output = Pout + Dout;
  duty_cycle += pid_output;
}


void start(void) {
  ACSR &= ~(1 << ACIE);           // Disable analog comparator interrupt
  duty_cycle = START_DUTY_CYCLE;  // Set the duty cycle
  set_pwm(duty_cycle);       // Start with a more moderate PWM

  uint16_t delay_us = 5000;
  while (delay_us > 100) {
    uint16_t i = delay_us;
    while(i--) _delay_us(1);
    bldc_move();
    step++;                 // Move to the next step
    step %= 6;              // Wrap around if necessary
    i -= 20;                // More gradual reduction in delay
  }
  ACSR |= (1 << ACIE);            // Enable analog comparator interrupt
  rps_target = MIN_TARGET;        // Set the target speed
}


void stop(void) {
  ACSR &= ~(1 << ACIE);           // Disable analog comparator interrupt
  // Set inverter pins as inputs
  DDRB &= ~((1 << AL) | (1 << BL) | (1 << CL));
  DDRB &= ~((1 << AH) | (1 << BH) | (1 << CH));
}

void BEMF_A_RISING(void) {
  ADCSRB = (0 << ACME);    // Select AIN1 as comparator negative input
  ACSR |= (1 << ACIS1) | (1 << ACIS0);  // Set interrupt on rising edge
}

void BEMF_A_FALLING(void) {
  ADCSRB = (0 << ACME);    // Select AIN1 as comparator negative input
  ACSR = (ACSR & ~(1 << ACIS0)) | (1 << ACIS1);  // Set interrupt on falling edge
}

void BEMF_B_RISING(void) {
  ADCSRA = (0 << ADEN);   // Disable the ADC module
  ADCSRB = (1 << ACME);
  ADMUX = 2;              // Select analog channel 2 as comparator negative input
  ACSR |= (1 << ACIS1) | (1 << ACIS0);  // Set interrupt on rising edge
}

void BEMF_B_FALLING(void) {
  ADCSRA = (0 << ADEN);   // Disable the ADC module
  ADCSRB = (1 << ACME);
  ADMUX = 2;              // Select analog channel 2 as comparator negative input
  ACSR = (ACSR & ~(1 << ACIS0)) | (1 << ACIS1);  // Set interrupt on falling edge
}

void BEMF_C_RISING(void) {
  ADCSRA = (0 << ADEN);   // Disable the ADC module
  ADCSRB = (1 << ACME);
  ADMUX = 3;              // Select analog channel 3 as comparator negative input
  ACSR |= (1 << ACIS1) | (1 << ACIS0);  // Set interrupt on rising edge
}

void BEMF_C_FALLING(void) {
  ADCSRA = (0 << ADEN);   // Disable the ADC module
  ADCSRB = (1 << ACME);
  ADMUX = 3;              // Select analog channel 3 as comparator negative input
  ACSR = (ACSR & ~(1 << ACIS0)) | (1 << ACIS1);  // Set interrupt on falling edge
}


void AH_BL(void) {
  PORTD &= ~(1 << AL | 1 << BL); // Clear bits AL and BL
  PORTD |=  (1 << BL);           // Set bit BL
  TCCR1A =  0;                   // Turn pin 11 (OC2A) PWM ON (pin 9 & pin 10 OFF)
  TCCR2A =  (1 << WGM20 | 1 << COM2A1); // Set PWM settings for TCCR2A
}

void AH_CL(void) {
  PORTD &= ~(1 << BL | 1 << AL); // Clear bits BL and AL
  PORTD |=  (1 << CL);           // Set bit CL
  TCCR1A =  0;                   // Turn pin 11 (OC2A) PWM ON (pin 9 & pin 10 OFF)
  TCCR2A =  (1 << WGM20 | 1 << COM2A1); // Set PWM settings for TCCR2A
}

void BH_CL(void) {
  PORTD &= ~(1 << BL | 1 << AL); // Clear bits BL and AL
  PORTD |=  (1 << CL);           // Set bit CL
  TCCR2A =  0;                   // Turn pin 10 (OC1B) PWM ON (pin 9 & pin 11 OFF)
  TCCR1A =  (1 << WGM10 | 1 << COM1B1); // Set PWM settings for TCCR1A
}

void BH_AL(void) {
  PORTD &= ~(1 << CL | 1 << BL); // Clear bits CL and BL
  PORTD |=  (1 << AL);           // Set bit AL
  TCCR2A =  0;                   // Turn pin 10 (OC1B) PWM ON (pin 9 & pin 11 OFF)
  TCCR1A =  (1 << WGM10 | 1 << COM1B1); // Set PWM settings for TCCR1A
}

void CH_AL(void) {
  PORTD &= ~(1 << CL | 1 << BL); // Clear bits CL and BL
  PORTD |=  (1 << AL);           // Set bit AL
  TCCR2A =  0;                   // Turn pin 9 (OC1A) PWM ON (pin 10 & pin 11 OFF)
  TCCR1A =  (1 << WGM10 | 1 << COM1A1); // Set PWM settings for TCCR1A
}

void CH_BL(void) {
  PORTD &= ~(1 << AL | 1 << BL); // Clear bits AL and BL
  PORTD |=  (1 << BL);           // Set bit BL
  TCCR2A =  0;                   // Turn pin 9 (OC1A) PWM ON (pin 10 & pin 11 OFF)
  TCCR1A =  (1 << WGM10 | 1 << COM1A1); // Set PWM settings for TCCR1A
}


void set_pwm(float duty){
  if (duty > 255) duty = 255;
  if (duty < 0) duty = 0;
  duty += 0.5;
  OCR1A = (uint8_t)duty;                   // Set pin 9  PWM duty cycle
  OCR1B = (uint8_t)duty;                   // Set pin 10 PWM duty cycle
  OCR2A = (uint8_t)duty;                   // Set pin 11 PWM duty cycle
}