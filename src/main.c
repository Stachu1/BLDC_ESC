#include <stdlib.h>
#include <stdio.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "usart.h"

#define MIN_SPEED_TARGET 40
#define START_DUTY_CYCLE 100

// Define inverter pins
#define AH PD5
#define BH PD4
#define CH PD3

#define AL PB3
#define BL PB2
#define CL PB1


volatile uint8_t step = 0;
volatile uint16_t steps_count = 0;
volatile uint16_t millis_count = 0;
volatile float rps = 0;
volatile uint16_t rps_target = 0;

void update_speed_target(void);

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
  ACSR = (1 << ACI);                        // Disable and clear (flag bit) analog comparator interrupt
  sei();

  while(1) {
    update_speed_target();
    printf("Current target speed: %u\n", rps_target);
    _delay_ms(500);
  }
}


void update_speed_target(void) {
  static char buff[6];                      // buff to hold the incoming string (max 5 digits + null terminator)
  static uint8_t index = 0;

  if (UCSR0A & (1 << RXC0)) {               // Check if data is available in the UART buff
    char c = uart_getchar(NULL);
    if (c == '\n') {
      buff[index] = '\0';                   // Null-terminate the string
      rps_target = (uint16_t)atoi(buff);    // Convert string to uint16_t
      index = 0;                            // Reset buff index
    } else if (index < sizeof(buff) - 1) {
      buff[index] = c;                      // Add character to buff
      index++;                              // Increment buff index
    }
  }
}


// Timer0 ISR - triggers every 1ms
ISR(TIMER0_COMPA_vect) {
  millis_count++;
}


// Analog comparator ISR
ISR (ANALOG_COMP_vect) {
  for (int i = 0; i < 10; i++) {            // BEMF debounce
    if (step & 1) {
      if (!(ACSR & 0x20)) i -= 1;
    } else {
      if ((ACSR & 0x20))  i -= 1;
    }
  }
  bldc_move();
  step++;
  step %= 6;
  steps_count++;
}


void bldc_move() {        // BLDC motor commutation function
  switch (step) {
    case 0: AH_BL(); BEMF_C_RISING(); break;
    case 1: AH_CL(); BEMF_B_FALLING(); break;
    case 2: BH_CL(); BEMF_A_RISING(); break;
    case 3: BH_AL(); BEMF_C_FALLING(); break;
    case 4: CH_AL(); BEMF_B_RISING(); break;
    case 5: CH_BL(); BEMF_A_FALLING(); break;
  }
}


void update_duty_cycle(float *duty_cycle, uint16_t rps_target, uint16_t rps) {
  float Kp = 0.2;  // Proportional gain
  float Kd = 0.5; // Derivative gain

  static float previous_error = 0;

  float error = (float)rps_target - (float)rps;

  float Pout = Kp * error;

  float derivative = error - previous_error;
  float Dout = Kd * derivative;

  previous_error = error;

  float pid_output = Pout + Dout;
  *duty_cycle += pid_output;

  // Clamp duty cycle to valid range (e.g., 0% to 100%)
  if (*duty_cycle > 255) *duty_cycle = 255;
  if (*duty_cycle < 0) *duty_cycle = 0;
}


void start() {
  ACSR &= ~0x08;           // Disable analog comparator interrupt
  DDRD |= 0x38;            // Configure pins 3, 4, and 5 as outputs
  DDRB |= 0x0E;            // Configure pins 9, 10, and 11 as outputs

  // Set an initial boost in PWM for extra torque during startup
  // SET_PWM_DUTY(180);       // High duty cycle for a brief initial torque boost
  // bldc_move();

  // Now set starting PWM to ramp up smoothly
  SET_PWM_DUTY(100);       // Start with a more moderate PWM

  int i = 5000;
  while (i > 100) {
    delayMicroseconds(i);
    bldc_move();
    step++;
    step %= 6;
    i -= 20;                // More gradual reduction in delay
  }
  ACSR |= 0x08;            // Enable analog comparator interrupt
}


void stop() {
  ACSR &= ~0x08;           // Disable analog comparator interrupt
  DDRD &= ~0x38;           // Configure pins 3, 4 and 5 as inputs
  DDRB &= ~0x0E;           // Configure pins 9, 10 and 11 as inputs
}

void BEMF_A_RISING(){
  ADCSRB = (0 << ACME);    // Select AIN1 as comparator negative input
  ACSR |= 0x03;            // Set interrupt on rising edge
}

void BEMF_A_FALLING(){
  ADCSRB = (0 << ACME);    // Select AIN1 as comparator negative input
  ACSR &= ~0x01;           // Set interrupt on falling edge
}

void BEMF_B_RISING(){
  ADCSRA = (0 << ADEN);   // Disable the ADC module
  ADCSRB = (1 << ACME);
  ADMUX = 2;              // Select analog channel 2 as comparator negative input
  ACSR |= 0x03;
}

void BEMF_B_FALLING(){
  ADCSRA = (0 << ADEN);   // Disable the ADC module
  ADCSRB = (1 << ACME);
  ADMUX = 2;              // Select analog channel 2 as comparator negative input
  ACSR &= ~0x01;
}

void BEMF_C_RISING(){
  ADCSRA = (0 << ADEN);   // Disable the ADC module
  ADCSRB = (1 << ACME);
  ADMUX = 3;              // Select analog channel 3 as comparator negative input
  ACSR |= 0x03;
}

void BEMF_C_FALLING(){
  ADCSRA = (0 << ADEN);   // Disable the ADC module
  ADCSRB = (1 << ACME);
  ADMUX = 3;              // Select analog channel 3 as comparator negative input
  ACSR &= ~0x01;
}


void AH_BL() {
  PORTD &= ~(1 << AL | 1 << BL); // Clear bits AL and BL
  PORTD |=  (1 << BL);           // Set bit BL
  TCCR1A =  0;                   // Turn pin 11 (OC2A) PWM ON (pin 9 & pin 10 OFF)
  TCCR2A =  (1 << WGM20 | 1 << COM2A1); // Set PWM settings for TCCR2A
}

void AH_CL() {
  PORTD &= ~(1 << BL | 1 << AL); // Clear bits BL and AL
  PORTD |=  (1 << CL);           // Set bit CL
  TCCR1A =  0;                   // Turn pin 11 (OC2A) PWM ON (pin 9 & pin 10 OFF)
  TCCR2A =  (1 << WGM20 | 1 << COM2A1); // Set PWM settings for TCCR2A
}

void BH_CL() {
  PORTD &= ~(1 << BL | 1 << AL); // Clear bits BL and AL
  PORTD |=  (1 << CL);           // Set bit CL
  TCCR2A =  0;                   // Turn pin 10 (OC1B) PWM ON (pin 9 & pin 11 OFF)
  TCCR1A =  (1 << WGM10 | 1 << COM1B1); // Set PWM settings for TCCR1A
}

void BH_AL() {
  PORTD &= ~(1 << CL | 1 << BL); // Clear bits CL and BL
  PORTD |=  (1 << AL);           // Set bit AL
  TCCR2A =  0;                   // Turn pin 10 (OC1B) PWM ON (pin 9 & pin 11 OFF)
  TCCR1A =  (1 << WGM10 | 1 << COM1B1); // Set PWM settings for TCCR1A
}

void CH_AL() {
  PORTD &= ~(1 << CL | 1 << BL); // Clear bits CL and BL
  PORTD |=  (1 << AL);           // Set bit AL
  TCCR2A =  0;                   // Turn pin 9 (OC1A) PWM ON (pin 10 & pin 11 OFF)
  TCCR1A =  (1 << WGM10 | 1 << COM1A1); // Set PWM settings for TCCR1A
}

void CH_BL() {
  PORTD &= ~(1 << AL | 1 << BL); // Clear bits AL and BL
  PORTD |=  (1 << BL);           // Set bit BL
  TCCR2A =  0;                   // Turn pin 9 (OC1A) PWM ON (pin 10 & pin 11 OFF)
  TCCR1A =  (1 << WGM10 | 1 << COM1A1); // Set PWM settings for TCCR1A
}


void SET_PWM_DUTY(float duty){
  if (duty > 255) duty = 255;
  if (duty < 0) duty = 0;
  duty += 0.5;
  OCR1A  = (uint8_t)duty;                   // Set pin 9  PWM duty cycle
  OCR1B  = (uint8_t)duty;                   // Set pin 10 PWM duty cycle
  OCR2A  = (uint8_t)duty;                   // Set pin 11 PWM duty cycle
}