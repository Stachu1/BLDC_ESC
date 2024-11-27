#include <stdlib.h>
#include <stdio.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "usart.h"

#define MIN_TARGET 60                     // Minimum target speed in RPS
#define START_DUTY_CYCLE 100              // Duty cycle for open loop start
#define ALLIGNMENT_DUTY_CYCLE 100         // Duty cycle for the rotor allignment step
#define ALLIGNMENT_DURATION 100           // Duration of the rotor allignment step in ms
#define DEBOUNCE 10                       // BEMF debounce count - default is 10 consecutive readings
#define MAX_DELTA_RPS 100                 // Maximum acceptable delta RPS before restarting the motor

// 3-half bridge high side pins
#define AH PD5
#define BH PD3
#define CH PD2

// 3-half bridge low side pins
#define AL PB3
#define BL PB2
#define CL PB1

// BEMF pins
// Virtual ground -> AIN0 (PD6) Positive input of the comparator
// BEMF_A -> AIN1 (PD7) Negative input of the comparator
#define BEMF_B ADC0D
#define BEMF_C ADC1D

// PID controller constants
#define Kp 0.2f                           // Proportional gain
#define Kd 0.5f                           // Derivative gain


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

  // Set 3-half bridge pins as outputs
  DDRD = (1 << AH) | (1 << BH) | (1 << CH);
  DDRB = (1 << AL) | (1 << BL) | (1 << CL);

  // Timer1: set clock source to clkI/O / 1 (no prescaling)
  TCCR1A = 0;
  TCCR1B = (1 << CS10);

  // Timer2: set clock source to clkI/O / 1 (no prescaling)
  TCCR2A = 0;
  TCCR2B = (1 << CS20);

  // Timer0: 1ms interrupt
  TCCR0A = (1 << WGM01);                  // Set CTC mode
  TCCR0B = (1 << CS01) | (1 << CS00);     // Prescaler 64
  OCR0A = 249;                            // Compare match for 1ms
  TIMSK0 |= (1 << OCIE0A);                // Enable Timer0 compare interrupt
  
  // Analog comparator setting
  ACSR &= ~(1 << ACIE);                   // Disable the Analog Comparator interrupt
  ACSR |= (1 << ACI);                     // Clear the Analog Comparator interrupt flag
  sei();                                  // Enable global interrupts

  while(1) {
    // Update the target speed
    if (update_speed_target()) {

      // Stop the motor if the target speed is 0
      if (rps_target == 0) {
        stop();
        spinning = 0;
      }

      // Start the motor if the target speed is greater than 0
      else if (!spinning && rps_target > 0) {
        start();
        spinning = 1;
        millis_count = 0;
        steps_count = 0;
      }
    }

    // Update RPS & delta_rps every 100ms
    if (millis_count % 100 == 0) {
      delta_rps = rps;
      rps = (steps_count / 4.2);          // Calculate RPS
      delta_rps = rps - delta_rps;        // Calculate delta RPS
      steps_count = 0;

      if (spinning) printf("%.2f\n", rps);


      // Update duty cycle to reach the target speed
      if (spinning) {
        update_duty_cycle();
        set_pwm(duty_cycle);
      }
    }

    // If detect a suddent increase in RPS, stop the motor
    if (delta_rps > MAX_DELTA_RPS && spinning) {
      stop();
      spinning = 0;
      delta_rps = 0;
    }
  }
}


// Timer0 ISR - triggers every 1ms
ISR(TIMER0_COMPA_vect) {
  millis_count++;
}


// TODO: Make is faster - move what can be moved to the main loop
// Analog comparator ISR
ISR (ANALOG_COMP_vect) {
  // Debounce the BEMF signal
  uint8_t move = 1;
  for (int8_t i = 0; i < DEBOUNCE; i++) {
    if (step & 1) {
      if (!(ACSR & (1 << ACO))) {
        move = 0;
        break;
      };
    } else {
      if ((ACSR & (1 << ACO)))  {
        move = 0;
        break;
      };
    }
  }
  if (move) {
    bldc_move();
    step++;
    step %= 6;
    steps_count++;
  }
}


// Read the target speed from the UART and update rps_target
int8_t update_speed_target(void) {
  static char buff[4];                    // Buffer to hold the incoming string (max 3 digits + null terminator)
  static uint8_t index = 0;

  if (UCSR0A & (1 << RXC0)) {             // Check if data is available in the UART buffer
    char c = uart_getchar(NULL);
    if (c == '\n') {
      buff[index] = '\0';                 // Null-terminate the string
      rps_target = (uint16_t)atoi(buff);  // Convert string to uint16_t
      index = 0;                          // Reset buff index
      return 1;                           // rps_target updated
    } else if (index < sizeof(buff) - 1) {
      buff[index] = c;                    // Add character to buff
      index++;                            // Increment buff index
    }
  }
  return 0;                               // rps_target not updated
}


// BLDC motor commutation function
void bldc_move(void) {        
  switch (step) {
    case 0: AH_BL(); BEMF_C_RISING(); break;
    case 1: AH_CL(); BEMF_B_FALLING(); break;
    case 2: BH_CL(); BEMF_A_RISING(); break;
    case 3: BH_AL(); BEMF_C_FALLING(); break;
    case 4: CH_AL(); BEMF_B_RISING(); break;
    case 5: CH_BL(); BEMF_A_FALLING(); break;
  }
}


// TODO: Make an actuall PID controller
// PID controller to update the duty cycle 
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


// Start the motor in open loop
void start(void) {
  ACSR &= ~(1 << ACIE);                   // Disable analog comparator interrupt

  set_pwm(ALLIGNMENT_DUTY_CYCLE);
  CH_BL();
  _delay_ms(ALLIGNMENT_DURATION);

  step = 0;
  duty_cycle = START_DUTY_CYCLE;
  set_pwm(duty_cycle);                    // Set the duty cycle

  uint16_t delay_us = 2e3;
  while (delay_us > 100) {
    bldc_move();
    uint16_t i = delay_us;
    while(i--) _delay_us(1);
    step++;                               // Move to the next step
    step %= 6;
    delay_us -= 20;                       // Reduce the delay    
  }
  ACSR |= (1 << ACIE);                    // Enable analog comparator interrupt
  rps_target = MIN_TARGET;                // Set the target speed
}


// Stop the motor by setting the 3-half bridge pins as inputs
void stop(void) {
  set_pwm(0);
  ACSR &= ~(1 << ACIE);                   // Disable analog comparator interrupt

  // Set 3-half bridge pins to low
  PORTB &= ~((1 << AL) | (1 << BL) | (1 << CL));
  PORTD &= ~((1 << AH) | (1 << BH) | (1 << CH));
}


void BEMF_A_RISING(void) {
  ADCSRB &= ~(1 << ACME);                 // Select AIN1 as comparator negative input
  ACSR |= (1 << ACIS1) | (1 << ACIS0);    // Set interrupt on rising edge
}

void BEMF_A_FALLING(void) {
  ADCSRB &= ~(1 << ACME);                 // Select AIN1 as comparator negative input
  ACSR = (ACSR & ~(1 << ACIS0)) | (1 << ACIS1);  // Set interrupt on falling edge
}

void BEMF_B_RISING(void) {
  ADCSRA &= ~(1 << ADEN);                 // Disable the ADC module
  ADCSRB |= (1 << ACME);
  ADMUX = BEMF_B;                         // Select analog channel BEMF_B as comparator negative input
  ACSR |= (1 << ACIS1) | (1 << ACIS0);    // Set interrupt on rising edge
}

void BEMF_B_FALLING(void) {
  ADCSRA &= ~(1 << ADEN);                 // Disable the ADC module
  ADCSRB |= (1 << ACME);
  ADMUX = BEMF_B;                         // Select analog channel BEMF_B as comparator negative input
  ACSR = (ACSR & ~(1 << ACIS0)) | (1 << ACIS1);   // Set interrupt on falling edge
}

void BEMF_C_RISING(void) {
  ADCSRA &= ~(1 << ADEN);                 // Disable the ADC module
  ADCSRB |= (1 << ACME);
  ADMUX = BEMF_C;                         // Select analog channel BEMF_C as comparator negative input
  ACSR |= (1 << ACIS1) | (1 << ACIS0);    // Set interrupt on rising edge
}

void BEMF_C_FALLING(void) {
  ADCSRA &= ~(1 << ADEN);                 // Disable the ADC module
  ADCSRB |= (1 << ACME);
  ADMUX = BEMF_C;                         // Select analog channel BEMF_C as comparator negative input
  ACSR = (ACSR & ~(1 << ACIS0)) | (1 << ACIS1);   // Set interrupt on falling edge
}


void AH_BL(void) {
  PORTD &= ~((1 << BH) | (1 << CH));      // Set BH CH to low
  PORTD |= (1 << AH);                     // Set AH to high
  TCCR2A = 0;                            // Turn pin 10 (OC1B) PWM ON (pin 9 & pin 11 OFF)
  TCCR1A = 0x21;
}

void AH_CL(void) {
  PORTD &= ~((1 << BH) | (1 << CH));      // Set BH CH to low
  PORTD |= (1 << AH);                     // Set AH to high
  TCCR2A = 0;                            // Turn pin 9 (OC1A) PWM ON (pin 10 & pin 11 OFF)
  TCCR1A = 0x81;
}

void BH_CL(void) {
  PORTD &= ~((1 << AH) | (1 << CH));      // Set AH CH to low
  PORTD |= (1 << BH);                     // Set BH to high
  TCCR2A = 0;                            // Turn pin 9 (OC1A) PWM ON (pin 10 & pin 11 OFF)
  TCCR1A = 0x81;
}

void BH_AL(void) {
  PORTD &= ~((1 << AH) | (1 << CH));      // Set AH CH to low
  PORTD |= (1 << BH);                     // Set BH to high
  TCCR1A = 0;                            // Turn pin 11 (OC2A) PWM ON (pin 9 & pin 10 OFF)
  TCCR2A = 0x81;
}

void CH_AL(void) {
  PORTD &= ~((1 << AH) | (1 << BH));      // Set AH CH to low
  PORTD |= (1 << CH);                     // Set BH to high
  TCCR1A = 0;                            // Turn pin 11 (OC2A) PWM ON (pin 9 & pin 10 OFF)
  TCCR2A = 0x81;
}

void CH_BL(void) {
  PORTD &= ~((1 << AH) | (1 << BH));      // Set AH BH to low
  PORTD |= (1 << CH);                     // Set CH to high
  TCCR2A = 0;                            // Turn pin 10 (OC1B) PWM ON (pin 9 & pin 11 OFF)
  TCCR1A = 0x21;
}


void set_pwm(float duty){
  if (duty > 255) duty = 255;
  if (duty < 0) duty = 0;
  duty += 0.5;
  OCR1A = (uint8_t)duty;                   // Set pin 9  PWM duty cycle
  OCR1B = (uint8_t)duty;                   // Set pin 10 PWM duty cycle
  OCR2A = (uint8_t)duty;                   // Set pin 11 PWM duty cycle
}