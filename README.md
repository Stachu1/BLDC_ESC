# Sensorless BLDC Motor Controller (ATmega328P)

## ⚙️ Overview

This firmware implements a **sensorless BLDC (Brushless DC) motor controller** for an ATmega328P microcontroller. It uses **back-EMF (BEMF) detection** via the analog comparator and a **6-step commutation** strategy. The project was built as part of a semester project to understand:

- Sensorless BLDC control using BEMF zero crossing
- PWM control and 3-phase bridge driving
- UART-based runtime speed control
- Basic PID-based speed regulation

<img width="1200" height="900" alt="image" src="https://github.com/user-attachments/assets/4a3c6d8c-e2b6-4244-8b7b-9a47122f9b01" />


---

## 🛠 Features

- Sensorless startup using open-loop ramp-up
- Zero-crossing BEMF detection with analog comparator
- PID-based duty cycle control to achieve target RPS
- Regenerative braking support via PWM
- Serial speed control via UART
- Over-speed protection using delta-RPS threshold
- PWM via Timer1 (pins 9/10) and Timer2 (pin 11)
- UART feedback for real-time speed monitoring

---

## 🔌 Pinout Summary

### 3-phase bridge Outputs
- High side: AH (PD5), BH (PD3), CH (PD2)
- Low side:  AL (PB3), BL (PB2), CL (PB1)

### BEMF Sensing
- Virtual ground: AIN0 (PD6)
- BEMF A: AIN1 (PD7)
- BEMF B: ADC0D
- BEMF C: ADC1D

### UART
- Standard ATmega328P UART (TX/RX)

---

## 📋 Serial Interface

- Send speed as integer via UART (e.g., `120\n`) to set RPS
- Send `r\n` to enable regenerative braking mode
- Firmware prints current RPS to UART every 100 ms

---

## 🧪 Control and Protection Parameters

| Parameter               | Value        | Description                               |
|------------------------|--------------|-------------------------------------------|
| `MIN_TARGET`           | 60           | Minimum RPS target                        |
| `MAX_DELTA_RPS`        | 200          | Max RPS change before stopping motor      |
| `START_DUTY_CYCLE`     | 80           | Initial duty cycle for open-loop start    |
| `REGEN_DUTY_CYCLE`     | 192          | Duty cycle used for regenerative braking  |
| `Kp`, `Ki`, `Kd`       | 0.2, 0.0, 0.5| PID gains for speed control               |

---

## 🚀 Startup Sequence

1. Align rotor using fixed phase (CH+BL) and `ALLIGNMENT_DUTY_CYCLE`
2. Ramp up rotor using fixed delay commutation
3. Switch to sensorless mode using BEMF zero-cross detection
4. Begin PID duty cycle control to reach target RPS
