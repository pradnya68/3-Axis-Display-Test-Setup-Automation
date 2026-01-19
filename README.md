# 3-Axis-Display-Test-Setup-Automation

## Overview
During my internship, I designed and implemented a **fully automated 3-axis display test setup**. The system evaluates touchscreen durability, responsiveness, and performance under repeated taps, providing quantitative data on display robustness. The setup integrates **mechanical servo actuation, motorized movement, embedded control logic, and an interactive TFT GUI**, creating a complete autonomous testing environment.

This platform enables controlled, repeatable testing of displays, simulating real-world tapping conditions and recording each interaction for accurate quality assessment.

## Key Features
- **Automated Servo Taps:** Servo performs precise, repeatable taps on the display.  
- **Motorized Platform:** Motorized movements allow sequential testing of multiple displays.  
- **Interactive TFT GUI:**  
  - Visual highlight of tapped areas.  
  - Tap counter increments in real-time.  
  - Easy monitoring of display response and testing progress.  
- **State-Machine Control:** Motors and servo controlled via non-blocking state machine logic.  
- **IR Sensor Feedback:** Detects surface changes to safely stop motors and prevent collisions.  
- **Serial Communication:** Real-time updates to GUI and logs for monitoring and debugging.  
- **Modular & Scalable:** Supports different display sizes and multiple sequential tests.

## Hardware Components
- Teensy 4.1 microcontroller  
- Servo motor for tapping  
- DC motors for platform movement  
- IR sensors for surface detection  
- TFT display for GUI  
- Relays and LEDs for motor direction control  
- Mounting platform for displays  

## Embedded System Logic
- **State Machine Control:** Manages motor and servo operations based on IR sensor inputs and timing events.  
- **Custom Motor Pulsing:** Motors run in precise ON/OFF cycles for controlled motion.  
- **Relay & Direction Management:** Relays toggle motor directions safely to prevent mechanical stress.  
- **Servo Timing & Pause:** Servo oscillates between defined positions with a pause to simulate realistic tapping.  
- **Safety Mechanisms:** Prevents motors from running if unsafe conditions are detected.  
- **Real-Time Logging:** Serial logs track all events including motor pulses, servo cycles, relay toggling, and tap events.  

## Testing & Operation
- Displays are securely mounted to simulate real-world conditions.  
- Servo performs repeated taps with controlled timing and force.  
- TFT GUI highlights each tap and records counts for quantitative measurement.  
- Multiple displays can be tested sequentially for scalability.  
- Automated operation eliminates human variability, ensuring consistent, repeatable results.

## Outcome & Impact
- Developed a **fully automated and interactive platform** for touchscreen durability testing.  
- Integrated mechanical actuation, motor control, sensor feedback, and GUI for precise, repeatable testing.  
- Engineers can **measure tap durability**, **evaluate display responsiveness**, and **ensure product quality**.  
- Modular and scalable design allows testing of multiple display models efficiently.  
