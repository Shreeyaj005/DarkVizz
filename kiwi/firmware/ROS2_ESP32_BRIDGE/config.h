#ifndef CONFIG_H_
#define CONFIG_H_

#define USE_BASE 

#define USE_GENERIC_MOTOR_DRIVER   // Motor drivers with 1 Direction Pin(INA) and 1 PWM(ENABLE) pin.

#define INDIVIDUAL_PID_GAINS     // Use specific PID coefficients for each motor
//#define IDENTICAL_PID_GAINS     // Use identical PID coefficients for each motor

/* Serial port baud rate */
#define BAUDRATE 115200
#define PID_RATE 30 // Hz
#define MAX_PWM 255


/* PID Parameters */
#ifdef INDIVIDUAL_PID_GAINS    
    #define KP_RIGHT 20
    #define KD_RIGHT 12
    #define KI_RIGHT 0
    #define KO_RIGHT 30

    #define KP_LEFT 18
    #define KD_LEFT 12
    #define KI_LEFT 0
    #define KO_LEFT 30

    #define KP_BACK 18
    #define KD_BACK 12
    #define KI_BACK 0
    #define KO_BACK 30
#endif

#ifdef IDENTICAL_PID_GAINS  
    #define KP 20
    #define KD 12
    #define KI 0
    #define KO 50
#endif


/* Stop all motors if no m/o command arrives for this long (ms).
   ROS sends commands every cycle, so if ROS dies the robot stops. */
#define AUTO_STOP_INTERVAL 500

/* Encoder pins - all of these have internal pull-ups, no external resistors needed.
   Encoder power must be 3.3V (ESP32 pins are NOT 5V tolerant).
   Slot order matches ROS: wheel_1 = slot 0 (LEFT), wheel_2 = slot 1 (RIGHT), wheel_3 = slot 2 (BACK) */
#define encoder0PinA 32      // wheel_1 encoder A
#define encoder0PinB 33      // wheel_1 encoder B

#define encoder1PinA 21      // wheel_2 encoder A
#define encoder1PinB 22      // wheel_2 encoder B

#define encoder2PinA 13      // wheel_3 encoder A
#define encoder2PinB 4       // wheel_3 encoder B

/* Encoder direction: all three wheels use the SAME decoding (kiwi wheels are mounted identically).
   Positive motor PWM must give POSITIVE counts. If a wheel counts backwards, set its sign to -1
   (or swap that encoder's A/B wires). */
#define ENC0_SIGN  1
#define ENC1_SIGN  1
#define ENC2_SIGN  1


/* Motor driver pins (ENA/ENB jumpers stay ON; PWM is applied on the IN pins).
   Driver #1: IN1/IN2 = wheel_2 (RIGHT slot), IN3/IN4 = wheel_1 (LEFT slot)
   Driver #2: IN1/IN2 (here named IN5/IN6) = wheel_3 (BACK slot) */
#ifdef USE_GENERIC_MOTOR_DRIVER
    //#define RIGHT_MOTOR_ENA  14    // PWM
    #define RIGHT_MOTOR_IN1  14    // direction
    #define RIGHT_MOTOR_IN2  25    // direction

    //#define LEFT_MOTOR_ENB   27    // PWM
    #define LEFT_MOTOR_IN3   26    // direction  ← pick free GPIO
    #define LEFT_MOTOR_IN4   27    // direction  ← pick free GPIO

    #define BACK_MOTOR_IN5   18    // direction/PWM (3rd wheel) ← pick free GPIO
    #define BACK_MOTOR_IN6   19    // direction/PWM (3rd wheel) ← pick free GPIO
#endif

#endif
