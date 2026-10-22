#ifndef PINMAP_H
#define PINMAP_H

/*
 * Team-agreed pin map (replaces the provisional Week 6 section 2 mapping).
 * Buddy 1 (communications) has no application GPIO.
 * These constants do not configure hardware.
 */
enum
{
    PIN_MOTOR_LEFT_A = 8,
    PIN_MOTOR_LEFT_B = 9,
    PIN_MOTOR_RIGHT_A = 10,
    PIN_MOTOR_RIGHT_B = 11,
    PIN_ENCODER_LEFT = 27,   /* channel B */
    PIN_ENCODER_RIGHT = 5,   /* channel B */
    PIN_LINE_LEFT = 7,
    PIN_LINE_RIGHT = 0,
    PIN_BARCODE_ADC = 26,    /* ADC0 */
    PIN_IMU_SDA = 2,   /* I2C1, Grove 2 / Maker port */
    PIN_IMU_SCL = 3,
    PIN_SONAR_TRIGGER = 16,
    PIN_SONAR_ECHO = 17,     /* 5 V echo through a voltage divider */
    PIN_SCAN_SERVO = 12,
    PIN_BUTTON_START = 20,
    PIN_BUTTON_STOP = 21
};

enum
{
    BARCODE_ADC_CHANNEL = 0  /* GP26 = ADC input 0 */
};

#endif
