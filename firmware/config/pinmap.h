#ifndef PINMAP_H
#define PINMAP_H

/*
 * Provisional Week 6 section 2 mapping. G1 electrical verification is open.
 * These constants do not configure hardware.
 */
enum
{
    PIN_MOTOR_LEFT_A = 8,
    PIN_MOTOR_LEFT_B = 9,
    PIN_MOTOR_RIGHT_A = 10,
    PIN_MOTOR_RIGHT_B = 11,
    PIN_ENCODER_LEFT = 26,
    PIN_ENCODER_RIGHT = 5,
    PIN_LINE_LEFT = 0,
    PIN_LINE_RIGHT = 1,
    PIN_BARCODE_ADC = 28,
    PIN_IMU_SDA = 16,
    PIN_IMU_SCL = 17,
    PIN_SONAR_TRIGGER = 6,
    PIN_SONAR_ECHO = 7,
    PIN_SCAN_SERVO = 12,
    PIN_BUTTON_START = 20,
    PIN_BUTTON_STOP = 21
};

enum
{
    BARCODE_ADC_CHANNEL = 2
};

#endif
