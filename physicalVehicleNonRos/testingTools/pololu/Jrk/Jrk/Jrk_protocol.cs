using System;
using System.Runtime.InteropServices;

namespace Pololu.Jrk
{
    /* protocol.h:  This file defines constants needed to 
       communicate with umc01a (a.k.a. jrk 21v3) via
       USB, USB serial, or TTL serial. */

    public enum jrkRequest
    {
        REQUEST_GET_PARAMETER = 0x81,
        REQUEST_SET_PARAMETER = 0x82,
        REQUEST_GET_VARIABLES = 0x83,
        REQUEST_SET_TARGET = 0x84,
        REQUEST_CLEAR_ERRORS = 0x86,
        REQUEST_MOTOR_OFF = 0x87,
        REQUEST_REINITIALIZE = 0x90,
        REQUEST_START_BOOTLOADER = 0xFF
    }

    [StructLayout(LayoutKind.Sequential, Pack=1)]
    public struct jrkVariables
    {
        public UInt16 input;
        public UInt16 target;
        public UInt16 feedback;
        public UInt16 scaledFeedback;
        public Int16 errorSum;
        public Int16 dutyCycleTarget;
        public Int16 dutyCycle;
        public Byte current;
        public Byte pidPeriodExceeded;
        public UInt16 pidPeriodCount;
        public UInt16 errorFlagBits;
        public UInt16 errorOccurredBits;
    }

    public struct jrkVariablesWithTime
    {
        public int time;
        public jrkVariables vars;
    }

    public enum jrkParameter : byte
    {
        PARAMETER_INITIALIZED = 0, // 1 bit boolean value
        PARAMETER_INPUT_MODE = 1, // 1 byte unsigned value.  Valid values are INPUT_MODE_*.  Init parameter.
        PARAMETER_INPUT_MINIMUM = 2, // 2 byte unsigned value (0-4095)
        PARAMETER_INPUT_MAXIMUM = 6, // 2 byte unsigned value (0-4095)
        PARAMETER_OUTPUT_MINIMUM = 8, // 2 byte unsigned value (0-4095)
        PARAMETER_OUTPUT_NEUTRAL = 10, // 2 byte unsigned value (0-4095)
        PARAMETER_OUTPUT_MAXIMUM = 12, // 2 byte unsigned value (0-4095)
        PARAMETER_INPUT_INVERT = 16, // 1 bit boolean value
        PARAMETER_INPUT_SCALING_DEGREE = 17, // 1 bit boolean value
        PARAMETER_INPUT_POWER_WITH_AUX = 18, // 1 bit boolean value
        PARAMETER_INPUT_ANALOG_SAMPLES_EXPONENT = 20, // 1 byte unsigned value, 0-8 - averages together 4 * 2^x samples
        PARAMETER_INPUT_DISCONNECT_MINIMUM = 22, // 2 byte unsigned value (0-4095)
        PARAMETER_INPUT_DISCONNECT_MAXIMUM = 24, // 2 byte unsigned value (0-4095)    
        PARAMETER_INPUT_NEUTRAL_MAXIMUM = 26, // 2 byte unsigned value (0-4095)
        PARAMETER_INPUT_NEUTRAL_MINIMUM = 28, // 2 byte unsigned value (0-4095)

        PARAMETER_SERIAL_MODE = 30, // 1 byte unsigned value.  Valid values are SERIAL_MODE_*.  MUST be SERIAL_MODE_USB_DUAL_PORT if INPUT_MODE!=INPUT_MODE_SERIAL.  Init variable.
        PARAMETER_SERIAL_FIXED_BAUD_RATE = 31, // 2-byte unsigned value; 0 means autodetect.  Init parameter.
        PARAMETER_SERIAL_TIMEOUT = 34, // 2-byte unsigned value
        PARAMETER_SERIAL_ENABLE_CRC = 36, // 1 bit boolean value
        PARAMETER_SERIAL_NEVER_SUSPEND = 37, // 1 bit boolean value
        PARAMETER_SERIAL_DEVICE_NUMBER = 38, // 1 byte unsigned value, 0-127

        PARAMETER_FEEDBACK_MODE = 50, // 1 byte unsigned value.  Valid values are FEEDBACK_MODE_*.  Init parameter.
        PARAMETER_FEEDBACK_MINIMUM = 51, // 2 byte unsigned value
        PARAMETER_FEEDBACK_MAXIMUM = 53, // 2 byte unsigned value
        PARAMETER_FEEDBACK_INVERT = 55, // 1 bit boolean value
        PARAMETER_FEEDBACK_POWER_WITH_AUX = 57, // 1 bit boolean value
        PARAMETER_FEEDBACK_DEAD_ZONE = 58, // 1 byte unsigned value
        PARAMETER_FEEDBACK_ANALOG_SAMPLES_EXPONENT = 59, // 1 byte unsigned value, 0-8 - averages together 4 * 2^x samples
        PARAMETER_FEEDBACK_DISCONNECT_MINIMUM = 61, // 2 byte unsigned value (0-4095)
        PARAMETER_FEEDBACK_DISCONNECT_MAXIMUM = 63, // 2 byte unsigned value (0-4095)

        PARAMETER_PROPORTIONAL_MULTIPLIER = 70, // 2 byte unsigned value (0-1023)
        PARAMETER_PROPORTIONAL_EXPONENT = 72, // 1 byte unsigned value (0-15)
        PARAMETER_INTEGRAL_MULTIPLIER = 73, // 2 byte unsigned value (0-1023)
        PARAMETER_INTEGRAL_EXPONENT = 75, // 1 byte unsigned value (0-15)
        PARAMETER_DERIVATIVE_MULTIPLIER = 76, // 2 byte unsigned value (0-1023)
        PARAMETER_DERIVATIVE_EXPONENT = 78, // 1 byte unsigned value (0-15)
        PARAMETER_PID_PERIOD = 79, // 2 byte unsigned value
        PARAMETER_PID_INTEGRAL_LIMIT = 81, // 2 byte unsigned value
        PARAMETER_PID_RESET_INTEGRAL = 84, // 1 bit boolean value

        PARAMETER_MOTOR_PWM_FREQUENCY = 100, // 1 byte unsigned value.  Valid values are MOTOR_PWM_FREQUENCY.  Init parameter.
        PARAMETER_MOTOR_INVERT = 101, // 1 bit boolean value

        // WARNING: The EEPROM initialization assumes the 5 parameters below are consecutive!
        PARAMETER_MOTOR_MAX_DUTY_CYCLE_WHILE_FEEDBACK_OUT_OF_RANGE = 102, // 2 byte unsigned value (0-600)
        PARAMETER_MOTOR_MAX_ACCELERATION_FORWARD = 104, // 2 byte unsigned value (1-600)
        PARAMETER_MOTOR_MAX_ACCELERATION_REVERSE = 106, // 2 byte unsigned value (1-600)
        PARAMETER_MOTOR_MAX_DUTY_CYCLE_FORWARD = 108, // 2 byte unsigned value (0-600)
        PARAMETER_MOTOR_MAX_DUTY_CYCLE_REVERSE = 110, // 2 byte unsigned value (0-600)
        // WARNING: The EEPROM initialization assumes the 5 parameters above are consecutive!

        // WARNING: The EEPROM initialization assumes the 2 parameters below are consecutive!
        PARAMETER_MOTOR_MAX_CURRENT_FORWARD = 112, // 1 byte unsigned value (units of current_calibration_forward)
        PARAMETER_MOTOR_MAX_CURRENT_REVERSE = 113, // 1 byte unsigned value (units of current_calibration_reverse)
        // WARNING: The EEPROM initialization assumes the 2 parameters above are consecutive!

        // WARNING: The EEPROM initialization assumes the 2 parameters below are consecutive!
        PARAMETER_MOTOR_CURRENT_CALIBRATION_FORWARD = 114, // 1 byte unsigned value (units of mA)
        PARAMETER_MOTOR_CURRENT_CALIBRATION_REVERSE = 115, // 1 byte unsigned value (units of mA)
        // WARNING: The EEPROM initialization assumes the 2 parameters above are consecutive!

        PARAMETER_MOTOR_BRAKE_DURATION_FORWARD = 116, // 1 byte unsigned value (units of 5 ms)
        PARAMETER_MOTOR_BRAKE_DURATION_REVERSE = 117, // 1 byte unsigned value (units of 5 ms)
        PARAMETER_MOTOR_COAST_WHEN_OFF = 118, // 1 bit boolean value (coast=1, brake=0)

        PARAMETER_ERROR_ENABLE = 130, // 2 byte unsigned value.  See below for the meanings of the bits.
        PARAMETER_ERROR_LATCH = 132, // 2 byte unsigned value.  See below for the meanings of the bits.
    };

    public enum jrkInputMode : byte
    {
        // Motor is controlled by USB commands,
        // USB serial commands, and/or TTL serial commands.
        INPUT_MODE_SERIAL = 0,

        // Motor is controlled by an analog input on the RX pin
        INPUT_MODE_ANALOG = 1,

        // Motor is controlled by a varying pulse length on the RX(?) pin
        INPUT_MODE_PULSE_WIDTH = 2,
    };

    public enum jrkSerialMode : byte
    {
        // On the Command Port, user can send commands and receive responses.
        // TTL port/UART are connected to make a USB-to-serial adapter.
        SERIAL_MODE_USB_DUAL_PORT = 0,

        // On the Command Port, user can send commands to UMC01 and
        // simultaneously transmit bytes on the UART TX line, and user
        // can receive bytes from the UMC01 and the UART RX line.
        // COM2 does not do anything.
        SERIAL_MODE_USB_CHAINED = 1,

        // On the UART, user can send commands and receive reponses.
        // Command Port and TTL Port don't do anything.
        SERIAL_MODE_UART_DETECT_BAUD_RATE = 2,
        SERIAL_MODE_UART_FIXED_BAUD_RATE = 3,
    };

    public enum jrkFeedbackMode : byte
    {
        // There is no feedback, so all that can be controlled
        // is motor direction and speed.
        FEEDBACK_MODE_NONE = 0,

        // Feedback comes from an analog input on FB
        FEEDBACK_MODE_ANALOG = 1,

        // Feedback comes from a tachometer connected to FB
        FEEDBACK_MODE_TACHOMETER = 2,
    };

    public enum jrkMotorPWMFrequency : byte
    {
        MOTOR_PWM_FREQUENCY_20 = 0,
        MOTOR_PWM_FREQUENCY_5 = 1,
    };

    public enum jrkError : byte
    {
        ERROR_AWAITING_COMMAND = 0, // Always enabled.  Never latched.
        ERROR_NO_POWER = 1, // Always enabled.
        ERROR_MOTOR_DRIVER = 2,  // Always enabled.
        ERROR_INPUT_INVALID = 3, // Always enabled.
        ERROR_INPUT_DISCONNECT = 4, 
        ERROR_FEEDBACK_DISCONNECT = 5,
        ERROR_MAXIMUM_CURRENT_EXCEEDED = 6,
        ERROR_SERIAL_SIGNAL = 7, // Always latched.
        ERROR_SERIAL_OVERRUN = 8, // Always latched.
        ERROR_SERIAL_BUFFER_FULL = 9, // Always latched.
        ERROR_SERIAL_CRC = 10, // Always latched.
        ERROR_SERIAL_PROTOCOL = 11, // Always latched.
        ERROR_SERIAL_TIMEOUT = 12, // Always latched.
    };

    /* To enter bootloading mode, send this request:
     *   bRequestType = 0b01000000  Direction=Host-to-device, Type=Vendor, Recipient=Device
     *   bRequest = REQUEST_START_BOOTLOADER
     *   wValue = 0
     *   wIndex = 0
     *   wLength = 0
     */


    // Local Variables: **
    // mode: C **
    // c-basic-offset: 4 **
    // tab-width: 4 **
    // indent-tabs-mode: t **
    // end: **


}