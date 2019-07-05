/* This C header file that defines the constants needed to communicate with the
   Maestro via USB, USB serial, or TTL serial.

   Most of the information here is also in Usc_protocol.cs.

   This file can be included directly in C/C++ programs or used
   as a reference when writing programs in other languages.
 */

// Serial commands, sent on the virtual serial port or over TTL Serial.
// See the user's guide at http://www.pololu.com/docs/0J40 for more info.
enum uscCommand
{
    COMMAND_SET_TARGET = 0x84, // 3 data bytes
    COMMAND_SET_SPEED = 0x87, // 3 data bytes
    COMMAND_SET_ACCELERATION = 0x89, // 3 data bytes
    COMMAND_GET_POSITION = 0x90, // 0 data
    COMMAND_GET_MOVING_STATE = 0x93, // 0 data
    COMMAND_GET_ERRORS = 0xA1, // 0 data
    COMMAND_GO_HOME = 0xA2, // 0 data
    COMMAND_STOP_SCRIPT = 0xA4, // 0 data
    COMMAND_RESTART_SCRIPT_AT_SUBROUTINE                = 0xA7, // 1 data bytes
    COMMAND_RESTART_SCRIPT_AT_SUBROUTINE_WITH_PARAMETER = 0xA8, // 3 data bytes
    COMMAND_GET_SCRIPT_STATUS = 0xAE, // 0 data
    COMMAND_MINI_SSC = 0xFF, // (2 data bytes)
};

// These are the values to put in to bRequest when making a setup packet
// for a control transfer to the Maestro.  See the comments and code in Usc.cs
// for more information about what these requests do and the format of the
// setup packet.
enum uscRequest
{
    REQUEST_GET_PARAMETER = 0x81,
    REQUEST_SET_PARAMETER = 0x82,
    REQUEST_GET_VARIABLES = 0x83,
    REQUEST_SET_SERVO_VARIABLE = 0x84, // (also clears the serial timeout timer)
    REQUEST_SET_TARGET = 0x85,   // (also clears the serial timeout timer)
    REQUEST_CLEAR_ERRORS = 0x86, // (also clears the serial timeout timer)
    REQUEST_REINITIALIZE = 0x90,
    REQUEST_ERASE_SCRIPT = 0xA0,
    REQUEST_WRITE_SCRIPT = 0xA1,
    REQUEST_SET_SCRIPT_DONE = 0xA2, // value.low.b is 0 for go, 1 for stop, 2 for single-step
    REQUEST_RESTART_SCRIPT_AT_SUBROUTINE = 0xA3,
    REQUEST_RESTART_SCRIPT_AT_SUBROUTINE_WITH_PARAMETER = 0xA4,
    REQUEST_RESTART_SCRIPT = 0xA5,
    REQUEST_START_BOOTLOADER = 0xFF
};

// These are the bytes used to refer to the different parameters
// in REQUEST_GET_PARAMETER and REQUEST_SET_PARAMETER.  After changing
// any parameter marked as an "Init parameter", you must do REQUEST_REINITIALIZE
// before the new value will be used.
enum uscParameter
{
    PARAMETER_SERVOS_AVAILABLE                  = 1, // 1 byte - 0-5.  Init parameter.
    PARAMETER_SERVO_PERIOD                      = 2, // 1 byte - instruction cycles allocated to each servo/256, (units of 21.3333 us).  Init parameter.
    PARAMETER_SERIAL_MODE                       = 3, // 1 byte unsigned value.  Valid values are SERIAL_MODE_*.  Init parameter.
    PARAMETER_SERIAL_FIXED_BAUD_RATE            = 4, // 2-byte unsigned value; 0 means autodetect.  Init parameter.
    PARAMETER_SERIAL_TIMEOUT                    = 6, // 2-byte unsigned value (units of 10ms)
    PARAMETER_SERIAL_ENABLE_CRC                 = 8, // 1 byte boolean value
    PARAMETER_SERIAL_NEVER_SUSPEND              = 9, // 1 byte boolean value
    PARAMETER_SERIAL_DEVICE_NUMBER              = 10, // 1 byte unsigned value, 0-127
    PARAMETER_SERIAL_BAUD_DETECT_TYPE           = 11, // 1 byte - reserved

    PARAMETER_IO_MASK_A                         = 12, // 1 byte - reserved, init parameter
    PARAMETER_OUTPUT_MASK_A                     = 13, // 1 byte - reserved, init parameter
    PARAMETER_IO_MASK_B                         = 14, // 1 byte - reserved, init parameter
    PARAMETER_OUTPUT_MASK_B                     = 15, // 1 byte - reserved, init parameter
    PARAMETER_IO_MASK_C                         = 16, // 1 byte - pins used for I/O instead of servo, init parameter
    PARAMETER_OUTPUT_MASK_C                     = 17, // 1 byte - outputs that are enabled, init parameter
    PARAMETER_IO_MASK_D                         = 18, // 1 byte - reserved, init parameter
    PARAMETER_OUTPUT_MASK_D                     = 19, // 1 byte - reserved, init parameter
    PARAMETER_IO_MASK_E                         = 20, // 1 byte - reserved, init parameter
    PARAMETER_OUTPUT_MASK_E                     = 21, // 1 byte - reserved, init parameter

    PARAMETER_SCRIPT_CRC                        = 22, // 2 byte CRC of script
    PARAMETER_SCRIPT_DONE                       = 24, // 1 byte - if 0, run the bytecode on restart, if 1, stop

    PARAMETER_SERIAL_MINI_SSC_OFFSET            = 25, // 1 byte (0-254)

    PARAMETER_SERVO0_HOME                       = 30, // 2 byte home position (0=off; 1=ignore)
    PARAMETER_SERVO0_MIN                        = 32, // 1 byte min allowed value (x2^6)
    PARAMETER_SERVO0_MAX                        = 33, // 1 byte max allowed value (x2^6)
    PARAMETER_SERVO0_NEUTRAL                    = 34, // 2 byte neutral position
    PARAMETER_SERVO0_RANGE                      = 36, // 1 byte range
    PARAMETER_SERVO0_SPEED                      = 37, // 1 byte (5 mantissa,3 exponent) us per 10ms.  Init parameter.
    PARAMETER_SERVO0_ACCELERATION               = 38, // 1 byte (speed changes that much every 10ms). Init parameter.

    PARAMETER_SERVO1_HOME                       = 39,
    // The pattern continues.  Each servo takes 9 bytes of configuration space.
};

struct servoSetting
{
    u16 position;
    u16 target;
    u16 speed;
    u8 acceleration;
};

/* uscVariables: This struct stores all the variables that can be read via
   REQUEST_GET_VARIABLES.

   There are 12 bytes used per servo setting
 */
struct uscVariables
{
    // Fix bytecode_asm.asm if you change the order or size of
    // variables in this struct.

    // offset: 0
    u8 stackPointer;

    // offset: 1
    u8 callStackPointer;

    // offset: 2
    u16 errors;

    // offset: 4
    u16 programCounter;

    // offset: 6
    s16 buffer[3]; // protects other RAM from being corrupted by improper instructions

    // offset: 12
    s16 stack[32];

    // offset: 76
    u16 callStack[10];

    // offset: 96
    u8 scriptDone; // 1 = done; 2 = about to run a single step then be done - placed here to protect against accidental overwriting of servoSetting

    // offset: 97
    u8 buffer2; // protects other RAM from being corrupted by improper instructions

    // offset: 98
    struct servoSetting servoSetting[6];
}; // total length 139 bytes

#define BAUD_DETECT_TYPE_AA 0u
#define BAUD_DETECT_TYPE_FF 1u

// serialMode: Value of PARAMETER_SERIAL_MODE.
enum serialMode
{
    // On the Command Port, user can send commands and receive responses.
    // TTL port/UART are connected to make a USB-to-serial adapter.
    SERIAL_MODE_USB_DUAL_PORT = 0u,

    // On the Command Port, user can send commands to UMC01 and
    // simultaneously transmit bytes on the UART TX line, and user
    // can receive bytes from the UMC01 and the UART RX line.
    // COM2 does not do anything.
    SERIAL_MODE_USB_CHAINED = 1u,

    // On the UART, user can send commands and receive reponses.
    // Command Port and TTL Port don't do anything.
    SERIAL_MODE_UART_DETECT_BAUD_RATE = 2u,
    SERIAL_MODE_UART_FIXED_BAUD_RATE = 3u,
};

// There are several different errors.  Each error is represented by a
// different bit number from 0 to 15.
#define ERROR_SERIAL_SIGNAL            0
#define ERROR_SERIAL_OVERRUN           1
#define ERROR_SERIAL_BUFFER_FULL       2
#define ERROR_SERIAL_CRC               3
#define ERROR_SERIAL_PROTOCOL          4
#define ERROR_SERIAL_TIMEOUT           5
#define ERROR_SCRIPT_STACK             6
#define ERROR_SCRIPT_CALL_STACK        7
#define ERROR_SCRIPT_PROGRAM_COUNTER   8

// Local Variables: **
// mode: C **
// c-basic-offset: 4 **
// tab-width: 4 **
// indent-tabs-mode: t **
// end: **
