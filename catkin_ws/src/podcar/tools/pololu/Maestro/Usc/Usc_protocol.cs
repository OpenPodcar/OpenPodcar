using System;
using System.Runtime.InteropServices;

namespace Pololu.Usc
{
    /* protocol.h:  This file defines constants needed to communicate with the
     * Maestro via USB, USB serial, or TTL serial. */

    ///<summary>
    /// These are the values to put in to bRequest when making a setup packet
    /// for a control transfer to the Maestro.  See the comments and code in Usc.cs
    /// for more information about what these requests do and the format of the
    /// setup packet.
    ///</summary>
    public enum uscRequest
    {
        REQUEST_GET_PARAMETER = 0x81,
        REQUEST_SET_PARAMETER = 0x82,
        REQUEST_GET_VARIABLES = 0x83,
        REQUEST_SET_SERVO_VARIABLE = 0x84,
        REQUEST_SET_TARGET = 0x85,
        REQUEST_CLEAR_ERRORS = 0x86,

        // These four requests are only valid on *Mini* Maestros.
        REQUEST_GET_SERVO_SETTINGS = 0x87,
		REQUEST_GET_STACK = 0x88,
        REQUEST_GET_CALL_STACK = 0x89,
        REQUEST_SET_PWM = 0x8A,

        REQUEST_REINITIALIZE = 0x90,
        REQUEST_ERASE_SCRIPT = 0xA0,
        REQUEST_WRITE_SCRIPT = 0xA1,
        REQUEST_SET_SCRIPT_DONE = 0xA2, // wValue is 0 for go, 1 for stop, 2 for single-step
        REQUEST_RESTART_SCRIPT_AT_SUBROUTINE = 0xA3,
        REQUEST_RESTART_SCRIPT_AT_SUBROUTINE_WITH_PARAMETER = 0xA4,
        REQUEST_RESTART_SCRIPT = 0xA5,
        REQUEST_START_BOOTLOADER = 0xFF
    }

    /// <summary>
    /// Represents the current status of a channel.
    /// </summary>
    [StructLayout(LayoutKind.Sequential, Pack = 1)]
    public struct ServoStatus
    {
        /// <summary>The position in units of quarter-microseconds.</summary>
        public UInt16 position;

        /// <summary>The target position in units of quarter-microseconds.</summary>
        public UInt16 target;

        /// <summary>The speed limit.  Units depends on your settings.</summary>
        public UInt16 speed;

        /// <summary>The acceleration limit.  Units depend on your settings.</summary>
        public Byte acceleration;
    };

    /// <summary>
    /// Represents the variables that can be read from
    /// a Micro Maestro or Mini Maestro using REQUEST_GET_VARIABLES,
    /// excluding channel information, the stack, and the call stack.
    /// </summary>
	public struct MaestroVariables
    {
        /// <summary>
        /// The number of values on the data stack (0-32).  A value of 0 means the stack is empty.
        /// </summary>
        public Byte stackPointer;

        /// <summary>
        /// The number of return locations on the call stack (0-10).  A value of 0 means the stack is empty.
        /// </summary>
        public Byte callStackPointer;

        /// <summary>
        /// The error register.  Each bit stands for a different error (see uscError).
        /// If the bit is one, then it means that error occurred some time since the last
        /// GET_ERRORS serial command or CLEAR_ERRORS USB command.
        /// </summary>
        public UInt16 errors;

        /// <summary>
        /// The address (in bytes) of the next bytecode instruction that will be executed.
        /// </summary>
        public UInt16 programCounter;

        /// <summary>
        /// 0 = script is running.
        /// 1 = script is done.
        /// 2 = script will be done as soon as it executes one more instruction
        ///     (used to implement step-through debugging features)
        /// </summary>
        public Byte scriptDone;

        /// <summary>
        /// The performance flag register.  Each bit represents a different flag.
        /// If it is 1, then it means that the flag occurred some time since the last
        /// getVariables request.  This register is always 0 for the Micro Maestro
        /// because performance flags only apply to the Mini Maestros.
        /// </summary>
        public Byte performanceFlags;
    }

    /// <summary>
    /// Represents the non-channel-specific variables that can be read from
    /// a Micro Maestro using REQUEST_GET_VARIABLES.
    /// </summary>
    [StructLayout(LayoutKind.Sequential, Pack = 1)]
    unsafe public struct MicroMaestroVariables
    {
        /// <summary>
        /// The number of values on the data stack (0-32).  A value of 0 means the stack is empty.
        /// </summary>
        public Byte stackPointer;

        /// <summary>
        /// The number of return locations on the call stack (0-10).  A value of 0 means the stack is empty.
        /// </summary>
        public Byte callStackPointer;

        /// <summary>
        /// The error register.  Each bit stands for a different error (see uscError).
        /// If the bit is one, then it means that error occurred some time since the last
        /// GET_ERRORS serial command or CLEAR_ERRORS USB command.
        /// </summary>
        public UInt16 errors;

        /// <summary>
        /// The address (in bytes) of the next bytecode instruction that will be executed.
        /// </summary>
        public UInt16 programCounter;

        /// <summary>Meaningless bytes to protect the program from stack underflows.</summary>
        /// <remarks>This is public to avoid mono warning CS0169.</remarks>
        public fixed Int16 buffer[3];

        /// <summary>
        /// The data stack used by the script.  The values in locations 0 through stackPointer-1
        /// are on the stack.
        /// </summary>
        public fixed Int16 stack[32];

        /// <summary>
        /// The call stack used by the script.  The addresses in locations 0 through
        /// callStackPointer-1 are on the call stack.  The next return will make the
        /// program counter go to callStack[callStackPointer-1].
        /// </summary>
        public fixed UInt16 callStack[10];

        /// <summary>
        /// 0 = script is running.
        /// 1 = script is done.
        /// 2 = script will be done as soon as it executes one more instruction
        ///     (used to implement step-through debugging features)
        /// </summary>
        public Byte scriptDone;

        /// <summary>Meaningless byte to protect the program from call stack overflows.</summary>
        /// <remarks>This is public to avoid mono warning CS0169.</remarks>
        public Byte buffer2;

        // NOTE: C# does not allow fixed arrays of structs; after these variables,
        // 6 copies of servoSetting follow on the Micro Maestro.
    }

    /// <summary>
    /// Represents the variables that can be read from a Mini Maestro using REQUEST_GET_VARIABLES.
    /// </summary>
    [StructLayout(LayoutKind.Sequential, Pack = 1)]
    unsafe public struct MiniMaestroVariables
    {
        /// <summary>
        /// The number of values on the data stack (0-32).  A value of 0 means the stack is empty.
        /// </summary>
        public Byte stackPointer;

        /// <summary>
        /// The number of return locations on the call stack (0-10).  A value of 0 means the stack is empty.
        /// </summary>
        public Byte callStackPointer;

        /// <summary>
        /// The error register.  Each bit stands for a different error (see uscError).
        /// If the bit is one, then it means that error occurred some time since the last
        /// GET_ERRORS serial command or CLEAR_ERRORS USB command.
        /// </summary>
        public UInt16 errors;

        /// <summary>
        /// The address (in bytes) of the next bytecode instruction that will be executed.
        /// </summary>
        public UInt16 programCounter;

        /// <summary>
        /// 0 = script is running.
        /// 1 = script is done.
        /// 2 = script will be done as soon as it executes one more instruction
        ///     (used to implement step-through debugging features)
        /// </summary>
        public Byte scriptDone;

        /// <summary>
        /// The performance flag register.  Each bit represents a different error.
        /// If it is 1, then it means that the error occurred some time since the last
        /// getVariables request.
        /// </summary>
        public Byte performanceFlags; 
    }

    /// <summary>
    /// The different parameters that can be read or written with REQUEST_GET_PARAMETER
    /// and REQUEST_SET_PARAMETER.  These values should only be used by the Usc class.
    /// </summary>
    public enum uscParameter : byte
    {
        PARAMETER_INITIALIZED = 0, // 1 byte - 0 or 0xFF
        PARAMETER_SERVOS_AVAILABLE = 1, // 1 byte - 0-5
        PARAMETER_SERVO_PERIOD = 2, // 1 byte - ticks allocated to each servo/256
        PARAMETER_SERIAL_MODE = 3, // 1 byte unsigned value.  Valid values are SERIAL_MODE_*.  Init variable.
        PARAMETER_SERIAL_FIXED_BAUD_RATE = 4, // 2-byte unsigned value; 0 means autodetect.  Init parameter.
        PARAMETER_SERIAL_TIMEOUT = 6, // 2-byte unsigned value
        PARAMETER_SERIAL_ENABLE_CRC = 8, // 1 byte boolean value
        PARAMETER_SERIAL_NEVER_SUSPEND = 9, // 1 byte boolean value
        PARAMETER_SERIAL_DEVICE_NUMBER = 10, // 1 byte unsigned value, 0-127
        PARAMETER_SERIAL_BAUD_DETECT_TYPE = 11, // 1 byte value

        PARAMETER_IO_MASK_C = 16, // 1 byte - pins used for I/O instead of servo
        PARAMETER_OUTPUT_MASK_C = 17, // 1 byte - outputs that are enabled

        PARAMETER_CHANNEL_MODES_0_3                 = 12, // 1 byte - channel modes 0-3
        PARAMETER_CHANNEL_MODES_4_7                 = 13, // 1 byte - channel modes 4-7
        PARAMETER_CHANNEL_MODES_8_11                = 14, // 1 byte - channel modes 8-11
        PARAMETER_CHANNEL_MODES_12_15               = 15, // 1 byte - channel modes 12-15
        PARAMETER_CHANNEL_MODES_16_19               = 16, // 1 byte - channel modes 16-19
        PARAMETER_CHANNEL_MODES_20_23               = 17, // 1 byte - channel modes 20-23
        PARAMETER_MINI_MAESTRO_SERVO_PERIOD_L = 18, // servo period: 3-byte unsigned values, units of quarter microseconds
        PARAMETER_MINI_MAESTRO_SERVO_PERIOD_HU = 19,
        PARAMETER_ENABLE_PULLUPS = 21,  // 1 byte: 0 or 1
        PARAMETER_SCRIPT_CRC = 22, // 2 bytes - stores a checksum of the bytecode program, for comparison
        PARAMETER_SCRIPT_DONE = 24, // 1 byte - copied to scriptDone on startup
        PARAMETER_SERIAL_MINI_SSC_OFFSET = 25, // 1 byte (0-254)
        PARAMETER_SERVO_MULTIPLIER = 26, // 1 byte (0-255)

        // 9 * 24 = 216, so we can safely start at 30
        PARAMETER_SERVO0_HOME = 30, // 2 byte home position (0=off; 1=ignore)
        PARAMETER_SERVO0_MIN = 32, // 1 byte min allowed value (x2^6)
        PARAMETER_SERVO0_MAX = 33, // 1 byte max allowed value (x2^6)
        PARAMETER_SERVO0_NEUTRAL = 34, // 2 byte neutral position
        PARAMETER_SERVO0_RANGE = 36, // 1 byte range
        PARAMETER_SERVO0_SPEED = 37, // 1 byte (5 mantissa,3 exponent) us per 10ms
        PARAMETER_SERVO0_ACCELERATION = 38, // 1 byte (speed changes that much every 10ms)
        PARAMETER_SERVO1_HOME = 39, // 2 byte home position (0=off; 1=ignore)
        PARAMETER_SERVO1_MIN = 41, // 1 byte min allowed value (x2^6)
        PARAMETER_SERVO1_MAX = 42, // 1 byte max allowed value (x2^6)
        PARAMETER_SERVO1_NEUTRAL = 43, // 2 byte neutral position
        PARAMETER_SERVO1_RANGE = 45, // 1 byte range
        PARAMETER_SERVO1_SPEED = 46, // 1 byte (5 mantissa,3 exponent) us per 10ms
        PARAMETER_SERVO1_ACCELERATION = 47, // 1 byte (speed changes that much every 10ms)
        PARAMETER_SERVO2_HOME = 48, // 2 byte home position (0=off; 1=ignore)
        PARAMETER_SERVO2_MIN = 50, // 1 byte min allowed value (x2^6)
        PARAMETER_SERVO2_MAX = 51, // 1 byte max allowed value (x2^6)
        PARAMETER_SERVO2_NEUTRAL = 52, // 2 byte neutral position
        PARAMETER_SERVO2_RANGE = 54, // 1 byte range
        PARAMETER_SERVO2_SPEED = 55, // 1 byte (5 mantissa,3 exponent) us per 10ms
        PARAMETER_SERVO2_ACCELERATION = 56, // 1 byte (speed changes that much every 10ms)
        PARAMETER_SERVO3_HOME = 57, // 2 byte home position (0=off; 1=ignore)
        PARAMETER_SERVO3_MIN = 59, // 1 byte min allowed value (x2^6)
        PARAMETER_SERVO3_MAX = 60, // 1 byte max allowed value (x2^6)
        PARAMETER_SERVO3_NEUTRAL = 61, // 2 byte neutral position
        PARAMETER_SERVO3_RANGE = 63, // 1 byte range
        PARAMETER_SERVO3_SPEED = 64, // 1 byte (5 mantissa,3 exponent) us per 10ms
        PARAMETER_SERVO3_ACCELERATION = 65, // 1 byte (speed changes that much every 10ms)
        PARAMETER_SERVO4_HOME = 66, // 2 byte home position (0=off; 1=ignore)
        PARAMETER_SERVO4_MIN = 68, // 1 byte min allowed value (x2^6)
        PARAMETER_SERVO4_MAX = 69, // 1 byte max allowed value (x2^6)
        PARAMETER_SERVO4_NEUTRAL = 70, // 2 byte neutral position
        PARAMETER_SERVO4_RANGE = 72, // 1 byte range
        PARAMETER_SERVO4_SPEED = 73, // 1 byte (5 mantissa,3 exponent) us per 10ms
        PARAMETER_SERVO4_ACCELERATION = 74, // 1 byte (speed changes that much every 10ms)
        PARAMETER_SERVO5_HOME = 75, // 2 byte home position (0=off; 1=ignore)
        PARAMETER_SERVO5_MIN = 77, // 1 byte min allowed value (x2^6)
        PARAMETER_SERVO5_MAX = 78, // 1 byte max allowed value (x2^6)
        PARAMETER_SERVO5_NEUTRAL = 79, // 2 byte neutral position
        PARAMETER_SERVO5_RANGE = 81, // 1 byte range
        PARAMETER_SERVO5_SPEED = 82, // 1 byte (5 mantissa,3 exponent) us per 10ms
        PARAMETER_SERVO5_ACCELERATION = 83, // 1 byte (speed changes that much every 10ms)
    };

    /// <summary>
    /// The different serial modes the Maestro can be in.  The serial mode
    /// determines how the Command Port, TTL Port, the TTL-level UART, and the
    /// command processor are connected.
    /// </summary>
    public enum uscSerialMode : byte
    {
        ///<summary>On the Command Port, user can send commands and receive responses.
        ///TTL port/UART are connected to make a USB-to-serial adapter.</summary> 
        SERIAL_MODE_USB_DUAL_PORT = 0,

        ///<summary>On the Command Port, user can send commands to Maestro and
        /// simultaneously transmit bytes on the UART TX line, and user
        /// can receive bytes from the Maestro and the UART RX line.
        /// TTL port does not do anything.</summary>
        SERIAL_MODE_USB_CHAINED = 1,

        /// <summary>
        /// On the UART, user can send commands and receive reponses after
        /// sending a 0xAA byte to indicate the baud rate.
        /// Command Port receives bytes from the RX line.
        /// TTL Port does not do anything.
        /// </summary>
        SERIAL_MODE_UART_DETECT_BAUD_RATE = 2,

        /// <summary>
        /// On the UART, user can send commands and receive reponses
        /// at a predetermined, fixed baud rate.
        /// Command Port receives bytes from the RX line.
        /// TTL Port does not do anything.
        /// </summary>
        SERIAL_MODE_UART_FIXED_BAUD_RATE = 3,
    };

    /// <summary>
    /// The correspondence between errors and bits in the two-byte error register.
    /// For more details about what the errors mean, see the user's guide. 
    /// </summary>
    public enum uscError : byte
    {
        ERROR_SERIAL_SIGNAL = 0,
        ERROR_SERIAL_OVERRUN = 1,
        ERROR_SERIAL_BUFFER_FULL = 2,
        ERROR_SERIAL_CRC = 3,
        ERROR_SERIAL_PROTOCOL = 4,
        ERROR_SERIAL_TIMEOUT = 5,
        ERROR_SCRIPT_STACK = 6,
        ERROR_SCRIPT_CALL_STACK = 7,
        ERROR_SCRIPT_PROGRAM_COUNTER = 8,
    };

    public enum performanceFlag : byte
    {
        PERROR_ADVANCED_UPDATE = 0,
        PERROR_BASIC_UPDATE = 1,
        PERROR_PERIOD = 2
    };
}

// Local Variables: **
// mode: java **
// c-basic-offset: 4 **
// tab-width: 4 **
// indent-tabs-mode: t **
// end: **
