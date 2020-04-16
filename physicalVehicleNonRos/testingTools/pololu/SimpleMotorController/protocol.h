#ifndef _PROTOCOL_H
#define _PROTOCOL_H

// Native USB commands.  Note: these are NOT serial command bytes.
enum HpmcRequest
{
    REQUEST_GET_SETTINGS = 0x81,
    REQUEST_SET_SETTINGS = 0x82,
    REQUEST_GET_VARIABLES = 0x83,
    REQUEST_RESET_SETTINGS = 0x84,
    REQUEST_GET_RESET_FLAGS = 0x85,
    REQUEST_SET_SPEED = 0x90,
    REQUEST_EXIT_SAFE_START = 0x91,
    REQUEST_SET_MOTOR_LIMIT = 0x92,
    REQUEST_SET_USB_KILL = 0x93,
    REQUEST_GET_STALL_ERROR = 0xB0,
    REQUEST_START_BOOTLOADER = 0xFF,
};

// NOTE: the order and types of these struct members must not change!
struct __attribute__((packed, aligned(4))) HpmcMotorLimits
{
	unsigned short maxSpeed;        // Absolute maximum speed; maps to maximum/minimum in RC/analog mode (0-3200).
	unsigned short maxAcceleration; // Maximum amount that speed magnitude can increase each update period (0-3200).
	unsigned short maxDeceleration; // Maximum amount that speed magnitude can decrease each update period (0-3200).
	unsigned short brakeDuration;   // Brake time required before switching to driving direction.  Units: 1 ms.
	unsigned short startingSpeed;	// Minimum non-zero speed (RAM value cannot be changed by serial/USB); maps to scaledVal==1 in RC/analog mode (0-3200).  0 means no effect.
	unsigned short RESERVED0;
};

struct __attribute__((packed, aligned(4))) HpmcChannelSettings
{
	unsigned char invert;			// 0 or 1.  Used to invert the scaling (higher rawValue => lower scaledValue).
	unsigned char scalingDegree; 	// 0 = linear, 1 = quadratic, 2 = cubic, etc.
	unsigned char alternateUse;		// determines if this is channel acts as a limit/kill switch
	unsigned char pinMode;			// determines if analog input is floating, pulled-up, or pulled-down (doesn't affect RC inputs)

	unsigned short errorMin;		// Raw values greater than this generate an error
	unsigned short errorMax;		// Raw values greater than this generate an error

	unsigned short inputMin;		// The rawValue that gets mapped to a speed of -reverseLimits.maxSpeed (or forwardLimits.maxSpeed if invert==1)
	unsigned short inputMax;		// The rawValue that gets mapped to a speed of forwardLimits.maxSpeed (or -reverseLimits.maxSpeed if invert==1)

	unsigned short inputNeutralMin;	// rawValues from inputNeutralMin to inputNeutralMax map to speed 0.
	unsigned short inputNeutralMax;
};

struct __attribute__((packed, aligned(4))) HpmcSettings
{
	unsigned char neverSuspend;			// Boolean (0 or 1)
	unsigned char uartResponseDelay;	// Boolean
	unsigned char useFixedBaudRate;     // Boolean
	unsigned char disableSafeStart;     // Boolean

	unsigned short fixedBaudRateRegister; // Value to put in USART->BRR (only used if useFixedBaudRate is non-zero)
	unsigned short speedUpdatePeriod;	// This is the time between application of accel/decel updates to speed (units of 1 ms).  should never be 0!

	unsigned short commandTimeout;		// 0 means disabled.  Units: 10 ms
	unsigned char serialDeviceNumber;
	unsigned char crcMode;              // See CRC_MODE_* macros.

	unsigned short overTempMin;         // Units: tenths of a degree Celsius.  See limitTempWithHysteresis.
	unsigned short overTempMax;         // Units: tenths of a degree Celsius.  Temperature where speed is limited to 0.

	unsigned char inputMode;            // See INPUT_MODE_* macros.
	unsigned char pwmMode;              // See PWM_MODE_DRIVE_* macros.
	unsigned char pwmPeriodFactor;      // Determines the PWM frequency (0-19, 0=highest freq).
	unsigned char mixingMode;           // See MIXING_MODE_* macros.

	unsigned short minPulsePeriod;      // Minimum allowed time between consecutive RC pulse rising edges (units of 1 ms).
	unsigned short maxPulsePeriod;      // Maximum allowed time between consecutive RC pulse rising edges (units of 1 ms).

	unsigned short rcTimeout;			// Generates error and shuts down motor if we go this long without heeding a pulse (units of 1 ms)
	unsigned char ignorePotDisconnect;	// Check for pot disconnect toggling POTPWR pin.  Boolean. False: check.  True: don't check (POTPWR always high)
	unsigned char tempLimitGradual;		// Boolean: true: gradual speed limit starting at overTempMin.  false: abrupt limit past overTempMax asserted until temp falls below overTempMin

	unsigned char consecGoodPulses;	    // Number of consecutive good pulses needed before we heed them (update channel's rawValue).  0 is same as 1.
	unsigned char motorInvert;          // Invert Motor Direction.  Boolean.  False: 3200=OUTA>OUTB.  True:  3200=OUTA<OUTB.
	unsigned char speedZeroBrakeAmount; // Brake amount while input is in deadband (0-32), or there is an error, or motor is driving at speed zero.
	unsigned char ignoreErrLineHigh;    // When set, ignore inputs state of ERR line (when cleared, allows you to connect the error lines of two devices and have them both stop when one has an error).

	signed short vinMultiplierOffset;	// value that gets added to vinConversionMultiplier (e.g. to compensate for variations in VIN voltage divider)
	unsigned short lowVinShutoffTimeout;// Vin must stay below lowVinShutoffMv for this duration before a low-VIN error occurs (units of 1 ms)

	unsigned short lowVinShutoffMv;		// Dropping below this voltage threshold for a duration of lowVinShutoffTimeout triggers a low-voltage error (units of mV)
	unsigned short lowVinStartupMv;		// Once asserting a low-voltage error, the voltage required to stop asserting this error (units of mV)

	unsigned short highVinShutoffMv;	// Rising above this voltage threshold triggers a high-voltage error and causes the motor to immediately brake at 100% (units of mV)
	unsigned char serialMode;			// see SERIAL_MODE+* macros.
	unsigned char RESERVED0;

	struct HpmcChannelSettings rc1;
	struct HpmcChannelSettings rc2;
	struct HpmcChannelSettings analog1;
	struct HpmcChannelSettings analog2;

	struct HpmcMotorLimits forwardLimits;
	struct HpmcMotorLimits reverseLimits;
};

struct __attribute__((packed, aligned(4))) HpmcChannelVariables
{
	unsigned short unlimitedRawValue;	// 0xFFFF if disconnected but not affected by absolute max/min limits. Units of quarter-microseconds
	unsigned short rawValue;  			// 0xFFFF if disconnected or outside of absolute maximum/minimum limits.
										//  Units of quarter-microseconds if an RC channel.  12-bit ADC reading if analog.
	signed short scaledValue;
	unsigned short RESERVED0;
};

// NOTE: the order and types of these struct members must not change because some of these
//  variables (errorStatus, errorOccurred, serialErrorOccurred, and baudRateRegister)
//  are treated as special cases by the get-variables command.  These four variables must
//  be loaded from shadow registers whenever they are requested.
//  Though not stored in this struct, the resetSource variable can be requested using the
//  get-variables command with variable ID 127.
struct __attribute__((packed, aligned(4))) HpmcVariables
{
	unsigned short errorStatus;         // varId 0: The errors that are currently happening.  See ERROR_* macros.
	unsigned short errorOccurred;       // varId 1: The errors that occurred since the last cleared.  See ERROR_* macros.

	unsigned short serialErrorOccurred;	// varId 2: The serial errors that occurred since last cleared.  See SERIAL_ERROR_* macros.
	unsigned short limitStatus;		    // varId 3: indicates things that are limiting operation but aren't errors.

	struct HpmcChannelVariables rc1;	// varId  4 (unlim),  5 (raw),  6 (scaled),  7 (n/a)
	struct HpmcChannelVariables rc2;	// varId  8 (unlim),  9 (raw), 10 (scaled), 11 (n/a)
	struct HpmcChannelVariables analog1;// varId 12 (unlim), 13 (raw), 14 (scaled), 15 (n/a)
	struct HpmcChannelVariables analog2;// varId 16 (unlim), 17 (raw), 18 (scaled), 19 (n/a)

	signed short targetSpeed;			// varId 20: Target speed of motor from -3200 to 3200.
	signed short speed;					// varId 21: Current speed of motor from -3200 to 3200.
	unsigned short brakeAmount;			// varId 22: Current braking amount, set to 0xFF when speed!=0 because it is irrelevant (0-32).
	unsigned short vinMv;				// varId 23: Units: millivolts

	unsigned short temperature;			// varId 24: Units: tenths of a degree Celsius
	unsigned short RESERVED0;			// varId 25: reserved

	unsigned short rcPeriod;			// varId 26: Period of rc signal (0 means no good signal).  Units: 0.1 ms
	unsigned short baudRateRegister;	// varId 27: Value from USART1->BRR (used to debug auto-baud detect)

	unsigned long timeMs;				// varId 28: system timer low half-word
										// varId 29: system timer high half-word

	struct HpmcMotorLimits forwardLimits;	// varId 30 - 34 (35 reserved)
	struct HpmcMotorLimits reverseLimits;	// varId 36 - 40 (45 reserved)
};

// Valid values for the inputMode setting:
#define INPUT_MODE_SERIAL_USB 0
#define INPUT_MODE_ANALOG     1
#define INPUT_MODE_RC         2

// Valid values for mixingMode setting:
#define MIXING_MODE_NONE  0
#define MIXING_MODE_LEFT  1
#define MIXING_MODE_RIGHT 2

// Valid values for serialMode setting:
#define SERIAL_MODE_BINARY				0	// compact or Pololu protocols
#define SERIAL_MODE_ASCII				1	// ASCII protocol with prompts and echoing of received bytes (for use with terminal program)

// Valid values for crcMode setting:
#define CRC_MODE_DISABLED               0
#define CRC_MODE_COMMANDS               1
#define CRC_MODE_COMMANDS_AND_RESPONSES 3  // Note: this is not 2

// Valid values for wIndex in the USB Set Duty Cycle request:
#define DIRECTION_FORWARD 0
#define DIRECTION_REVERSE 1
#define DIRECTION_BRAKE   2

// Valid values for channel setting alternateUse
#define ALTERNATE_USE_DISABLED			0
#define ALTERNATE_USE_LIMIT_FORWARD		1
#define ALTERNATE_USE_LIMIT_REVERSE		2
#define ALTERNATE_USE_KILL_SWITCH		3

// Valid values for channel setting pinMode
#define PIN_MODE_FLOATING				0	// recommended mode when not using a limit switch
#define PIN_MODE_PULL_UP				1	// internal pull-up enabled on analog input
#define PIN_MODE_PULL_DOWN				2	// internal pull-down enabled on analog input

// limitStatus variable bits
#define LIMITED_BY_STARTED_STATE	(1<<0)	// 1 => motors are not allowed to start running
#define LIMITED_BY_TEMPERATURE		(1<<1)	// 1 => temperature is actively reducing our target speed
#define LIMITED_BY_MAX_SPEED		(1<<2)	// 1 => max speed setting is actively reducing our target speed
#define LIMITED_BY_STARTING_SPEED	(1<<3)	// 1 => starting speed setting is actively reducing our target speed
#define LIMITED_BY_ACCELERATION		(1<<4)	// 1 => current speed != target speed because of accel/decel/brake-duration limits
#define	LIMITED_BY_RC1				(1<<5)	// 1 => RC1 limit switch triggered
#define	LIMITED_BY_RC2				(1<<6)	// 1 => RC2 limit switch triggered
#define	LIMITED_BY_ANALOG1			(1<<7)	// 1 => analog1 limit switch triggered
#define	LIMITED_BY_ANALOG2			(1<<8)	// 1 => analog2 limit switch triggered
#define LIMITED_BY_USB_KILL			(1<<9)	// 1 => native USB kill switch active

#define LIMIT_ALL					0xFFFF

// Valid values for pwmMode setting:
#define PWM_MODE_DRIVE_BRAKE		0	// default (intentionally making this zero)
#define PWM_MODE_DRIVE_COAST		1	// config utility will not give user this option (it doesn't work well)

// errorStatus (and therefore errorOccurred) bits:
#define ERROR_SAFE_START	  	(1<<0)	// In RC/Analog mode: target speed > 0.0625*maxSpeed && started==0 && safe start enabled
										// In serial/USB mode: set when started==0, cleared by special command; cannot set started=1 until cleared
#define ERROR_CHANNEL_INVALID 	(1<<1)
#define ERROR_SERIAL          	(1<<2)	// errorStatus bit set on serial error when in serial/USB mode; cleared on successful reception of serial command packet
#define ERROR_COMMAND_TIMEOUT	(1<<3)	// too much time has passed since we received the last valid command packet from USBCOM or UART or motor command over native USB
#define ERROR_LIMIT_SWITCH	  	(1<<4)
#define ERROR_VIN_LOW         	(1<<5)
#define ERROR_VIN_HIGH			(1<<6)
#define ERROR_TEMPERATURE_HIGH  (1<<7)
#define ERROR_MOTOR_DRIVER    	(1<<8)
#define ERROR_ERR_LINE_HIGH	  	(1<<9)	// external source is driving ERR line high (this errorStatus bit does NOT turn on red LED
										// and is only set when we are NOT driving our own ERR line high)

#define ERROR_ALL			  	0xFFFF

// serialErrorOccurred bits:
// *** NOTE: the bit values for the first four errors cannot change
//            (they match the locations of flag bits in USART1->SR status register)
#define SERIAL_ERROR_PARITY		(1 << 0)	// hardware parity error PE (not used)
#define SERIAL_ERROR_FRAME		(1 << 1)	// hardware frame error FE
#define SERIAL_ERROR_NOISE		(1 << 2)	// hardware noise error NE
#define SERIAL_ERROR_RX_OVERRUN	(1 << 3)	// rxBuffer or hardware overrun error ORE (rx byte while RXNE set)

#define SERIAL_ERROR_FORMAT		(1 << 5)	// command packet format error
#define SERIAL_ERROR_CRC		(1 << 6)	// received incorrect CRC byte


// Valid values for the reset source:
#define RESET_NRST_PIN		0x04	// NRST pin was pulled low by an external source
#define RESET_POWER			0x0C	// the device stopped running because power got too low
#define RESET_SOFTWARE		0x14   	// caused when entering application from the Bootloader
#define RESET_IWDG			0x24	// independent watchdog reset caused by firmware crash (could indicate a firmware bug)


// Value of RC or analog channel that is dead/disconnected or out of range
#define DISCONNECTED_INPUT	0xFFFF


// Serial command bytes (with MSB set)
// Note: every command differs from every other command by at least two bits, so if noise changes
// a single bit of a valid command, the result will be an invalid command (similar to parity checking
// but without the extra parity bit).
#define COMMAND_EXIT_SAFE_START			0x83	// 0 data bytes
#define COMMAND_MOTOR_FORWARD			0x85	// 2 data bytes	(Note: this command must be odd)
#define COMMAND_MOTOR_REVERSE			0x86	// 2 data bytes	(Note: this command must be even)
#define COMMAND_MOTOR_FORWARD_7BIT		0x89	// 1 data byte (Note: this command must be odd)
#define COMMAND_MOTOR_REVERSE_7BIT		0x8A	// 1 data byte (Note: this command must be even)
#define COMMAND_VARIABLE_BRAKE			0x92	// 1 data byte (0 = full coast, 32 = full brake)
#define COMMAND_GET_VARIABLE			0xA1	// 1 data byte
#define COMMAND_SET_MOTOR_LIMIT			0xA2	// 3 data bytes
#define COMMAND_GET_FIRMWARE_VERSION	0xC2	// 0 data bytes
#define COMMAND_STOP_MOTOR				0xE0	// 0 data bytes
#define COMMAND_MINI_SSC				0xFF	// 2 data bytes (2nd byte: 0 = full rev, 127 = spd 0, 254 = full forward)

#endif
