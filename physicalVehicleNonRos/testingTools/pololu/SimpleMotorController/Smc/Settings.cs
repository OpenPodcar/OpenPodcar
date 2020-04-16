using System;

namespace Pololu.SimpleMotorController
{
    /// <summary>
    /// Specifies the settings to use for a particular Input Channel.
    /// </summary>
    public class SmcChannelSettings : ICloneable
    {
        /// <summary>
        /// This is used to allow a channel be a limit switch if it is not controlling the motor.
        /// </summary>
        public SmcChannelAlternateUse alternateUse = SmcChannelAlternateUse.None;

        /// <summary>
        /// Used to invert the direction of scaling.
        /// If true, then a higher rawValue corresponds to lower scaledValue.
        /// </summary>
        public Boolean invert = false;

        /// <summary>
        /// 0 = Linear, 1 = Quadratic, 2 = Cubic, etc.
        /// </summary>
        public Byte scalingDegree = 0;

        /// <summary>
        /// Determines if the analog input is floating, pulled up, or pulled down.
        /// Does not apply to RC channels.
        /// </summary>
        public SmcPinMode pinMode = SmcPinMode.Floating;

        /// <summary>
        /// rawValues greater than this generate an error.
        /// </summary>
        public UInt16 errorMin;

        /// <summary>
        /// rawValues less than this generate an error.
        /// </summary>
        public UInt16 errorMax;

        /// <summary>
        /// rawValues greater than or equal to this get mapped to a
        /// speed of -reverseLimits.maxSpeed (or forwardLimits.maxSpeed
        /// if invert==true).
        /// </summary>
        public UInt16 inputMin;

        /// <summary>
        /// rawValues less than or equal to this get mapped to a
        /// speed of forwardLimits.maxSpeed (or -reverseLimits.maxSpeed
        /// if invert==true).
        /// </summary>
        public UInt16 inputMax;

        /// <summary>
        /// rawValues between inputNeutralMin and inputNeutralMax get mapped
        /// to a speed of zero.
        /// </summary>
        public UInt16 inputNeutralMin;

        /// <summary>
        /// rawValues between inputNeutralMin and inputNeutralMax get mapped
        /// to a speed of zero.
        /// </summary>
        public UInt16 inputNeutralMax;

        /// <summary>
        /// Returns the default settings for RC channels.
        /// </summary>
        public static SmcChannelSettings defaultRCSettings()
        {
            var cs = new SmcChannelSettings();
            cs.errorMin = 500 * 4;
            cs.errorMax = 2500 * 4;

            cs.inputMin = 1000 * 4;
            cs.inputMax = 2000 * 4;

            cs.inputNeutralMin = 1475 * 4;
            cs.inputNeutralMax = 1525 * 4;

            return cs;
        }

        /// <summary>
        /// Returns the default settings for Analog channels.
        /// </summary>
        public static SmcChannelSettings defaultAnalogSettings()
        {
            var cs = new SmcChannelSettings();
            cs.errorMin = 0;
            cs.errorMax = 4095;

            cs.inputMin = 40;
            cs.inputMax = 4055;

            cs.inputNeutralMin = 2015;
            cs.inputNeutralMax = 2080;
            return cs;
        }

        /// <summary>
        /// Returns the default settings for the given channel.
        /// </summary>
        /// <param name="channel"></param>
        /// <returns></returns>
        public static SmcChannelSettings defaults(SmcChannel channel)
        {
            if (channel.type() == SmcChannelType.Analog)
            {
                return SmcChannelSettings.defaultAnalogSettings();
            }
            else
            {
                return SmcChannelSettings.defaultRCSettings();
            }
        }

        /// <summary>
        /// Default constructor.
        /// </summary>
        public SmcChannelSettings() { }

        internal SmcChannelSettingsStruct convertToStruct()
        {
            SmcChannelSettingsStruct s = new SmcChannelSettingsStruct();
            s.invert = this.invert;
            s.scalingDegree = this.scalingDegree;
            s.alternateUse = this.alternateUse;
            s.pinMode = this.pinMode;
            s.errorMin = this.errorMin;
            s.errorMax = this.errorMax;
            s.inputMin = this.inputMin;
            s.inputMax = this.inputMax;
            s.inputNeutralMin = this.inputNeutralMin;
            s.inputNeutralMax = this.inputNeutralMax;
            return s;
        }

        internal SmcChannelSettings(SmcChannelSettingsStruct s)
        {
            this.invert = s.invert;
            this.scalingDegree = s.scalingDegree;
            this.alternateUse = s.alternateUse;
            this.pinMode = s.pinMode;
            this.errorMin = s.errorMin;
            this.errorMax = s.errorMax;
            this.inputMin = s.inputMin;
            this.inputMax = s.inputMax;
            this.inputNeutralMin = s.inputNeutralMin;
            this.inputNeutralMax = s.inputNeutralMax;
        }

        /// <summary>
        /// Creates an independent copy of this object.
        /// </summary>
        public object Clone()
        {
            return new SmcChannelSettings(this.convertToStruct());
        }

        /// <summary>
        /// Compares this object to another and sees if they have the same values.
        /// </summary>
        public override bool Equals(object x)
        {
            SmcChannelSettings s = x as SmcChannelSettings;
            if (s == null)
            {
                return false;
            }
            return this.convertToStruct().Equals(s.convertToStruct());
        }

        /// <summary>
        /// Returns the hash code for this instance.
        /// </summary>
        public override int GetHashCode()
        {
            return this.convertToStruct().GetHashCode();
        }
    }

    /// <summary>
    /// Specifies motor limits for a particular direction.
    /// </summary>
    public class SmcMotorLimits : ICloneable
    {
        /// <summary>
        /// Max Speed is a number between 0 and 3200 that specifies the maximum speed at
        /// which the motor controller will ever drive the motor.
        /// </summary>
        public UInt16 maxSpeed = 3200;

        /// <summary>
        /// Max Acceleration is a number between 0 and 3200 that specifies how much the
        /// magnitude (absolute value) of the motor speed is allowed to increase every
        /// speed update period.  0 means no limit.
        /// </summary>
        public UInt16 maxAcceleration = 0;

        /// <summary>
        /// Max Deceleration is a number between 0 and 3200 that specifies how much the
        /// magnitude (absolute value) of the motor speed is allowed to decrease every
        /// speed update period.  0 means no limit.
        /// </summary>
        public UInt16 maxDeceleration = 0;

        /// <summary>
        /// Brake duration is the time, in milliseconds, that the motor controller will
        /// spend braking the motor (Current Speed = 0) before allowing the Current Speed
        /// to change signs.
        /// </summary>
        public UInt16 brakeDuration = 0;

        /// <summary>
        /// Minimum non-zero speed (1-3200).  In RC or Analog mode, a scaled value of 1 maps to this speed.
        /// 0 means no effect.
        /// </summary>
        public UInt16 startingSpeed = 0;

        /// <summary>
        /// Default constructor.
        /// </summary>
        public SmcMotorLimits() { }

        internal SmcMotorLimits(SmcMotorLimitsStruct s)
        {
            this.maxSpeed = s.maxSpeed;
            this.maxAcceleration = s.maxAcceleration;
            this.maxDeceleration = s.maxDeceleration;
            this.brakeDuration = s.brakeDuration;
            this.startingSpeed = s.startingSpeed;
        }

        internal SmcMotorLimitsStruct convertToStruct()
        {
            SmcMotorLimitsStruct s = new SmcMotorLimitsStruct();
            s.maxSpeed = this.maxSpeed;
            s.maxAcceleration = this.maxAcceleration;
            s.maxDeceleration = this.maxDeceleration;
            s.brakeDuration = this.brakeDuration;
            s.startingSpeed = this.startingSpeed;
            return s;
        }

        /// <summary>
        /// Creates an independent copy of this object.
        /// </summary>
        public object Clone()
        {
            return new SmcMotorLimits(this.convertToStruct());
        }

        /// <summary>
        /// Compares this object to another to see if they have the same values.
        /// </summary>
        public override bool Equals(object x)
        {
            SmcMotorLimits s = x as SmcMotorLimits;
            if (s == null)
            {
                return false;
            }
            return this.convertToStruct().Equals(s.convertToStruct());
        }

        /// <summary>
        /// Returns the hash code of this instance.
        /// </summary>
        public override int GetHashCode()
        {
            return this.convertToStruct().GetHashCode();
        }
    }

    /// <summary>
    /// This class represents all the settings for a Simple Motor Controller,
    /// including the channel settings and hard motor limits.
    /// </summary>
    public class SmcSettings : ICloneable
    {
        // [Add-new-settings-here]

        /// <summary>If true, then never enter USB suspend mode.</summary>
        public Boolean neverSuspend = false;

        /// <summary>
        /// If true, then insert a one-byte delay before sending serial responses.
        /// This allows slower processors time to get ready to receive the response.
        /// </summary>
        public Boolean uartResponseDelay = false;

        /// <summary>
        /// If true, then don't auto-detect the baud rate and instead use a fixed baud rate,
        /// specified by fixedBaudRateBps.
        /// </summary>
        public Boolean useFixedBaudRate = false;

        /// <summary>
        /// When false, puts strct requirements on what is necessary to start the motor.
        /// See the user's guide for for more information.
        /// </summary>
        public Boolean disableSafeStart = false;

        /// <summary>
        /// The value to put in to the device's baud rate register to generate the fixed
        /// baud rate.  Only relevant if useFixedBaudRate is true.
        /// </summary>
        public UInt32 fixedBaudRateBps = 9600;

        /// <summary>
        /// Time between accel/decel updates to the speed. Units of 1 ms.  Should never be 0!
        /// </summary>
        public UInt16 speedUpdatePeriod = 1;

        /// <summary>
        /// The time before a command timeout error occurs, in units of 10 ms.
        /// A value of 0 disables the command timeout feature.
        /// </summary>
        public UInt16 commandTimeout = 0;

        /// <summary>
        /// The serial device number used to address this device in the Pololu
        /// protocol.
        /// </summary>
        public Byte serialDeviceNumber = 13;

        /// <summary>
        /// Whether CRC is disabled, required for reception, or required for
        /// reception and sent during transmission.
        /// </summary>
        public SmcCrcMode crcMode = SmcCrcMode.Disabled;

        /// <summary>
        /// See tempLimitGradual.
        /// Units: tenths of a degree Celsius.
        /// </summary>
        public UInt16 overTempMin = 70*10;

        /// <summary>
        /// The temperature where speed is limited to 0.
        /// Units: tenths of a degree Celsius.
        /// </summary>
        public UInt16 overTempMax = 80*10;

        /// <summary>
        /// Input mode: Serial, RC, or Analog.
        /// </summary>
        public SmcInputMode inputMode = SmcInputMode.SerialUsb;

        /// <summary>
        /// PWM Mode: Drive/Brake or Drive/Coast.
        /// The drive/coast mode is a lot worse than drive/brake mode so we don't actually
        /// let the user use this setting.
        /// </summary>
        public SmcPwmMode pwmMode = SmcPwmMode.DriveBrake;

        /// <summary>Determines the PWM frequency (0-19, 0=highest freq).</summary>
        public Byte pwmPeriodFactor = 0;

        /// <summary>
        /// Mixing Mode: None, Left, or Right
        /// </summary>
        public SmcMixingMode mixingMode = SmcMixingMode.None;

        /// <summary>
        /// Minimum allowed time between consecutive RC pulse rising edges.
        /// Units: 1 ms.
        /// </summary>
        public UInt16 minPulsePeriod = 9;

        /// <summary>
        /// Maximum allowed time between consecutive RC pulse rising edges.
        /// If this amount of time elapses and no pulses at all have been received,
        /// then the motor is shut down and an error is generated.
        /// Units: 1 ms.
        /// </summary>
        public UInt16 maxPulsePeriod = 100;

        /// <summary>
        /// Generates error and shuts down motor if we go this long without
        /// heeding a pulse (units of 1 ms).
        /// </summary>
        public UInt16 rcTimeout = 500;

        /// <summary>
        /// If false, check for pot disconnects by toggling the pot power pins.
        /// </summary>
        public Boolean ignorePotDisconnect = false;

        /// <summary>
        /// False: abrupt limit past overTempMax asserted until temp falls below overTempMin.
        /// True: gradual speed limit starting at overTempMin.
        /// </summary>
        public Boolean tempLimitGradual = false;

        /// <summary>
        /// Number of previously received consecutive good pulses needed
        /// to heed the latest pulse received and update the channel's raw value.
        /// 0 means no previous good pulses are required, so we update the channel's
        /// value on every good pulse.
        /// </summary>
        public Byte consecGoodPulses = 2;

        /// <summary>
        /// Invert Motor Direction.  Boolean that specifies the correspondence between
        /// speed (-3200 to 3200) and voltages on OUTA/OUTB.
        /// Normally, a speed of 3200 means OUTA~VIN, OUTB=0.
        /// If motorInvert is true, then a speed of 3200 means OUTA=0, OUTB~VIN.
        /// </summary>
        public Boolean motorInvert;

        /// <summary>
        /// Brake amount while input is in deadband (0-32), or there is an error,
        /// or motor is driving at speed zero.
        /// </summary>
        public Byte speedZeroBrakeAmount = 32;

        /// <summary>
        /// Stop the motor if the ERR line is high (allows you to connect the
        /// error lines of two devices and have them both stop when one has an error).
        /// </summary>
        public Boolean ignoreErrLineHigh = false;

        /// <summary>
        /// The VIN multiplier offset is a calibration factor used in computing VIN.
        /// The default value of 0 should work fine for most purposes.
        /// </summary>
        public Int16 vinMultiplierOffset = 0;

        /// <summary>
        /// VIN must stay below lowVinShutoffMv for this duration before a low-VIN error
        /// occurs (units of 1 ms).
        /// </summary>
        public UInt16 lowVinShutoffTimeout = 250;

        /// <summary>Dropping below this voltage threshold triggers a low-voltage error (units of mV).</summary>
        public UInt16 lowVinShutoffMv = 5500;

        /// <summary>
        /// Once asserting a low-voltage error, the voltage required to stop asserting this error (units of mV).
        /// </summary>
        public UInt16 lowVinStartupMv = 6000;

        /// <summary>
        /// Rising above this voltage threshold triggers a high-voltage error and
        /// causes the motor to immediately brake at 100% (units of mV).
        /// </summary>
        public UInt16 highVinShutoffMv = 25000;

        /// <summary>
        /// Determines what types of commands are accepted and whether to echo incoming bytes.
        /// </summary>
        public SmcSerialMode serialMode = SmcSerialMode.Binary;

        /// <summary>
        /// Settings for RC Channel 1.
        /// </summary>
        public SmcChannelSettings rc1;

        /// <summary>
        /// Settings for RC Channel 2.
        /// </summary>
        public SmcChannelSettings rc2;

        /// <summary>
        /// Settings for Analog Channel 1.
        /// </summary>
        public SmcChannelSettings analog1;

        /// <summary>
        /// Settings for Analog Channel 2.
        /// </summary>
        public SmcChannelSettings analog2;

        /// <summary>
        /// Hard motor limits in the forward direction.
        /// </summary>
        public SmcMotorLimits forwardLimits;

        /// <summary>
        /// Hard motor limits in the reverse direction.
        /// </summary>
        public SmcMotorLimits reverseLimits;

        /// <summary>
        /// Constructs a new SmcSettings object that has the default settings for a particular product.
        /// </summary>
        /// <param name="productId">The product ID of the device.  This determines the initial value of
        /// some product-dependent parameters like highVinShutoffMv.  Pass this argument as 0 if the
        /// product ID is unknown at this time.
        /// </param>
        public SmcSettings(UInt16 productId)
        {
            rc1 = SmcChannelSettings.defaultRCSettings();
            rc2 = SmcChannelSettings.defaultRCSettings();
            analog1 = SmcChannelSettings.defaultAnalogSettings();
            analog2 = SmcChannelSettings.defaultAnalogSettings();
            forwardLimits = new SmcMotorLimits();
            reverseLimits = new SmcMotorLimits();

            if (productId == 0x99 || productId == 0x9D)
            {
                // For the versions with 40V MOSFETs:
                this.highVinShutoffMv = 35000;
            }
            else
            {
                // For the versions with 30V MOSFETs:
                this.highVinShutoffMv = 25000;
            }
        }

        internal SmcSettings(SmcSettingsStruct s)
        {
            // [Add-new-settings-here]
            this.neverSuspend = s.neverSuspend;
            this.uartResponseDelay = s.uartResponseDelay;
            this.useFixedBaudRate = s.useFixedBaudRate;
            this.disableSafeStart = s.disableSafeStart;
            this.fixedBaudRateBps = Smc.convertBaudRegisterToBps(s.fixedBaudRateRegister);
            this.speedUpdatePeriod = s.speedUpdatePeriod;
            this.commandTimeout = s.commandTimeout;
            this.serialDeviceNumber = s.serialDeviceNumber;
            this.crcMode = s.crcMode;
            this.overTempMin = s.overTempMin;
            this.overTempMax = s.overTempMax;
            this.inputMode = s.inputMode;
            this.pwmMode = s.pwmMode;
            this.pwmPeriodFactor = s.pwmPeriodFactor;
            this.mixingMode = s.mixingMode;
            this.minPulsePeriod = s.minPulsePeriod;
            this.maxPulsePeriod = s.maxPulsePeriod;
            this.rcTimeout = s.rcTimeout;
            this.ignorePotDisconnect = s.ignorePotDisconnect;
            this.tempLimitGradual = s.tempLimitGradual;
            this.consecGoodPulses = s.consecGoodPulses;
            this.motorInvert = s.motorInvert;
            this.speedZeroBrakeAmount = s.speedZeroBrakeAmount;
            this.ignoreErrLineHigh = s.ignoreErrLineHigh;
            this.vinMultiplierOffset = s.vinMultiplierOffset;
            this.lowVinShutoffTimeout = s.lowVinShutoffTimeout;
            this.lowVinShutoffMv = s.lowVinShutoffMv;
            this.lowVinStartupMv = s.lowVinStartupMv;
            this.highVinShutoffMv = s.highVinShutoffMv;
            this.serialMode = s.serialMode;
            this.rc1 = new SmcChannelSettings(s.rc1);
            this.rc2 = new SmcChannelSettings(s.rc2);
            this.analog1 = new SmcChannelSettings(s.analog1);
            this.analog2 = new SmcChannelSettings(s.analog2);
            this.forwardLimits = new SmcMotorLimits(s.forwardLimits);
            this.reverseLimits = new SmcMotorLimits(s.reverseLimits);
        }

        internal SmcSettingsStruct convertToStruct()
        {
            SmcSettingsStruct s = new SmcSettingsStruct();
            // [Add-new-settings-here]
            s.neverSuspend = this.neverSuspend;
            s.uartResponseDelay = this.uartResponseDelay;
            s.useFixedBaudRate = this.useFixedBaudRate;
            s.disableSafeStart = this.disableSafeStart;
            s.fixedBaudRateRegister = Smc.convertBpsToBaudRegister(this.fixedBaudRateBps);
            s.speedUpdatePeriod = this.speedUpdatePeriod;
            s.commandTimeout = this.commandTimeout;
            s.serialDeviceNumber = this.serialDeviceNumber;
            s.crcMode = this.crcMode;
            s.overTempMin = this.overTempMin;
            s.overTempMax = this.overTempMax;
            s.inputMode = this.inputMode;
            s.pwmMode = this.pwmMode;
            s.pwmPeriodFactor = this.pwmPeriodFactor;
            s.mixingMode = this.mixingMode;
            s.minPulsePeriod = this.minPulsePeriod;
            s.maxPulsePeriod = this.maxPulsePeriod;
            s.rcTimeout = this.rcTimeout;
            s.ignorePotDisconnect = this.ignorePotDisconnect;
            s.tempLimitGradual = this.tempLimitGradual;
            s.consecGoodPulses = this.consecGoodPulses;
            s.motorInvert = this.motorInvert;
            s.speedZeroBrakeAmount = this.speedZeroBrakeAmount;
            s.ignoreErrLineHigh = this.ignoreErrLineHigh;
            s.vinMultiplierOffset = this.vinMultiplierOffset;
            s.lowVinShutoffTimeout = this.lowVinShutoffTimeout;
            s.lowVinShutoffMv = this.lowVinShutoffMv;
            s.lowVinStartupMv = this.lowVinStartupMv;
            s.highVinShutoffMv = this.highVinShutoffMv;
            s.serialMode = this.serialMode;
            s.rc1 = this.rc1.convertToStruct();
            s.rc2 = this.rc2.convertToStruct();
            s.analog1 = this.analog1.convertToStruct();
            s.analog2 = this.analog2.convertToStruct();
            s.forwardLimits = this.forwardLimits.convertToStruct();
            s.reverseLimits = this.reverseLimits.convertToStruct();
            return s;
        }

        /// <summary>
        /// Creates an independent copy of this object.
        /// </summary>
        public object Clone()
        {
            return new SmcSettings(this.convertToStruct());
        }

        /// <summary>
        /// Compares this object to another to see if they have the same values.
        /// </summary>
        public override bool Equals(object x)
        {
            SmcSettings s = x as SmcSettings;
            if (s == null)
            {
                return false;
            }
            return this.convertToStruct().Equals(s.convertToStruct());
        }

        /// <summary>
        /// Returns the hash code for this instance.
        /// </summary>
        public override int GetHashCode()
        {
            return this.convertToStruct().GetHashCode();
        }
    }
}
