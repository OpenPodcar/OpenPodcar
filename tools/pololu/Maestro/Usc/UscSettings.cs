using System;
using System.Collections.Generic;
using System.Text;
using Pololu.Usc.Bytecode;
using Pololu.Usc.Sequencer;

namespace Pololu.Usc
{
    /// <summary>
    /// This class represents all the settings for a Maestro.
    /// This includes every parameter that is stored on the device, as well
    /// as servo names, sequences, and the compiled script (BytecodeProgram).
    /// </summary>
    public class UscSettings
    {
        /// <summary>
        /// The number of servo ports available (0-5).  This, along with the
        /// servoPeriod, determine the "maximum maximum pulse width".
        /// </summary>
        public Byte servosAvailable = 6;

        /// <summary>
        /// This setting only applies to the Micro Maestro.
        /// For the Mini Maestro, see miniMaestroServoPeriod.
        /// 
        /// The total time allotted to each servo channel, in units of
        /// 256/12 = 21.33333 us.  The unit for this one are unusual, because
        /// that is the way it is stored on the device and its unit is not
        /// a multiple of 4, so we would have inevitable rounding errors if we
        /// tried to represent it in quarter-microseconds.
        /// 
        /// Default is 156, so with 6 servos available you get ~20ms between
        /// pulses on a given channel.
        /// </summary>
        public Byte servoPeriod = 156;

        /// <summary>
        /// This setting only applies to the Mini Maestro.
        /// For the Micro Maestro, see microMaestroServoPeriod.
        /// 
        /// The length of the time period in which the Mini Maestro sends pulses
        /// to all the enabled servos, in units of quarter microseconds.
        /// 
        /// Valid values for this parameter are 0 to 16,777,215.  But 
        /// 
        /// Default is 80000, so each servo receives a pulse every 20 ms (50 Hz).
        /// </summary>
        public UInt32 miniMaestroServoPeriod = 80000;

        /// <summary>
        /// This setting only applied to the Mini Maestro.
        /// The non-multiplied servos have a period specified by miniMaestroServoPeriod.
        /// The multiplied servos have a period specified by miniMaestroServoPeriod*servoMultiplier.
        /// 
        /// Valid values for this parameter are 1 to 256.
        /// </summary>
        public UInt16 servoMultiplier = 1;

        /// <summary>
        /// Determines how serial bytes flow between the two USB COM ports, the TTL port,
        /// and the Maestro's serial command processor.
        /// </summary>
        public uscSerialMode serialMode = uscSerialMode.SERIAL_MODE_UART_DETECT_BAUD_RATE;

        /// <summary>
        /// The fixed baud rate, in units of bits per second.  This gets stored in a
        /// different format on the usc.cs, so there will be rounding errors
        /// which get bigger at higher baud rates, but they will be less than
        /// 1% for baud rates of 120000 or less.
        /// 
        /// This parameter only applies if serial mode is USB UART Fixed Baud.
        /// 
        /// All values above 184 are valid, but values significantly higher than
        /// 250000 are subject to high rounding errors and the usc firmware might not
        /// be able to keep up with those higher data rates.  If the baud rate is too
        /// high and the firmware can't keep up, the Maestro will indicate this to you
        /// by generating a serial overrun or buffer full error.
        /// </summary>
        public UInt32 fixedBaudRate = 9600;

        /// <summary>
        /// If true, then you must send a 7-bit CRC byte at the end of every serial
        /// command (except the Mini SSC II command).
        /// </summary>
        public bool enableCrc = false;

        /// <summary>
        /// If true, then the Maestro will never go to sleep.  This lets you power 
        /// the processer off of USB even when the computer has gone to sleep and put
        /// all of its USB devices in the suspend state.
        /// </summary>
        public bool neverSuspend = false;

        /// <summary>
        /// The serial device number used to identify this device in Pololu protocol
        /// commands.  Valid values are 0-127, default is 12.
        /// </summary>
        public Byte serialDeviceNumber = 12;

        /// <summary>
        /// The offset used to determine which Mini SSC commands this device will
        /// respond to.  The second byte of the Mini SSC command contains the servo
        /// number; the correspondence between servo number and maestro number (0-5)
        /// is servo# = miniSSCoffset + channel#.  Valid values are 0-254.
        /// </summary>
        public Byte miniSscOffset = 0;

        /// <summary>
        /// The time it takes for a serial timeout error to occur, in units of 10 ms.
        /// A value of 0 means no timeout error will occur.  All values 0-65535 are valid.
        /// </summary>
        public UInt16 serialTimeout = 0;

        /// <summary>
        /// True if the script should not be started when the device starts up.
        /// False if the script should be started.
        /// </summary>
        public bool scriptDone = true;

        /// <summary>
        /// A list of the configurable parameters for each channel, including
        /// name, type, home type, home position, range, neutral, min, max.
        /// </summary>
        public List<ChannelSetting> channelSettings;

        /// <summary>
        /// If true, this setting enables pullups for each channel 18-20 which
        /// is configured as an input.  This makes the input value be high by
        /// default, allowing the user to connect a button or switch without
        /// supplying their own pull-up resistor.  Thi setting only applies to
        /// the Mini Maestro 24-Channel Servo Controller.
        /// </summary>
        public bool enablePullups = false;

        /// <summary>
        /// true if when loading the script, the checksum did not match or there was an error in compilation, so that it had to be reset to an empty script
        /// </summary>
        public bool scriptInconsistent = false;

        private string privateScript = null;
        private BytecodeProgram privateProgram;

        public string script
        {
            get { return privateScript; }
            // There is no set accessor here.  The only way to set the script string is to call
            // setAndCompileScript.  We don't want to have a set accessor here because the script
            // needs to be compiled, which can take a second, so it should not be set casually.
        }

        public void setAndCompileScript(string script)
        {
            privateScript = null;

            privateProgram = BytecodeReader.Read(script, servoCount != 6);

            // If no exceptions were raised, set the script.
            privateScript = script;
        }

        public decimal periodInMicroseconds
        {
            get
            {
                if (servoCount == 6)
                {
                    return Usc.periodToMicroseconds(servoPeriod, servosAvailable);
                }
                else
                {
                    return miniMaestroServoPeriod / 4;
                }
            }
        }

    	public List<Sequence> sequences = new List<Sequence>();

        public BytecodeProgram bytecodeProgram
        {
            get { return privateProgram; }
        }

        public UscSettings()
        {
            channelSettings = new List<ChannelSetting>();
        }

        /// <summary>
        /// The number of servos on the device.
        /// </summary>
        public byte servoCount
        {
            get
            {
                return (byte)channelSettings.Count;
            }
        }
    }

    /// <summary>
    /// An object that represents the settings for one servo,
    /// e.g. the information in the Settings tab.  One of these objects
    /// corresponds to one ServoSettingsControl.
    /// </summary>
    public class ChannelSetting
    {
        /// <summary>
        /// Name.  The Usc class stores this in the registry, not the device.
        /// </summary>
        public String name = "";

        /// <summary>
        /// Type (servo, output, input).
        /// </summary>
        public ChannelMode mode = ChannelMode.Servo;

        /// <summary>
        /// HomeType (off, ignore, goto).
        /// </summary>
        public HomeMode homeMode = HomeMode.Off;

        /// <summary>
        /// Home position: the place to go on startup.
        /// If type==servo, units are 0.25 us (qus).
        /// If type==output, the threshold between high and low is 1500.
        /// 
        /// This value is only saved on the device if homeType == Goto.
        /// </summary>
        public UInt16 home = 6000;
        
        /// <summary>
        /// Minimum (units of 0.25 us, but stored on the device in units of 16 us).
        /// </summary>
        public UInt16 minimum = 3968;

        /// <summary>
        /// Maximum (units of 0.25 us, but stored on the device in units of 16 us).
        /// </summary>
        public UInt16 maximum = 8000;

        /// <summary>
        /// Neutral: the center of the 8-bit set target command (value at 127).
        /// If type==servo, units are 0.25 us (qus).
        /// If type==output, the threshold between high and low is 1500.
        /// </summary>
        public UInt16 neutral = 6000;

        /// <summary>
        /// Range: the +/- extent of the 8-bit command.
        ///   8-bit(254) = neutral + range,
        ///   8-bit(0) = neutral - range
        /// If type==servo units are 0.25 us (qus) (but stored on device in
        /// units of 127*0.25us = 31.75 us.
        /// Range = 0-127*255 = 0-32385 qus.
        /// Increment = 127 qus
        /// </summary>
        public UInt16 range = 1905;

        /// <summary>
        /// Speed: the maximum change in position (qus) per update.  0 means no limit.
        /// Units depend on your settings.
        /// Stored on device in this format: [0-31]*2^[0-7]
        /// Range = 0-31*2^7 = 0-3968.
        /// Increment = 1.
        /// 
        /// Note that the *current speed* is stored on the device in units
        /// of qus, and so it is not subject to the restrictions above!
        /// It can be any value 0-65535.
        /// </summary>
        public UInt16 speed = 0;

        /// <summary>
        /// Acceleration: the max change in speed every 80 ms.  0 means no limit.
        /// Units depend on your settings.
        /// Range = 0-255.
        /// Increment = 1.
        /// </summary>
        public Byte acceleration = 0;
    }

    public enum ChannelMode
    {
        Servo=0,
        ServoMultiplied=1,
        Output=2,
        Input = 3,
    }

    public enum HomeMode
    {
        Off,
        Ignore,
        Goto
    }
}
