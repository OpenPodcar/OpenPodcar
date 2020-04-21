using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Pololu.UsbWrapper;

namespace Pololu.UsbAvrProgrammer
{
    public class Programmer : UsbDevice
    {
        /// <summary>
        /// The device interface GUID used to detect the native USB interface
        /// of the USB AVR Programmer.
        /// </summary>
        /// <remarks>From maestro.inf.</remarks>
        public static Guid deviceInterfaceGuid = new Guid("7892a772-3cac-4440-adcb-cc3e16f79f6d");

        /// <summary>Pololu's USB vendor id.</summary>
        /// <value>0x1FFB</value>
        public const ushort vendorID = 0x1ffb;

        /// <summary>The Micro Maestro's product ID.</summary>
        /// <value>0x0089</value>
        public const ushort productID = 0x0081;

        /// <value>USB AVR Programmer</value>
        public const string englishName = "USB AVR Programmer";

        /// <value>Maestro</value>
        public const string shortProductName = "Programmer";

        /// <summary>
        /// An array of strings needed to detect which bootloaders are connected.
        /// when doing firmware upgrades.
        /// </summary>
        public static string[] bootloaderDeviceInstanceIdPrefixes
        {
            get
            {
                return new string[] { "USB\\VID_1FFB&PID_0080" };
            }
        }

        public Programmer(DeviceListItem deviceListItem) : base(deviceListItem)
        {
        }
        
        public static List<DeviceListItem> getConnectedDevices()
        {
            try
            {
                return UsbDevice.getDeviceList(Programmer.deviceInterfaceGuid);
            }
            catch (NotImplementedException)
            {
                // use vendor and product instead
                return UsbDevice.getDeviceList(Programmer.vendorID, new ushort[] { Programmer.productID });
            }
        }
        
        Byte privateFirmwareVersionMajor = 0xFF;
        Byte privateFirmwareVersionMinor = 0xFF;

        public UInt16 firmwareVersionMajor
        {
            get
            {
                if (privateFirmwareVersionMajor == 0xFF)
                {
                    getFirmwareVersion();
                }
                return privateFirmwareVersionMajor;
            }
        }

        public Byte firmwareVersionMinor
        {
            get
            {
                if (privateFirmwareVersionMajor == 0xFF)
                {
                    getFirmwareVersion();
                }
                return privateFirmwareVersionMinor;
            }
        }

        public String firmwareVersionString
        {
            get
            {
                return firmwareVersionMajor.ToString() + "." + firmwareVersionMinor.ToString("D2");
            }
        }

        void getFirmwareVersion()
        {
            Byte[] buffer = new Byte[14];

            try
            {
                controlTransfer(0x80, 6, 0x0100, 0x0000, buffer);
            }
            catch (Exception exception)
            {
                throw new Exception("There was an error getting the firmware version from the device.", exception);
            }

            privateFirmwareVersionMinor = (Byte)((buffer[12] & 0xF) + (buffer[12] >> 4 & 0xF) * 10);
            privateFirmwareVersionMajor = (Byte)((buffer[13] & 0xF) + (buffer[13] >> 4 & 0xF) * 10);

            // Correct the incorrect value of bcdDevice in the original firmware version:
            // 0x0001 really means 0x0100.
            if (privateFirmwareVersionMajor == 0 && privateFirmwareVersionMinor == 1)
            {
                privateFirmwareVersionMajor = 1;
                privateFirmwareVersionMinor = 0;
            }

        }

        public void startBootloader()
        {
            try
            {
                controlTransfer(0x40, 0xFF, 0, 0);
            }
            catch (Exception e)
            {
                throw new Exception("There was an error entering bootloader mode.", e);
            }
        }

        private void setVariable(VariableId variableId, ushort value)
        {
            try
            {
                controlTransfer(0x40, 0x82, value, (ushort)variableId);
            }
            catch (Exception e)
            {
                throw new Exception("There was an error setting variable " + variableId.ToString() + " on the device.", e);
            }
        }

        private unsafe ushort getVariable(VariableId variableId)
        {
            byte bytes = getVariableBytes(variableId);

            ushort value = 0;
            byte[] array = new byte[bytes];
            try
            {
                controlTransfer(0xC0, 0x81, 0, (ushort)variableId, array);
            }
            catch (Exception e)
            {
                throw new Exception("There was an error getting variable " + variableId.ToString() + " from the device.", e);
            }
            if (bytes == 1)
            {
                // read a single byte
                fixed (byte* pointer = array)
                {
                    value = *(byte*)pointer;
                }
            }
            else
            {
                // read two bytes
                fixed (byte* pointer = array)
                {
                    value = *(ushort*)pointer;
                }
            }
            return value;
        }

        /// <summary>
        /// Returns the number of bytes that the variable takes.
        /// </summary>
        private byte getVariableBytes(VariableId variableId)
        {
            switch (variableId)
            {
                case VariableId.TARGET_VCC_ALLOWED_MINIMUM:
                case VariableId.TARGET_VCC_ALLOWED_MAXIMUM_RANGE:
                case VariableId.TARGET_VCC_MEASURED_MINIMUM:
                case VariableId.TARGET_VCC_MEASURED_MAXIMUM:
                case VariableId.PROGRAMMING_ERROR:
                case VariableId.LINE_A_IDENTITY:
                case VariableId.LINE_B_IDENTITY:
                case VariableId.SLOSCOPE_STATE:
                case VariableId.SW_MINOR:
                case VariableId.SW_MAJOR:
                case VariableId.HW_VER:
                case VariableId.SCK_DURATION:
                    return 1;

                case VariableId.FVR_ADC:
                case VariableId.SLOSCOPE_OUTPUT_STATE:
                    return 2;

                default: throw new Exception("Unrecognized variabledId " + variableId.ToString());
            }
        }

        public void setSettings(ProgrammerSettings settings)
        {
            setTargetVccAllowedMinimum(settings.targetVccAllowedMinimum);
            setTargetVccAllowedMaximumRange(settings.targetVccAllowedMaximumRange);
            setLineAIdentity(settings.lineAIdentity);
            setLineBIdentity(settings.lineBIdentity);
            setSoftwareVersionMinor(settings.softwareVersionMinor);
            setSoftwareVersionMajor(settings.softwareVersionMajor);
            setHardwareVersion(settings.hardwareVersion);
            setSckDuration(settings.sckDuration);
        }

        public void setTargetVccAllowedMinimum(ushort value)
        {
            setVariable(VariableId.TARGET_VCC_ALLOWED_MINIMUM, (ushort)(value / 32));
        }

        public void setTargetVccAllowedMaximumRange(ushort value)
        {
            setVariable(VariableId.TARGET_VCC_ALLOWED_MAXIMUM_RANGE, (ushort)(value / 32));
        }

        public void setLineAIdentity(LineIdentity value)
        {
            setVariable(VariableId.LINE_A_IDENTITY, (byte)value);
        }

        public void setLineBIdentity(LineIdentity value)
        {
            setVariable(VariableId.LINE_B_IDENTITY, (byte)value);
        }

        public void setSoftwareVersionMinor(byte value)
        {
            setVariable(VariableId.SW_MINOR, value);
        }

        public void setSoftwareVersionMajor(byte value)
        {
            setVariable(VariableId.SW_MAJOR, value);
        }

        public void setHardwareVersion(byte value)
        {
            setVariable(VariableId.HW_VER, value);
        }

        public void setSckDuration(SckDuration value)
        {
            setVariable(VariableId.SCK_DURATION, (byte)value);
        }

        

        public ProgrammerSettings getSettings()
        {
            var settings = new ProgrammerSettings();

            settings.targetVccAllowedMinimum = (ushort)(32*getVariable(VariableId.TARGET_VCC_ALLOWED_MINIMUM));
            settings.targetVccAllowedMaximumRange = (ushort)(32*getVariable(VariableId.TARGET_VCC_ALLOWED_MAXIMUM_RANGE));
            settings.lineAIdentity = (LineIdentity)getVariable(VariableId.LINE_A_IDENTITY);
            settings.lineBIdentity = (LineIdentity)getVariable(VariableId.LINE_B_IDENTITY);
            settings.softwareVersionMinor = (byte)getVariable(VariableId.SW_MINOR);
            settings.softwareVersionMajor = (byte)getVariable(VariableId.SW_MAJOR);
            settings.hardwareVersion = (byte)getVariable(VariableId.HW_VER);
            settings.sckDuration = (SckDuration)getVariable(VariableId.SCK_DURATION);

            return settings;
        }

        public void restoreDefaultSettings()
        {
            setSettings(new ProgrammerSettings());
        }

        public static string lineIdentityToString(LineIdentity lineIdentity)
        {
            switch (lineIdentity)
            {
                case LineIdentity.DTR: return "DTR (output)";
                case LineIdentity.RTS: return "RTS (output)";
                case LineIdentity.CD: return "CD (input)";
                case LineIdentity.DSR: return "DSR (input)";
                case LineIdentity.RI: return "RI (input)";
                default: return "None";
            }
        }

        /// <summary>
        /// Returns the reason the last programming session was exited.
        /// If the programmer has not programmed since it last powered on,
        /// returns ProgrammingError.None.
        /// </summary>
        public ProgrammingError getProgrammingError()
        {
            return (ProgrammingError)getVariable(VariableId.PROGRAMMING_ERROR);
        }

        /// <summary>
        /// Returns lowest target VCC reading observed during the last
        /// programming session, in units of mV.  If the programmer has not
        /// programmed since it last powered on, returns 32*255.
        /// </summary>
        public ushort getTargetVccMeasuredMinimum()
        {
            return (ushort)(32*getVariable(VariableId.TARGET_VCC_MEASURED_MINIMUM));
        }

        /// <summary>
        /// Returns highest target VCC reading observed during the last
        /// programming session, in units of mV.  If the programmer has not
        /// programmed since it last powered on, returns 0.
        /// </summary>
        public ushort getTargetVccMeasuredMaximum()
        {
            return (ushort)(32*getVariable(VariableId.TARGET_VCC_MEASURED_MAXIMUM));
        }

        public SloscopeState getSloscopeState()
        {
            return (SloscopeState)getVariable(VariableId.SLOSCOPE_STATE);
        }

        public void setSloscopeState(SloscopeState value)
        {
            setVariable(VariableId.SLOSCOPE_STATE, (byte)value);
        }

        /// <summary>
        /// Returns the sloscope output state (off, low, high) of lines A and B.
        /// </summary>
        public void getSloscopeOutputState(out SloscopeOutputState outputA, out SloscopeOutputState outputB)
        {
            ushort state = getVariable(VariableId.SLOSCOPE_OUTPUT_STATE);
            outputA = (SloscopeOutputState)(state & 0xFF);
            outputB = (SloscopeOutputState)(state >> 8);
        }

        /// <summary>
        /// The ID numbers of all the variables that the programmer recognizes
        /// the set/get variable requests.  Persistent means it is stored in
        /// persistent memory.  Read-only means you can read it but not set it.
        /// </summary>
        private enum VariableId
        {
            TARGET_VCC_ALLOWED_MINIMUM        = 0x01, // Persistent. 1 byte, units of 32 mV. 
            TARGET_VCC_ALLOWED_MAXIMUM_RANGE  = 0x02, // Persistent. 1 byte, units of 32 mV.
            TARGET_VCC_MEASURED_MINIMUM       = 0x03, // Read-only.  1 byte, units of 32 mV.
            TARGET_VCC_MEASURED_MAXIMUM       = 0x04, // Read-only.  1 byte, units of 32 mV.
            PROGRAMMING_ERROR                 = 0x08, // Read-only.  1 byte enum.
            LINE_A_IDENTITY                   = 0x20, // Persistent.  1 byte enum.
            LINE_B_IDENTITY                   = 0x21, // Persistent.  1 byte enum.
            FVR_ADC                           = 0x41, // Read-only.  2 bytes, performs and returns 10-bit ADC reading of 1024 mV.
            SLOSCOPE_STATE                    = 0x42, // 1 byte enum.
            SLOSCOPE_OUTPUT_STATE             = 0x43, // 2 bytes, both bytes are enum.
            SW_MINOR                          = 0x90, // Persistent.  1 byte, unitless.
            SW_MAJOR                          = 0x91, // Persistent.  1 byte, unitless.
            HW_VER                            = 0x92, // Persistent.  1 byte, unitless.
            SCK_DURATION                      = 0x98, // Persistent.  1 byte, enum.
        }

        public static string sckDurationToString(SckDuration sckDuration)
        {
            switch (sckDuration)
            {
                case SckDuration.Frequency2000: return "2000 kHz";
                case SckDuration.Frequency1500: return "1500 kHz";
                case SckDuration.Frequency750: return "750 kHz";
                case SckDuration.Frequency200: return "200 kHz";
                default:
                    if (sckDuration <= SckDuration.Frequency4)
                    {
                        return "4 kHz";
                    }
                    else
                    {
                        return "1.5 kHz";
                    }
            }
        }

        /// <summary>
        /// Converts the given frequency (in kHz) to a valid SckDuration parameter that
        /// specifies a supported frequency to the programmer.  If frequency does not
        /// exactly match a supported frequency, the next lowest frequency is used, unless
        /// there is no lower frequency supportedin, in which case the lowest frequency is used.
        /// </summary>
        /// <param name="frequency">The desired frequency in units of kHz.</param>
        public static SckDuration frequencyToSckDuration(decimal frequency)
        {
            bool tmp;
            return frequencyToSckDuration(frequency, out tmp);
        }

        /// <summary>
        /// Converts the given frequency (in kHz) to a valid SckDuration parameter that
        /// specifies a supported frequency to the programmer.  If frequency does not
        /// exactly match a supported frequency, the next lowest frequency is used, unless
        /// there is no lower frequency supportedin, in which case the lowest frequency is used.
        /// </summary>
        /// <param name="frequency">The desired frequency in units of kHz.</param>
        /// <param name="exactMatch">True if frequency exactly matches a supported frequency.</param>
        public static SckDuration frequencyToSckDuration(decimal frequency, out bool exactMatch)
        {
            exactMatch = true;
            if (frequency == 2000) { return SckDuration.Frequency2000; }
            else if (frequency == 1500) { return SckDuration.Frequency1500; }
            else if (frequency == 750) { return SckDuration.Frequency750; }
            else if (frequency == 200) { return SckDuration.Frequency200; }
            else if (frequency == 4) { return SckDuration.Frequency4; }
            else if (frequency == 1.5M) { return SckDuration.Frequency1_5; }
            exactMatch = false;

            if (frequency >= 2000) { return SckDuration.Frequency2000; }
            else if (frequency >= 1500) { return SckDuration.Frequency1500; }
            else if (frequency >= 750) { return SckDuration.Frequency750; }
            else if (frequency >= 200) { return SckDuration.Frequency200; }
            else if (frequency >= 4) { return SckDuration.Frequency4; }
            else { return SckDuration.Frequency1_5; }
        }

        //public AsynchronousInTransfer newSloscopeInTransfer()
        //{
        //    return newAsynchronousInTransfer(5, 22, 100);
        //}
    }

    /// <summary>
    /// ProgrammingError represents all the different reasons that the programmer
    /// can exit programming mode (stop programming the AVR).
    /// </summary>
    public enum ProgrammingError : byte
    {
        /// <summary>
        /// Either the last programming completed successfully, or there has
        /// been no programming attempt yet.
        /// </summary>
        None = 0x00,

        /// <summary>
        /// Target VDD either went too low or had too much range, so programming
        /// was aborted.  Make sure that the target is powered on and its
        /// batteries are not too low (if applicable).
        /// </summary>
        TargetVccBad = 0x01,

        /// <summary>
        /// The SPI command for entering programming mode was sent, but the
        /// expected response from the target was not received.  Make sure that
        /// the ISP frequency setting is less than 1/6th of the target's clock
        /// frequency.
        /// </summary>
        Synch = 0x02,

        /// <summary>
        /// The programmer received no programming commands from the computer
        /// for more than 0.3 seconds, so it left programming mode.  Make sure
        /// that the programming software does not wait too long between
        /// successive programming commands.
        /// </summary>
        IdleForTooLong = 0x03,

        /// <summary>
        /// The computer's USB controller deconfigured the progammer, so
        /// programming was aborted.
        /// </summary>
        /// <remarks>Very unlikely.</remarks>
        UsbNotConfigured = 0x04,

        /// <summary>
        /// The computer's USB controller put the programmer in suspend mode, so
        /// programming was aborted.
        /// </summary>
        /// <remarks>Very unlikely.</remarks>
        UsbSuspend = 0x05,
    }

    public enum LineIdentity : byte
    {
        None = 0x00,
        DTR = 0x04,
        RTS = 0x07,
        CD = 0x81,
        DSR = 0x86,
        RI = 0x89
    }

    ///<summary>
    ///AVRISP Freq Label    Actual Freq  sckDuration
    ///-----------------    -----------  ------------
    ///  1843               2000            0x00       Good for 16000, 20000 target
    ///   460.8             1500            0x01       Good for 8000 target
    ///   115.2              750            0x02       Okay for 8000 target, good for 4000 target
    ///    57.6              200            0x03
    ///     4.00              4.0           0x26
    ///     1.21              1.5           0xFE
    ///</summary>
    public enum SckDuration : byte
    {
        Frequency2000 = 0x00,
        Frequency1500 = 0x01,
        Frequency750 = 0x02,
        Frequency200 = 0x03,
        Frequency4 = 0x26,
        Frequency1_5 = 0xFE
    };

    /// <summary>
    /// Represents the output state of line A or line B while the SLO-scope is active.
    /// </summary>
    public enum SloscopeOutputState
    {
        /// <summary>
        /// The line is an input.
        /// </summary>
        Off = 0,

        /// <summary>
        /// The line is being driven low (0V).
        /// </summary>
        Low = 1,

        /// <summary>
        /// The line is being driven high (5V).
        /// </summary>
        High = 3,

        /// <summary>
        /// Do not change the state of the line (this is only used when
        /// setting the state, not when getting it).
        /// </summary>
        NoChange = 0xFF
    };


    public enum SloscopeState
    {
        Off = 0,
        Analog2 = 1,
        Analog1Digital1 = 2
    }
}

// Local Variables: **
// mode: java **
// c-basic-offset: 4 **
// tab-width: 4 **
// indent-tabs-mode: nil **
// end: **
