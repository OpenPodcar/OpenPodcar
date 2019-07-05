using System;
using Pololu.UsbWrapper;
using System.Collections.Generic;

namespace Pololu.Jrk
{
    public partial class Jrk : UsbDevice, IJrkParameterHolder
    {
        public static Guid deviceInterfaceGuid = new Guid("098da609-8fe1-410b-9fab-4d5df167d8a1");

        public static String productName = "Pololu Jrk";
        public static String shortProductName = "Jrk";
        
        /// <summary>
        /// The first part of the device instance ID for the bootloader for the Jrk 21v3.
        /// </summary>
        public static String bootloaderDeviceInstanceIdPrefix82 = "USB\\VID_1FFB&PID_0082";

        /// <summary>
        /// The first part of the device instance ID for the bootloader for the next Jrk.
        /// </summary>
        public static String bootloaderDeviceInstanceIdPrefix84 = "USB\\VID_1FFB&PID_0084";

        /// <summary>
        /// Instructions are executed at 12 MHZ
        /// </summary>
        const int INSTRUCTION_FREQUENCY = 12000000;

        public Jrk(DeviceListItem deviceListItem)
            : base(deviceListItem)
        {
            switch (getProductID())
            {
                case 0x0083:
                    product = jrkProduct.umc01a;
                    break;
                case 0x0085:
                    product = jrkProduct.umc02a;
                    break;
                default:
                    product = jrkProduct.UNKNOWN;
                    break;
            }
        }

        /// <summary>
        /// The different boards.
        /// </summary>
        public enum jrkProduct
        {
            /// <summary>
            /// 21v3
            /// </summary>
            umc01a,
            /// <summary>
            /// 12v12
            /// </summary>
            umc02a,
            /// <summary>
            /// unknown product
            /// </summary>
            UNKNOWN
        }

        /// <summary>
        /// the model of this jrk
        /// </summary>
        public readonly jrkProduct product = jrkProduct.UNKNOWN;

        /// <summary>
        /// True if to get a current reading we need to divide the current variable by the duty cycle.
        /// </summary>
        public bool divideCurrent
        {
            get
            {
                return product == jrkProduct.umc01a;
            }
        }

        /// <summary>
        /// See Sec 16.3 of the PIC18F14K50 datasheet for information about SPBRG.
        /// On the umc01a, we have SYNC=0, BRG16=1, and BRGH=1, so the pure math
        /// formula for the baud rate is Baud = INSTRUCTION_FREQUENCY / (spbrg+1);
        /// </summary>
        private static UInt32 convertSpbrgToBps(UInt16 spbrg)
        {
            if (spbrg == 0)
            {
                return 0;
            }

            return (UInt32)((INSTRUCTION_FREQUENCY + (spbrg + 1) / 2) / (spbrg + 1));
        }

        /// <summary>
        /// The converts from bps to SPBRG, so it is the opposite of convertSpbrgToBps.
        /// The purse math formula is spbrg = INSTRUCTION_FREQUENCY/Baud - 1.
        /// </summary>
        /// <param name="value"></param>
        /// <returns></returns>
        private static UInt16 convertBpsToSpbrg(UInt32 bps)
        {
            if (bps == 0)
            {
                return 0;
            }

            return (UInt16)((INSTRUCTION_FREQUENCY - bps/2) / bps);
        }

        internal static void testBaudRateConversions()
        {
            // These numbers are taken from Table 16-5, with SYNC=0, BRG16=1, BRGH=1, FOSC=48MHZ.
            assertBaudConversion(39999, 300);
            assertBaudConversion(9999, 1200);
            assertBaudConversion(4999, 2400);
            assertBaudConversion(1249, 9600);
            assertBaudConversion(1151, 10417);
            assertBaudConversion(624, 19200);
            assertBaudConversion(207, 57600);
            assertBaudConversion(103, 115200);
            
            // 184 is the minimum value the user can enter for the baud rate
            assertBaudConversion(65216, 184);

            // 115385 is the maximum value the user can enter for the baud rate
            assertBaudConversion(103, 115385);
        }

        private static void assertBaudConversion(UInt16 spbrg, UInt32 bps)
        {
            // If the user types this baud rate in, we want SPBRG to be correct.
            assertEqual(spbrg, convertBpsToSpbrg(bps), bps + " bps.");

            // If the user then loads the baud rate in to the configuration utility,
            // he will see a different BPS value but that is okay, as long as SPBRG
            // remains the same when he clicks Apply.
            UInt32 modifiedBps = convertSpbrgToBps(spbrg);
            assertEqual(spbrg, convertBpsToSpbrg(modifiedBps), modifiedBps + " bps should produce the same spbrg as " + bps + " bps.");
        }

        private static void assertEqual(Int32 expected, Int32 actual, string description)
        {
            if (expected != actual)
            {
                throw new Exception(description + "  Expected " + expected + " but got " + actual + ".");
            }
        }

        public void setTarget(UInt16 target)
        {
            requireArgumentRange(target, 0, 4095, "target");

            try
            {
                controlTransfer(0x40, (byte)jrkRequest.REQUEST_SET_TARGET, target, 0);
            }
            catch (Exception exception)
            {
                throw new Exception("There was an error setting the target.", exception);
            }
        }

        public void motorOff()
        {
            try
            {
                controlTransfer(0x40, (byte)jrkRequest.REQUEST_MOTOR_OFF, 0, 0);
            }
            catch (Exception exception)
            {
                throw new Exception("There was an error turning off the motor.", exception);
            }
        }

        /// <summary>
        /// Gets the jrk parameter.  The only one that is modified by this function relative to
        /// what is actually on the device is the serial baud rate.
        /// </summary>
        /// <param name="parameterId"></param>
        /// <returns></returns>
        public uint getJrkParameter(jrkParameter parameterId)
        {
            uint value = getParameter(parameterId);
            if (parameterId == jrkParameter.PARAMETER_SERIAL_FIXED_BAUD_RATE)
            {
                return convertSpbrgToBps((ushort)value);
            }
            
            return value;
        }

        private unsafe uint getParameter(jrkParameter parameterId)
        {
            Range range = parameterId.range();

            try
            {
                return getRequest(jrkRequest.REQUEST_GET_PARAMETER, (UInt16)parameterId, range.bytes);
            }
            catch (Exception exception)
            {
                throw new Exception("There was an error reading " + parameterId.ToString() + " from the device.", exception);
            }                    
        }

        /// <summary>
        /// Clears the error flag bits so that the motor can run again.
        /// Does not clear the "waiting for command" bit.
        /// </summary>
        public void clearErrors()
        {
            try
            {
                controlTransfer(0x40, (byte)jrkRequest.REQUEST_CLEAR_ERRORS, 0, 0);
            }
            catch (Exception exception)
            {
                throw new Exception("There was an error clearing the device's error bits.", exception);
            }
        }

        /// <summary>
        /// sets the parameter on the device.  The only parameter modified before setting is the
        /// serial fixed baud rate, which is converted to the PIC's SPBRG setting.
        /// </summary>
        /// <param name="parameterId"></param>
        /// <param name="value"></param>
        public void setJrkParameter(jrkParameter parameterId, uint value)
        {
            if (parameterId == jrkParameter.PARAMETER_SERIAL_FIXED_BAUD_RATE)
            {
                value = convertBpsToSpbrg(value);
            }
            setParameter(parameterId, value);
        }

        private void setParameter(jrkParameter parameterId, uint value)
        {
            Range range = parameterId.range();
            requireArgumentRange(value, range.minimumValue, range.maximumValue, parameterId.ToString());

            try
            {
                if (range.bytes == 1)
                {
                    setRequestU8(jrkRequest.REQUEST_SET_PARAMETER, (Byte)parameterId, (Byte)value);
                }
                else if (range.bytes == 2)
                {
                    setRequestU16(jrkRequest.REQUEST_SET_PARAMETER, (Byte)parameterId, (UInt16)value);
                }
                else
                {
                    throw new NotImplementedException("Byte count " + range.bytes + " is not implemented yet.");
                }
            }
            catch (Exception exception)
            {
                throw new Exception("There was an error setting " + parameterId.ToString() + ".", exception);
            }
        }

        private void setRequestU8(jrkRequest requestId, Byte id, Byte value)
        {
            controlTransfer(0x40, (byte)requestId, value, (UInt16)(id + (1 << 8)));
        }

        private void setRequestU16(jrkRequest requestId, Byte id, UInt16 value)
        {
            controlTransfer(0x40, (byte)requestId, value, (UInt16)(id + (2 << 8)));
        }

        public void startBootloader()
        {
            try
            {
                controlTransfer(0x40, (byte)jrkRequest.REQUEST_START_BOOTLOADER, 0, 0);
            }
            catch (Exception exception)
            {
                throw new Exception("There was an error starting the bootloader.  The firmware on this device may be corrupted.  Please see Pololu.com for instructions on how to upgrade your device even when its firmware is corrupted.", exception);
            }
        }

        /// <summary>
        /// Reinitializes the motor controller system.
        /// </summary>
        public void reinitialize()
        {
            try
            {
                controlTransfer(0x40, (byte)jrkRequest.REQUEST_REINITIALIZE, 0, 0);
            }
            catch (Exception exception)
            {
                throw new Exception("There was an error re-initializing the device.", exception);
            }
        }

        private static void requireArgumentRange(uint argumentValue, Int32 minimum, Int32 maximum, String argumentName)
        {
            if (argumentValue < minimum || argumentValue > maximum)
            {
                throw new ArgumentException("The " + argumentName + " must be between " + minimum +
                    " and " + maximum + ", but the value given was " + argumentValue);
            }
        }

        private UInt16 getRequest(jrkRequest requestId, UInt16 id, Byte length)
        {
            UInt16 value = 0;

            byte[] value_array;

            if (length == 2)
            {
                value_array = new byte[2];
                controlTransfer(0xC0, (Byte)requestId, 0, (ushort)id, value_array);
                value = (ushort)(value_array[0] + 256 * value_array[1]);
            }
            else if (length == 1)
            {
                value_array = new byte[1];
                controlTransfer(0xC0, (Byte)requestId, 0, (ushort)id, value_array);
                value = value_array[0];
            }
            else
            {
                throw new ArgumentException("length must be 1 or 2", "length");
            }

            return value;
        }

        public unsafe jrkVariables getVariables()
        {
            byte[] array = new byte[sizeof(jrkVariables)];
            uint lengthTransferred = controlTransfer(0xC0, (Byte)jrkRequest.REQUEST_GET_VARIABLES, 0, 0, array);

            if (lengthTransferred != sizeof(jrkVariables))
            {
                throw new Exception("Error getting variables from Jrk.  Expected " + sizeof(jrkVariables) + " bytes, received " + lengthTransferred + ".");
            }

            jrkVariables variables;
            fixed (byte* pointer = array)
            {
                variables = *(jrkVariables*)pointer;
            }

            return variables;
        }

        UInt16 privateFirmwareVersionMajor = 0xFFFF;
        Byte privateFirmwareVersionMinor = 0xFF;

        public UInt16 firmwareVersionMajor
        {
            get
            {
                if (privateFirmwareVersionMajor == 0xFFFF)
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
                if (privateFirmwareVersionMajor == 0xFFFF)
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
                if (privateFirmwareVersionMajor == 0xFFFF)
                {
                    getFirmwareVersion();
                }
                return privateFirmwareVersionMajor.ToString() + "." + privateFirmwareVersionMinor.ToString();
            }
        }

        void getFirmwareVersion()
        {
            Byte[] buffer = new Byte[14];
            
            try
            {
                controlTransfer(0x80, 6, 0x0100, 0x0000, buffer);
            }
            catch(Exception exception)
            {
                throw new Exception("There was an error getting the firmware version from the device.", exception);
            }

            privateFirmwareVersionMinor = (Byte)((buffer[12] & 0xF));
            privateFirmwareVersionMajor = (UInt16)((buffer[12] >> 4 & 0xF) + (buffer[13] & 0xF)*10 + (buffer[13] >> 4 & 0xF)*100);
        }

        public static List<DeviceListItem> getConnectedDevices()
        {
            try
            {
                return UsbDevice.getDeviceList(Jrk.deviceInterfaceGuid);
            }
            catch(NotImplementedException)
            {
                return UsbDevice.getDeviceList(0x1ffb, new ushort[] {0x0083,0x0085});
            }
        }
    }

    public struct Range
    {
        public Byte bytes;
        public Int32 minimumValue;
        public Int32 maximumValue;

        internal Range(Byte bytes, Int32 minimumValue, Int32 maximumValue)
        {
            this.bytes = bytes;
            this.minimumValue = minimumValue;
            this.maximumValue = maximumValue;
        }

        public Boolean signed
        {
            get
            {
                return minimumValue < 0;
            }
        }

        internal static Range u16 = new Range(2, 0, 0xFFFF);
        internal static Range u12 = new Range(2, 0, 0x0FFF);
        internal static Range u10 = new Range(2, 0, 0x03FF);
        internal static Range u8 = new Range(1, 0, 0xFF);
        internal static Range boolean = new Range(1, 0, 1);
    }

    public static class JrkExtensionMethods
    {
        public static Range range(this jrkParameter parameterId)
        {
            switch (parameterId)
            {
                default:
                    throw new ArgumentException("Invalid parameterId " + parameterId.ToString() + ", can not determine the range of this parameter.");

                case jrkParameter.PARAMETER_INITIALIZED: return new Range(1, 0, 255);
                case jrkParameter.PARAMETER_INPUT_MODE: return new Range(1,0,2); // enum
                case jrkParameter.PARAMETER_INPUT_MINIMUM: return Range.u12;
                case jrkParameter.PARAMETER_INPUT_MAXIMUM: return Range.u12;
                case jrkParameter.PARAMETER_OUTPUT_MINIMUM: return Range.u12;
                case jrkParameter.PARAMETER_OUTPUT_NEUTRAL: return Range.u12;
                case jrkParameter.PARAMETER_OUTPUT_MAXIMUM: return Range.u12;
                case jrkParameter.PARAMETER_INPUT_INVERT: return Range.boolean;
                case jrkParameter.PARAMETER_INPUT_SCALING_DEGREE: return Range.u8;  // But anything over 4 is crazy
                case jrkParameter.PARAMETER_INPUT_POWER_WITH_AUX: return Range.boolean;
                case jrkParameter.PARAMETER_FEEDBACK_DEAD_ZONE: return Range.u8;
                case jrkParameter.PARAMETER_INPUT_ANALOG_SAMPLES_EXPONENT: return new Range(1,0,8);
                case jrkParameter.PARAMETER_INPUT_DISCONNECT_MAXIMUM: return Range.u12;
                case jrkParameter.PARAMETER_INPUT_DISCONNECT_MINIMUM: return Range.u12;
                case jrkParameter.PARAMETER_INPUT_NEUTRAL_MAXIMUM: return Range.u12;
                case jrkParameter.PARAMETER_INPUT_NEUTRAL_MINIMUM: return Range.u12;

                case jrkParameter.PARAMETER_SERIAL_MODE: return new Range(1, 0, 3);
                case jrkParameter.PARAMETER_SERIAL_FIXED_BAUD_RATE: return Range.u16;
                case jrkParameter.PARAMETER_SERIAL_TIMEOUT: return Range.u16;
                case jrkParameter.PARAMETER_SERIAL_ENABLE_CRC: return Range.boolean;
                case jrkParameter.PARAMETER_SERIAL_NEVER_SUSPEND: return Range.boolean;
                case jrkParameter.PARAMETER_SERIAL_DEVICE_NUMBER: return new Range(1, 0, 127);

                case jrkParameter.PARAMETER_FEEDBACK_MODE: return new Range(1,0,5);  // enum
                case jrkParameter.PARAMETER_FEEDBACK_MINIMUM: return Range.u12;
                case jrkParameter.PARAMETER_FEEDBACK_MAXIMUM: return Range.u12;
                case jrkParameter.PARAMETER_FEEDBACK_INVERT: return Range.boolean;
                case jrkParameter.PARAMETER_FEEDBACK_POWER_WITH_AUX: return Range.boolean;
                case jrkParameter.PARAMETER_FEEDBACK_DISCONNECT_MAXIMUM: return Range.u12;
                case jrkParameter.PARAMETER_FEEDBACK_DISCONNECT_MINIMUM: return Range.u12;

                case jrkParameter.PARAMETER_FEEDBACK_ANALOG_SAMPLES_EXPONENT: return new Range(1,0,8);

                case jrkParameter.PARAMETER_PROPORTIONAL_MULTIPLIER: return Range.u10;
                case jrkParameter.PARAMETER_PROPORTIONAL_EXPONENT: return new Range(1,0,15);
                case jrkParameter.PARAMETER_INTEGRAL_MULTIPLIER: return Range.u10;
                case jrkParameter.PARAMETER_INTEGRAL_EXPONENT: return new Range(1,0,15);
                case jrkParameter.PARAMETER_DERIVATIVE_MULTIPLIER: return Range.u10;
                case jrkParameter.PARAMETER_DERIVATIVE_EXPONENT: return new Range(1,0,15);

                case jrkParameter.PARAMETER_PID_PERIOD: return new Range(2,1, 8191);
                case jrkParameter.PARAMETER_PID_INTEGRAL_LIMIT: return new Range(2, 0, 32767);
                case jrkParameter.PARAMETER_PID_RESET_INTEGRAL: return Range.boolean;
                
                case jrkParameter.PARAMETER_MOTOR_PWM_FREQUENCY: return new Range(1,0,2); // enum
                case jrkParameter.PARAMETER_MOTOR_INVERT: return Range.boolean;
                case jrkParameter.PARAMETER_MOTOR_MAX_ACCELERATION_FORWARD: return new Range(2,1,600);
                case jrkParameter.PARAMETER_MOTOR_MAX_ACCELERATION_REVERSE: return new Range(2,1,600);
                case jrkParameter.PARAMETER_MOTOR_MAX_DUTY_CYCLE_FORWARD: return new Range(2,0,600);
                case jrkParameter.PARAMETER_MOTOR_MAX_DUTY_CYCLE_REVERSE: return new Range(2,0,600);
                case jrkParameter.PARAMETER_MOTOR_MAX_CURRENT_FORWARD: return new Range(1,0,255);
                case jrkParameter.PARAMETER_MOTOR_MAX_CURRENT_REVERSE: return new Range(1,0, 255);
                case jrkParameter.PARAMETER_MOTOR_CURRENT_CALIBRATION_FORWARD: return new Range(1,0,255);
                case jrkParameter.PARAMETER_MOTOR_CURRENT_CALIBRATION_REVERSE: return new Range(1, 0, 255);
                case jrkParameter.PARAMETER_MOTOR_BRAKE_DURATION_FORWARD: return new Range(1,0,255);
                case jrkParameter.PARAMETER_MOTOR_BRAKE_DURATION_REVERSE: return new Range(1,0,255);
                case jrkParameter.PARAMETER_MOTOR_COAST_WHEN_OFF: return Range.boolean;
                case jrkParameter.PARAMETER_MOTOR_MAX_DUTY_CYCLE_WHILE_FEEDBACK_OUT_OF_RANGE: return new Range(2, 1, 600);
                
                case jrkParameter.PARAMETER_ERROR_ENABLE: return Range.u16;
                case jrkParameter.PARAMETER_ERROR_LATCH: return Range.u16;
            }
        }
    }
}

// Local Variables: **
// mode: java **
// c-basic-offset: 4 **
// tab-width: 4 **
// indent-tabs-mode: nil **
// end: **
