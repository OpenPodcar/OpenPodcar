/* Change Log:
 * 2011-07-20: Fixed a bug in setSubroutines that made subroutines 64-127 not work.
 */

using System;
using System.Collections.Generic;
using System.Text;
using Pololu.UsbWrapper;
using Microsoft.Win32;
using Pololu.Usc.Bytecode;

namespace Pololu.Usc
{
    /// <summary>
    /// This class represents a Maestro that is connected to the computer.
    /// </summary>
    /// <remarks>
    /// Future improvements to this class might allow it to represent
    /// an abstract Maestro and attempt to re-connect whenever the connection is
    /// lost.
    /// </remarks>
    public partial class Usc : UsbDevice, IUscSettingsHolder
    {
        /// <summary>
        /// The device interface GUID used to detect the native USB interface
        /// of the Maestro Servo Controllers in windows.
        /// </summary>
        /// <remarks>From maestro.inf.</remarks>
        public static Guid deviceInterfaceGuid = new Guid("e0fbe39f-7670-4db6-9b1a-1dfb141014a7");

        /// <summary>Pololu's USB vendor id.</summary>
        /// <value>0x1FFB</value>
        public const ushort vendorID = 0x1ffb;

        /// <summary>The Micro Maestro's product ID.</summary>
        /// <value>0x0089</value>
        public static ushort[] productIDArray = new ushort[] { 0x0089, 0x008a, 0x008b, 0x008c };

        /// <value>
        /// Maestro USB servo controller
        /// </value>
        /// <remarks>
        /// Warning: EnglishName is used to choose the registry key.  So this should
        /// never be changed unless you change the code that selects the registry key.
        /// </remarks>
        public const string englishName = "Maestro USB servo controller";

        /// <summary>
        /// Instructions are executed at 12 MHZ
        /// </summary>
        const int INSTRUCTION_FREQUENCY = 12000000;

        /// <summary>
        /// An array of strings needed to detect which bootloaders are connected.
        /// when doing firmware upgrades.
        /// </summary>
        public static string[] bootloaderDeviceInstanceIdPrefixes
        {
            get
            {
                return new string[] { "USB\\VID_1FFB&PID_0088", "USB\\VID_1FFB&PID_008D", "USB\\VID_1FFB&PID_008E", "USB\\VID_1FFB&PID_008F" };
            }
        }

        /// <summary>
        /// Maestro
        /// </summary>
        public static string shortProductName
        {
            get
            {
                return "Maestro";
            }
        }

        private static ushort exponentialSpeedToNormalSpeed(byte exponentialSpeed)
        {
            // Maximum value of normalSpeed is 31*(1<<7)=3968

            int mantissa = exponentialSpeed >> 3;
            int exponent = exponentialSpeed & 7;

            return (ushort)(mantissa * (1 << exponent));
        }

        private static byte normalSpeedToExponentialSpeed(ushort normalSpeed)
        {
            ushort mantissa = normalSpeed;
            byte exponent = 0;

            while (true)
            {
                if (mantissa < 32)
                {
                    // We have reached the correct representation.
                    return (byte)(exponent + (mantissa << 3));
                }

                if (exponent == 7)
                {
                    // The number is too big to express in this format.
                    return 0xFF;
                }

                // Try representing the number with a bigger exponent.
                exponent += 1;
                mantissa >>= 1;
            }
        }

        public static decimal positionToMicroseconds(ushort position)
        {
            return (decimal)position / 4M;
        }

        public static ushort microsecondsToPosition(decimal us)
        {
            return (ushort)(us * 4M);
        }

        /// <summary>
        /// The approximate number of microseconds represented by the servo
        /// period when PARAMETER_SERVO_PERIOD is set to this value.
        /// </summary>
        public static decimal periodToMicroseconds(ushort period, byte servos_available)
        {
            return (decimal)period * 256M * servos_available / 12M;
        }

        /// <summary>
        /// The closest value of PARAMETER_SERVO_PERIOD for a given number of us per period.
        /// </summary>
        /// <returns>Amount of time allocated to each servo, in units of 256/12.</returns>
        public static byte microsecondsToPeriod(decimal us, byte servos_avaiable)
        {
            return (byte)Math.Round(us / 256M * 12M / servos_avaiable);
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
        private static UInt16 convertBpsToSpbrg(UInt32 bps)
        {
            if (bps == 0)
            {
                return 0;
            }

            return (UInt16)((INSTRUCTION_FREQUENCY - bps / 2) / bps);
        }

        /// <summary>
        /// Converts channel number (0-5) to port mask bit number
        /// on the Micro Maestro.  Not useful on other Maestros.
        /// </summary>
        private byte channelToPort(byte channel)
        {
            if (channel <= 3)
            {
                return channel;
            }
            else if (channel < 6)
            {
                return (byte)(channel + 2);
            }
            throw new ArgumentException("Invalid channel number " + channel);
        }

        /// <summary>
        /// The number of servos on the device.  This will be 6, 12, 18, or 24.
        /// </summary>
        public readonly byte servoCount;

        ///<summary>The number of parameter bytes per servo.</summary>
        const byte servoParameterBytes = 9;

        /// <summary>
        /// Returns the parameter number for the parameter of a given servo,
        /// given the corresponding parameter number for servo 0.
        /// </summary>
        /// <param name="p">e.g. PARAMETER_SERVO0_HOME</param>
        /// <param name="servo">Channel number.</param>
        uscParameter specifyServo(uscParameter p, byte servo)
        {
            return (uscParameter)((byte)(p) + servo * servoParameterBytes);
        }

        public Usc(DeviceListItem deviceListItem) : base(deviceListItem)
        {
            // Determine the number of servos from the product id.
            switch(getProductID())
            {
                case 0x89: servoCount = 6; break;
                case 0x8A: servoCount = 12; break;
                case 0x8B: servoCount = 18; break;
                case 0x8C: servoCount = 24; break;
                default: throw new Exception("Unknown product id " + getProductID().ToString("x2") + ".");
            }
        }

        protected bool microMaestro
        {
            get
            {
                return servoCount == 6;
            }
        }

        public byte stackSize
        {
            get
            {
                if (microMaestro)
                {
                    return MicroMaestroStackSize;
                }
                else
                {
                    return MiniMaestroStackSize;
                }
            }
        }

        public byte callStackSize
        {
            get
            {
                if (microMaestro)
                {
                    return MicroMaestroCallStackSize;
                }
                else
                {
                    return MiniMaestroCallStackSize;
                }
            }
        }


        public static List<DeviceListItem> getConnectedDevices()
        {
            try
            {
                return UsbDevice.getDeviceList(Usc.deviceInterfaceGuid);
            }
            catch (NotImplementedException)
            {
                // use vendor and product instead
                return UsbDevice.getDeviceList(Usc.vendorID, Usc.productIDArray);
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
        }

        /// <summary>
        /// Erases the entire script and subroutine address table from the devices.
        /// </summary>
        public void eraseScript()
        {
            try
            {
                controlTransfer(0x40, (byte)uscRequest.REQUEST_ERASE_SCRIPT, 0, 0);
            }
            catch (Exception e)
            {
                throw new Exception("There was an error erasing the script.", e);
            }
        }

        /// <summary>
        /// Stops and resets the script, sets the program counter to the beginning of the
        /// specified subroutine.  After this function has run, the script will be paused,
        /// so you must use setScriptDone() to start it.
        /// </summary>
        /// <param name="subroutine"></param>
        public void restartScriptAtSubroutine(byte subroutine)
        {
            try
            {
                controlTransfer(0x40, (byte)uscRequest.REQUEST_RESTART_SCRIPT_AT_SUBROUTINE, 0, subroutine);
            }
            catch (Exception e)
            {
                throw new Exception("There was an error restarting the script at subroutine " + subroutine + ".", e);
            }
        }

        public void restartScriptAtSubroutineWithParameter(byte subroutine, short parameter)
        {
            try
            {
                controlTransfer(0x40, (byte)uscRequest.REQUEST_RESTART_SCRIPT_AT_SUBROUTINE_WITH_PARAMETER, (ushort)parameter, subroutine);
            }
            catch (Exception e)
            {
                throw new Exception("There was an error restarting the script with a parameter at subroutine " + subroutine + ".", e);
            }
        }

        public void restartScript()
        {
            try
            {
                controlTransfer(0x40, (byte)uscRequest.REQUEST_RESTART_SCRIPT, 0, 0);
            }
            catch (Exception e)
            {
                throw new Exception("There was an error restarting the script.", e);
            }
        }

        public void writeScript(List<byte> bytecode)
        {
            ushort block;
            for (block = 0; block < (bytecode.Count + 15) / 16; block++)
            {
                // write each block in a separate request
                byte[] block_bytes = new byte[16];

                ushort j;
                for (j = 0; j < 16; j++)
                {
                    if (block * 16 + j < bytecode.Count)
                        block_bytes[j] = bytecode[block * 16 + j];
                    else
                        block_bytes[j] = (byte)0xFF; // don't change flash if it is not necessary
                }

                try
                {
                    //                    System.Console.WriteLine((block)+": "+block_bytes[0]+" "+block_bytes[1]+" "+block_bytes[2]+" "+block_bytes[3]+" "+block_bytes[4]+" "+block_bytes[5]+" "+block_bytes[6]+" "+block_bytes[7]+" "+block_bytes[8]+" "+block_bytes[9]+" "+block_bytes[10]+" "+block_bytes[11]+" "+block_bytes[12]+" "+block_bytes[13]+" "+block_bytes[14]+" "+block_bytes[15]); // XXX
                    controlTransfer(0x40, (byte)uscRequest.REQUEST_WRITE_SCRIPT, 0, block,
                                           block_bytes);
                }
                catch (Exception e)
                {
                    throw new Exception("There was an error writing script block " + block + ".", e);
                }
            }
        }

        /// <remarks>
        /// Prior to 2011-7-20, this function had a bug in it that made
        /// subroutines 64-123 not work!
        /// </remarks>
        public void setSubroutines(Dictionary<string, ushort> subroutineAddresses,
                                   Dictionary<string, byte> subroutineCommands)
        {
            byte[] subroutineData = new byte[256];

            ushort i;
            for (i = 0; i < 256; i++)
                subroutineData[i] = 0xFF; // initialize to the default flash state

            foreach (KeyValuePair<string, ushort> kvp in subroutineAddresses)
            {
                string name = kvp.Key;
                byte bytecode = subroutineCommands[name];

                if (bytecode == (byte)Opcode.CALL)
                    continue; // skip CALLs - these do not get a position in the subroutine memory

                subroutineData[2 * (bytecode - 128)] = (byte)(kvp.Value % 256);
                subroutineData[2 * (bytecode - 128) + 1] = (byte)(kvp.Value >> 8);
            }

            ushort block;
            for (block = 0; block < 16; block++)
            {
                // write each block in a separate request
                byte[] block_bytes = new byte[16];

                ushort j;
                for (j = 0; j < 16; j++)
                {
                    block_bytes[j] = subroutineData[block * 16 + j];
                }

                try
                {
                    //                    System.Console.WriteLine((block + subroutineOffsetBlocks)+": "+block_bytes[0]+" "+block_bytes[1]+" "+block_bytes[2]+" "+block_bytes[3]+" "+block_bytes[4]+" "+block_bytes[5]+" "+block_bytes[6]+" "+block_bytes[7]+" "+block_bytes[8]+" "+block_bytes[9]+" "+block_bytes[10]+" "+block_bytes[11]+" "+block_bytes[12]+" "+block_bytes[13]+" "+block_bytes[14]+" "+block_bytes[15]); // XXX
                    controlTransfer(0x40, (byte)uscRequest.REQUEST_WRITE_SCRIPT, 0,
                                           (ushort)(block + subroutineOffsetBlocks),
                                           block_bytes);
                }
                catch (Exception e)
                {
                    throw new Exception("There was an error writing subroutine block " + block + ".", e);
                }
            }
        }

        private uint subroutineOffsetBlocks
        {
            get
            {
                switch(getProductID())
                {
                case 0x89: return 64;
                case 0x8A: return 512;
                case 0x8B: return 512;
                case 0x8C: return 512;
                default: throw new Exception("unknown product ID");
                }
            }
        }

        public void setScriptDone(byte value)
        {
            try
            {
                controlTransfer(0x40, (byte)uscRequest.REQUEST_SET_SCRIPT_DONE, value, 0);
            }
            catch (Exception e)
            {
                throw new Exception("There was an error setting the script done.", e);
            }
        }

        public void startBootloader()
        {
            try
            {
                controlTransfer(0x40, (byte)uscRequest.REQUEST_START_BOOTLOADER, 0, 0);
            }
            catch (Exception e)
            {
                throw new Exception("There was an error entering bootloader mode.", e);
            }
        }

        public void reinitialize()
        {
            reinitialize(50);
        }

        private void reinitialize(int waitTime)
        {
            try
            {
                controlTransfer(0x40, (byte)uscRequest.REQUEST_REINITIALIZE, 0, 0);
            }
            catch (Exception e)
            {
                throw new Exception("There was an error re-initializing the device.", e);
            }

            System.Threading.Thread.Sleep(waitTime);
            if (!microMaestro)
            {
                // Flush out any spurious performance flags that might have occurred.
                MaestroVariables variables;
                getVariables(out variables);
            }
        }

        public void clearErrors()
        {
            try
            {
                controlTransfer(0x40, (byte)uscRequest.REQUEST_CLEAR_ERRORS, 0, 0);
            }
            catch (Exception e)
            {
                throw new Exception("There was a USB communication error while clearing the servo errors.", e);
            }
        }

        /// <summary>
        /// Gets the complete set of status information for the Maestro.
        /// </summary>
        /// <remarks>If you are using a Mini Maestro and do not need all of
        /// the data provided by this function, you can save some CPU time
        /// by using the overloads with fewer arguments.</remarks>
        public unsafe void getVariables(out MaestroVariables variables, out short[] stack, out ushort[] callStack, out ServoStatus[] servos)
        {
            if (microMaestro)
            {
                // On the Micro Maestro, this function requires just one control transfer:
                getVariablesMicroMaestro(out variables, out stack, out callStack, out servos);
            }
            else
            {
                // On the Mini Maestro, this function requires four control transfers:
                getVariablesMiniMaestro(out variables);
                getVariablesMiniMaestro(out servos);
                getVariablesMiniMaestro(out stack);
                getVariablesMiniMaestro(out callStack);
            }
        }

        /// <summary>
        /// Gets a MaestroVariables struct representing the current status
        /// of the device.
        /// </summary>
        /// <remarks>If you are using the Micro Maestro and calling
        /// getVariables more than once in quick succession,
        /// then you can save some CPU time by just using the
        /// overload that has 4 arguments.
        /// </remarks>
        public void getVariables(out MaestroVariables variables)
        {
            if (microMaestro)
            {
                ServoStatus[] servos;
                short[] stack;
                ushort[] callStack;
                getVariablesMicroMaestro(out variables, out stack, out callStack, out servos);
            }
            else
            {
                getVariablesMiniMaestro(out variables);
            }
        }

        /// <summary>
        /// Gets an array of ServoStatus structs representing
        /// the current status of all the channels.
        /// </summary>
        /// <remarks>If you are using the Micro Maestro and calling
        /// getVariables more than once in quick succession,
        /// then you can save some CPU time by just using the
        /// overload that has 4 arguments.
        /// </remarks>
        public void getVariables(out ServoStatus[] servos)
        {
            if (microMaestro)
            {
                MaestroVariables variables;
                short[] stack;
                ushort[] callStack;
                getVariablesMicroMaestro(out variables, out stack, out callStack, out servos);
            }
            else
            {
                getVariablesMiniMaestro(out servos);
            }
        }

        /// <summary>
        /// Gets an array of shorts[] representing the current stack.
        /// The maximum size of the array is stackSize.
        /// </summary>
        /// <remarks>If you are using the Micro Maestro and calling
        /// getVariables more than once in quick succession,
        /// then you can save some CPU time by just using the
        /// overload that has 4 arguments.
        /// </remarks>
        public void getVariables(out short[] stack)
        {
            if (microMaestro)
            {
                MaestroVariables variables;
                ServoStatus[] servos;
                ushort[] callStack;
                getVariablesMicroMaestro(out variables, out stack, out callStack, out servos);
            }
            else
            {
                getVariablesMiniMaestro(out stack);
            }
        }

        /// <summary>
        /// Gets an array of ushorts[] representing the current stack.
        /// The maximum size of the array is callStackSize.
        /// </summary>
        /// <remarks>If you are using the Micro Maestro and calling
        /// getVariables more than once in quick succession,
        /// then you can save some CPU time by just using the
        /// overload that has 4 arguments.
        /// </remarks>
        public void getVariables(out ushort[] callStack)
        {
            if (microMaestro)
            {
                MaestroVariables variables;
                short[] stack;
                ServoStatus[] servos;
                getVariablesMicroMaestro(out variables, out stack, out callStack, out servos);
            }
            else
            {
                getVariablesMiniMaestro(out callStack);
            }
        }

        public const int MicroMaestroStackSize = 32;
        public const int MicroMaestroCallStackSize = 10;

        public const int MiniMaestroStackSize = 126;
        public const int MiniMaestroCallStackSize = 126;

        private unsafe void getVariablesMicroMaestro(out MaestroVariables variables, out short[] stack, out ushort[] callStack, out ServoStatus[] servos)
        {
            byte[] array = new byte[sizeof(MicroMaestroVariables) + servoCount * sizeof(ServoStatus)];

            try
            {
                controlTransfer(0xC0, (byte)uscRequest.REQUEST_GET_VARIABLES, 0, 0, array);
            }
            catch (Exception e)
            {
                throw new Exception("There was an error getting the device variables.", e);
            }

            fixed (byte* pointer = array)
            {
                // copy the variable data
                MicroMaestroVariables tmp = *(MicroMaestroVariables*)pointer;
                variables.stackPointer = tmp.stackPointer;
                variables.callStackPointer = tmp.callStackPointer;
                variables.errors = tmp.errors;
                variables.programCounter = tmp.programCounter;
                variables.scriptDone = tmp.scriptDone;
                variables.performanceFlags = 0;

                servos = new ServoStatus[servoCount];
                for (byte i = 0; i < servoCount; i++)
                {
                    servos[i] = *(ServoStatus*)(pointer + sizeof(MicroMaestroVariables) + sizeof(ServoStatus) * i);
                }

                stack = new short[variables.stackPointer];
                for(byte i = 0; i < stack.Length; i++) { stack[i] = *(tmp.stack+i); }

                callStack = new ushort[variables.callStackPointer];
                for (byte i = 0; i < callStack.Length; i++) { callStack[i] = *(tmp.callStack + i); }
            }
        }

        private unsafe void getVariablesMiniMaestro(out MaestroVariables variables)
        {
            try
            {
                // Get miscellaneous variables.
                MiniMaestroVariables tmp;
                UInt32 bytesRead = controlTransfer(0xC0, (byte)uscRequest.REQUEST_GET_VARIABLES, 0, 0, &tmp, (ushort)sizeof(MiniMaestroVariables));
                if (bytesRead != sizeof(MiniMaestroVariables))
                {
                    throw new Exception("Short read: " + bytesRead + " < " + sizeof(MiniMaestroVariables) + ".");
                }

                // Copy the variable data
                variables.stackPointer = tmp.stackPointer;
                variables.callStackPointer = tmp.callStackPointer;
                variables.errors = tmp.errors;
                variables.programCounter = tmp.programCounter;
                variables.scriptDone = tmp.scriptDone;
                variables.performanceFlags = tmp.performanceFlags;
            }
            catch (Exception e)
            {
                throw new Exception("Error getting variables from device.", e);
            }
        }

        private unsafe void getVariablesMiniMaestro(out ServoStatus[] servos)
        {
            try
            {
                byte[] servoSettingsArray = new byte[servoCount * sizeof(ServoStatus)];

                // Get the raw data from the device.
                UInt32 bytesRead = controlTransfer(0xC0, (byte)uscRequest.REQUEST_GET_SERVO_SETTINGS, 0, 0, servoSettingsArray);
                if (bytesRead != servoSettingsArray.Length)
                {
                    throw new Exception("Short read: " + bytesRead + " < " + servoSettingsArray.Length + ".");
                }

                // Put the data in to a managed array object.
                servos = new ServoStatus[servoCount];
                fixed (byte* pointer = servoSettingsArray)
                {
                    for (byte i = 0; i < servoCount; i++)
                    {
                        servos[i] = *(ServoStatus*)(pointer + sizeof(ServoStatus) * i);
                    }
                }
            }
            catch (Exception e)
            {
                throw new Exception("Error getting channel settings from device.", e);
            }
        }

        private unsafe void getVariablesMiniMaestro(out short[] stack)
        {
            try
            {
                // Get the data stack.
                stack = new short[MiniMaestroStackSize];
                fixed (short* pointer = stack)
                {
                    UInt32 bytesRead = controlTransfer(0xC0, (byte)uscRequest.REQUEST_GET_STACK, 0, 0, pointer, (ushort)(sizeof(short) * stack.Length));
                    Array.Resize<short>(ref stack, (int)(bytesRead / sizeof(short)));
                }
            }
            catch (Exception e)
            {
                throw new Exception("Error getting stack from device.", e);
            }
        }

        private unsafe void getVariablesMiniMaestro(out ushort[] callStack)
        {
            try
            {
                callStack = new ushort[MiniMaestroCallStackSize];
                fixed (ushort* pointer = callStack)
                {
                    UInt32 bytesRead = controlTransfer(0xC0, (byte)uscRequest.REQUEST_GET_CALL_STACK, 0, 0, pointer, (ushort)(sizeof(ushort) * callStack.Length));
                    Array.Resize<ushort>(ref callStack, (int)(bytesRead / sizeof(ushort)));
                }
            }
            catch (Exception e)
            {
                throw new Exception("Error getting call stack from device.", e);
            }
        }

        public void setTarget(byte servo, ushort value)
        {
            try
            {
                controlTransfer(0x40, (byte)uscRequest.REQUEST_SET_TARGET, value, servo);
            }
            catch (Exception e)
            {
                throw new Exception("Failed to set target of servo " + servo + " to " + value + ".", e);
            }
        }

        public void setSpeed(byte servo, ushort value)
        {
            try
            {
                controlTransfer(0x40, (byte)uscRequest.REQUEST_SET_SERVO_VARIABLE, value, servo);
            }
            catch (Exception e)
            {
                throw new Exception("Failed to set speed of servo " + servo + " to " + value + ".", e);
            }
        }

        public void setAcceleration(byte servo, ushort value)
        {
            // set the high bit of servo to specify acceleration
            try
            {
                controlTransfer(0x40, (byte)uscRequest.REQUEST_SET_SERVO_VARIABLE,
                                       value, (byte)(servo | 0x80));
            }
            catch (Exception e)
            {
                throw new Exception("Failed to set acceleration of servo " + servo + " to " + value + ".", e);
            }
        }

        public void setUscSettings(UscSettings settings, bool newScript)
        {
            setRawParameter(uscParameter.PARAMETER_SERIAL_MODE, (byte)settings.serialMode);
            setRawParameter(uscParameter.PARAMETER_SERIAL_FIXED_BAUD_RATE, convertBpsToSpbrg(settings.fixedBaudRate));
            setRawParameter(uscParameter.PARAMETER_SERIAL_ENABLE_CRC, (ushort)(settings.enableCrc ? 1 : 0));
            setRawParameter(uscParameter.PARAMETER_SERIAL_NEVER_SUSPEND, (ushort)(settings.neverSuspend ? 1 : 0));
            setRawParameter(uscParameter.PARAMETER_SERIAL_DEVICE_NUMBER, settings.serialDeviceNumber);
            setRawParameter(uscParameter.PARAMETER_SERIAL_MINI_SSC_OFFSET, settings.miniSscOffset);
            setRawParameter(uscParameter.PARAMETER_SERIAL_TIMEOUT, settings.serialTimeout);
            setRawParameter(uscParameter.PARAMETER_SCRIPT_DONE, (ushort)(settings.scriptDone ? 1 : 0));

            if (servoCount == 6)
            {
                setRawParameter(uscParameter.PARAMETER_SERVOS_AVAILABLE, settings.servosAvailable);
                setRawParameter(uscParameter.PARAMETER_SERVO_PERIOD, settings.servoPeriod);
            }
            else
            {
                setRawParameter(uscParameter.PARAMETER_MINI_MAESTRO_SERVO_PERIOD_L, (byte)(settings.miniMaestroServoPeriod & 0xFF));
                setRawParameter(uscParameter.PARAMETER_MINI_MAESTRO_SERVO_PERIOD_HU, (ushort)(settings.miniMaestroServoPeriod >> 8));

                byte multiplier;
                if (settings.servoMultiplier < 1)
                {
                    multiplier = 0;
                }
                else if (settings.servoMultiplier > 256)
                {
                    multiplier = 255;
                }
                else
                {
                    multiplier = (byte)(settings.servoMultiplier - 1);
                }
                setRawParameter(uscParameter.PARAMETER_SERVO_MULTIPLIER, multiplier);
            }

            if (servoCount > 18)
            {
                setRawParameter(uscParameter.PARAMETER_ENABLE_PULLUPS, (ushort)(settings.enablePullups ? 1 : 0));
            }

            RegistryKey key = openRegistryKey();

            byte ioMask = 0;
            byte outputMask = 0;
            byte[] channelModeBytes = new byte[6]{0,0,0,0,0,0};

            for (byte i = 0; i < servoCount; i++)
            {
                ChannelSetting setting = settings.channelSettings[i];

                key.SetValue("servoName" + i.ToString("d2"), setting.name, RegistryValueKind.String);

                if (microMaestro)
                {
                    if (setting.mode == ChannelMode.Input || setting.mode == ChannelMode.Output)
                    {
                        ioMask |= (byte)(1 << channelToPort(i));
                    }
                    
                    if (setting.mode == ChannelMode.Output)
                    {
                        outputMask |= (byte)(1 << channelToPort(i));
                    }
                }
                else
                {
                    channelModeBytes[i >> 2] |= (byte)((byte)setting.mode << ((i & 3) << 1));
                }

                // Make sure that HomeMode is "Ignore" for inputs.  This is also done in
                // fixUscSettings.
                HomeMode correctedHomeMode = setting.homeMode;
                if (setting.mode == ChannelMode.Input)
                {
                    correctedHomeMode = HomeMode.Ignore;
                }

                // Compute the raw value of the "home" parameter.
                ushort home;
                if (correctedHomeMode == HomeMode.Off) home = 0;
                else if (correctedHomeMode == HomeMode.Ignore) home = 1;
                else home = setting.home;
                setRawParameter(specifyServo(uscParameter.PARAMETER_SERVO0_HOME, i), home);

                setRawParameter(specifyServo(uscParameter.PARAMETER_SERVO0_MIN, i), (ushort)(setting.minimum / 64));
                setRawParameter(specifyServo(uscParameter.PARAMETER_SERVO0_MAX, i), (ushort)(setting.maximum / 64));
                setRawParameter(specifyServo(uscParameter.PARAMETER_SERVO0_NEUTRAL, i), setting.neutral);
                setRawParameter(specifyServo(uscParameter.PARAMETER_SERVO0_RANGE, i), (ushort)(setting.range / 127));
                setRawParameter(specifyServo(uscParameter.PARAMETER_SERVO0_SPEED, i), normalSpeedToExponentialSpeed(setting.speed));
                setRawParameter(specifyServo(uscParameter.PARAMETER_SERVO0_ACCELERATION, i), setting.acceleration);
            }

            if (microMaestro)
            {
                setRawParameter(uscParameter.PARAMETER_IO_MASK_C, ioMask);
                setRawParameter(uscParameter.PARAMETER_OUTPUT_MASK_C, outputMask);
            }
            else
            {
                for (byte i = 0; i < 6; i++)
                {
                    setRawParameter(uscParameter.PARAMETER_CHANNEL_MODES_0_3 + i, channelModeBytes[i]);
                }
            }

            if (newScript)
            {
                setScriptDone(1); // stop the script

                // load the new script
                BytecodeProgram program = settings.bytecodeProgram;
                List<byte> byteList = program.getByteList();
                if (byteList.Count > maxScriptLength)
                {
                    throw new Exception("Script too long for device (" + byteList.Count + " bytes)");
                }
                if (byteList.Count < maxScriptLength)
                {
                    // if possible, add QUIT to the end to prevent mysterious problems with
                    // unterminated scripts
                    byteList.Add((byte)Opcode.QUIT);
                }
                eraseScript();
                setSubroutines(program.subroutineAddresses, program.subroutineCommands);
                writeScript(byteList);
                setRawParameter(uscParameter.PARAMETER_SCRIPT_CRC, program.getCRC());

                // Save the script in the registry
                key.SetValue("script", settings.script, RegistryValueKind.String);
            }

            Sequencer.Sequence.saveSequencesInRegistry(settings.sequences, key);

            key.Close(); // This might be needed to flush the changes.
        }

        /// <summary>
        /// Tries to open the registry key that holds the information for this device.
        /// If the key does not exist, creates it.  Returns the key.
        /// </summary>
        /// <returns></returns>
        private RegistryKey openRegistryKey()
        {
            string keyname = "Software\\Pololu\\" + englishName + "\\" + getSerialNumber();
            RegistryKey key = Registry.CurrentUser.OpenSubKey(keyname, true);
            if (key == null)
            {
                key = Registry.CurrentUser.CreateSubKey(keyname);
            }
            return key;
        }

        private void setRawParameter(uscParameter parameter, ushort value)
        {
            Range range = Usc.getRange(parameter);
            requireArgumentRange(value, range.minimumValue, range.maximumValue, parameter.ToString());
            int bytes = range.bytes;
            setRawParameterNoChecks((ushort)parameter, value, bytes);
        }

        /// <summary>
        /// Sets the parameter without checking the range or bytes
        /// </summary>
        /// <param name="parameter"></param>
        /// <param name="value"></param>
        /// <param name="bytes"></param>
        private void setRawParameterNoChecks(ushort parameter, ushort value, int bytes)
        {
            ushort index = (ushort)((bytes << 8) + parameter); // high bytes = # of bytes
            try
            {
                controlTransfer(0x40, (byte)uscRequest.REQUEST_SET_PARAMETER, value, index);
            }
            catch (Exception e)
            {
                throw new Exception("There was an error setting parameter " + parameter.ToString() + " on the device.", e);
            }
        }

        private unsafe ushort getRawParameter(uscParameter parameter)
        {
            Range range = Usc.getRange(parameter);
            ushort value = 0;
            byte[] array = new byte[range.bytes];
            try
            {
                controlTransfer(0xC0, (byte)uscRequest.REQUEST_GET_PARAMETER, 0, (ushort)parameter, array);
            }
            catch (Exception e)
            {
                throw new Exception("There was an error getting parameter " + parameter.ToString() + " from the device.", e);
            }
            if (range.bytes == 1)
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
        /// Gets a settings object, pulling some info from the registry and some from the device.
        /// If there is an inconsistency, a special flag is set.
        /// </summary>
        /// <returns></returns>
        public UscSettings getUscSettings()
        {
            var settings = new UscSettings();

            settings.serialMode = (uscSerialMode)getRawParameter(uscParameter.PARAMETER_SERIAL_MODE);
            settings.fixedBaudRate = convertSpbrgToBps(getRawParameter(uscParameter.PARAMETER_SERIAL_FIXED_BAUD_RATE));
            settings.enableCrc = getRawParameter(uscParameter.PARAMETER_SERIAL_ENABLE_CRC) != 0;
            settings.neverSuspend = getRawParameter(uscParameter.PARAMETER_SERIAL_NEVER_SUSPEND) != 0;
            settings.serialDeviceNumber = (byte)getRawParameter(uscParameter.PARAMETER_SERIAL_DEVICE_NUMBER);
            settings.miniSscOffset = (byte)getRawParameter(uscParameter.PARAMETER_SERIAL_MINI_SSC_OFFSET);
            settings.serialTimeout = getRawParameter(uscParameter.PARAMETER_SERIAL_TIMEOUT);
            settings.scriptDone = getRawParameter(uscParameter.PARAMETER_SCRIPT_DONE) != 0;

            if (servoCount == 6)
            {
                settings.servosAvailable = (byte)getRawParameter(uscParameter.PARAMETER_SERVOS_AVAILABLE);
                settings.servoPeriod = (byte)getRawParameter(uscParameter.PARAMETER_SERVO_PERIOD);
            }
            else
            {
                UInt32 tmp = (UInt32)(getRawParameter(uscParameter.PARAMETER_MINI_MAESTRO_SERVO_PERIOD_HU) << 8);
                tmp |= (byte)getRawParameter(uscParameter.PARAMETER_MINI_MAESTRO_SERVO_PERIOD_L);
                settings.miniMaestroServoPeriod = tmp;

                settings.servoMultiplier = (ushort)(getRawParameter(uscParameter.PARAMETER_SERVO_MULTIPLIER) + 1);
            }

            if (servoCount > 18)
            {
                settings.enablePullups = getRawParameter(uscParameter.PARAMETER_ENABLE_PULLUPS) != 0;
            }

            byte ioMask = 0;
            byte outputMask = 0;
            byte[] channelModeBytes = new Byte[6];

            if (microMaestro)
            {
                ioMask = (byte)getRawParameter(uscParameter.PARAMETER_IO_MASK_C);
                outputMask = (byte)getRawParameter(uscParameter.PARAMETER_OUTPUT_MASK_C);
            }
            else
            {
                for (byte i = 0; i < 6; i++)
                {
                    channelModeBytes[i] = (byte)getRawParameter(uscParameter.PARAMETER_CHANNEL_MODES_0_3 + i);
                }
            }

            for (byte i = 0; i < servoCount; i++)
            {
                // Initialize the ChannelSettings objects and 
                // set all parameters except name and mode.
                ChannelSetting setting = new ChannelSetting();

                if (microMaestro)
                {
                    byte bitmask = (byte)(1 << channelToPort(i));
                    if ((ioMask & bitmask) == 0)
                    {
                        setting.mode = ChannelMode.Servo;
                    }
                    else if ((outputMask & bitmask) == 0)
                    {
                        setting.mode = ChannelMode.Input;
                    }
                    else
                    {
                        setting.mode = ChannelMode.Output;
                    }
                }
                else
                {
                    setting.mode = (ChannelMode)((channelModeBytes[i >> 2] >> ((i & 3)<<1)) & 3);
                }

                ushort home = getRawParameter(specifyServo(uscParameter.PARAMETER_SERVO0_HOME, i));
                if (home == 0)
                {
                    setting.homeMode = HomeMode.Off;
                    setting.home = 0;
                }
                else if (home == 1)
                {
                    setting.homeMode = HomeMode.Ignore;
                    setting.home = 0;
                }
                else
                {
                    setting.homeMode = HomeMode.Goto;
                    setting.home = home;
                }

                setting.minimum = (ushort)(64 * getRawParameter(specifyServo(uscParameter.PARAMETER_SERVO0_MIN, i)));
                setting.maximum = (ushort)(64 * getRawParameter(specifyServo(uscParameter.PARAMETER_SERVO0_MAX, i)));
                setting.neutral = getRawParameter(specifyServo(uscParameter.PARAMETER_SERVO0_NEUTRAL, i));
                setting.range = (ushort)(127 * getRawParameter(specifyServo(uscParameter.PARAMETER_SERVO0_RANGE, i)));
                setting.speed = exponentialSpeedToNormalSpeed((byte)getRawParameter(specifyServo(uscParameter.PARAMETER_SERVO0_SPEED, i)));
                setting.acceleration = (byte)getRawParameter(specifyServo(uscParameter.PARAMETER_SERVO0_ACCELERATION, i));

                settings.channelSettings.Add(setting);
            }

            RegistryKey key = openRegistryKey();
            if (key != null)
            {
                // Get names for servos from the registry.
                for (byte i = 0; i < servoCount; i++)
                {
                    settings.channelSettings[i].name = (string)key.GetValue("servoName" + i.ToString("d2"), "");
                }

                // Get the script from the registry
                string script = (string)key.GetValue("script");
                if (script == null)
                    script = "";
                try
                {
                    // compile it to get the checksum
                    settings.setAndCompileScript(script);

                    BytecodeProgram program = settings.bytecodeProgram;
                    if (program.getByteList().Count > this.maxScriptLength)
                    {
                        throw new Exception();
                    }
                    if (program.getCRC() != (ushort)getRawParameter(uscParameter.PARAMETER_SCRIPT_CRC))
                    {
                        throw new Exception();
                    }
                }
                catch (Exception)
                {
                    // no script found or error compiling - leave script at ""
                    settings.scriptInconsistent = true;
                }

                // Get the sequences from the registry.
                settings.sequences = Sequencer.Sequence.readSequencesFromRegistry(key, servoCount);
            }

            return settings;
        }

        public ushort maxScriptLength
        {
            get
            {
                if (microMaestro)
                {
                    return 1024;
                }
                else
                {
                    return 8192;
                }
            }
        }

        private static void requireArgumentRange(uint argumentValue, Int32 minimum, Int32 maximum, String argumentName)
        {
            if (argumentValue < minimum || argumentValue > maximum)
            {
                throw new ArgumentException("The " + argumentName + " must be between " + minimum +
                    " and " + maximum + " but the value given was " + argumentValue);
            }
        }

        public void restoreDefaultConfiguration()
        {
            setRawParameterNoChecks((byte)uscParameter.PARAMETER_INITIALIZED, (ushort)0xFF, 1);
            reinitialize(1500);
        }

        public void fixSettings(UscSettings settings, List<string> warnings)
        {
            // Discard extra channels if needed.
            if (settings.servoCount > this.servoCount)
            {
                warnings.Add("The settings loaded include settings for " + settings.servoCount + " channels, but this device has only " + this.servoCount + " channels.  The extra channel settings will be ignored.");
                settings.channelSettings.RemoveRange(this.servoCount, settings.servoCount - this.servoCount);
            }

            // Add channels if needed.
            if (settings.servoCount < this.servoCount)
            {
                warnings.Add("The settings loaded include settings for only " + settings.servoCount + " channels, but this device has " + this.servoCount + " channels.  The other channels will be initialized with default settings.");
                while(settings.servoCount < this.servoCount)
                {
                    ChannelSetting cs = new ChannelSetting();
                    if (this.microMaestro && settings.servosAvailable <= settings.servoCount)
                    {
                        cs.mode = ChannelMode.Input;
                    }
                    settings.channelSettings.Add(cs);
                }
            }

            // Prevent users from experiencing the bug with Ignore mode in Micro Maestro v1.00.
            if (settings.servoCount == 6 && firmwareVersionMajor <= 1 && firmwareVersionMinor == 0)
            {
                bool servoIgnoreWarningShown = false;

                foreach (ChannelSetting setting in settings.channelSettings)
                {
                    if ((setting.mode == ChannelMode.Servo) && setting.homeMode == HomeMode.Ignore)
                    {
                        setting.homeMode = HomeMode.Off;

                        if (!servoIgnoreWarningShown)
                        {
                            warnings.Add("Ignore mode does not work for servo channels on the Micro Maestro 6-Channel Servo Controller firmware versions prior to 1.01.\nYour channels will be changed to Off mode.\nVisit Pololu.com for a firmware upgrade.");
                            servoIgnoreWarningShown = true;
                        }
                    }
                }
            }

            // Set homeMode to ignore for inputs (silently, because it's not the user's fault).
            foreach (ChannelSetting cs in settings.channelSettings)
            {
                switch (cs.mode)
                {
                    case ChannelMode.Input:
                    {
                        cs.homeMode = HomeMode.Ignore;
                        cs.minimum = 0;
                        cs.maximum = 1024; // Should probably be 1023, but this is the tradition from the Micro Maestros.
                        cs.speed = 0;
                        cs.acceleration = 0;
                        cs.neutral = 1024;
                        cs.range = 1905;
                        break;
                    }
                    case ChannelMode.Output:
                    {
                        cs.minimum = 3986;
                        cs.maximum = 8000;
                        cs.speed = 0;
                        cs.acceleration = 0;
                        cs.neutral = 6000;
                        cs.range = 1905;
                        break;
                    }
                }
            }

            if (settings.serialDeviceNumber >= 128)
            {
                settings.serialDeviceNumber = 12;
                warnings.Add("The serial device number must be less than 128.  It will be changed to 12.");
            }
        }

        protected static Range getRange(uscParameter parameterId)
        {
            if (parameterId == uscParameter.PARAMETER_INITIALIZED)
                return Range.u8;

            switch (parameterId)
            {
                case uscParameter.PARAMETER_SERVOS_AVAILABLE:
                    return Range.u8;
                case uscParameter.PARAMETER_SERVO_PERIOD:
                    return Range.u8;
                case uscParameter.PARAMETER_MINI_MAESTRO_SERVO_PERIOD_L:
                    return Range.u8;
                case uscParameter.PARAMETER_MINI_MAESTRO_SERVO_PERIOD_HU:
                    return Range.u16;
                case uscParameter.PARAMETER_SERVO_MULTIPLIER:
                    return Range.u8;
                case uscParameter.PARAMETER_CHANNEL_MODES_0_3:
                case uscParameter.PARAMETER_CHANNEL_MODES_4_7:
                case uscParameter.PARAMETER_CHANNEL_MODES_8_11:
                case uscParameter.PARAMETER_CHANNEL_MODES_12_15:
                case uscParameter.PARAMETER_CHANNEL_MODES_16_19:
                case uscParameter.PARAMETER_CHANNEL_MODES_20_23:
                case uscParameter.PARAMETER_ENABLE_PULLUPS:
                    return Range.u8;
                case uscParameter.PARAMETER_SERIAL_MODE:
                    return new Range(1, 0, 3);
                case uscParameter.PARAMETER_SERIAL_BAUD_DETECT_TYPE:
                    return new Range(1, 0, 1);
                case uscParameter.PARAMETER_SERIAL_NEVER_SUSPEND:
                    return Range.boolean;
                case uscParameter.PARAMETER_SERIAL_TIMEOUT:
                    return Range.u16;
                case uscParameter.PARAMETER_SERIAL_ENABLE_CRC:
                    return Range.boolean;
                case uscParameter.PARAMETER_SERIAL_DEVICE_NUMBER:
                    return Range.u7;
                case uscParameter.PARAMETER_SERIAL_FIXED_BAUD_RATE:
                    return Range.u16;
                case uscParameter.PARAMETER_SERIAL_MINI_SSC_OFFSET:
                    return new Range(1, 0, 254);
                case uscParameter.PARAMETER_SCRIPT_CRC:
                    return Range.u16;
                case uscParameter.PARAMETER_SCRIPT_DONE:
                    return Range.boolean;
            }

            // must be one of the servo parameters
            switch ((((byte)parameterId - (byte)uscParameter.PARAMETER_SERVO0_HOME) % 9) +
                    (byte)uscParameter.PARAMETER_SERVO0_HOME)
            {
                case (byte)uscParameter.PARAMETER_SERVO0_HOME:
                case (byte)uscParameter.PARAMETER_SERVO0_NEUTRAL:
                    return new Range(2, 0, 32440); // 32640 - 200
                case (byte)uscParameter.PARAMETER_SERVO0_RANGE:
                    return new Range(1, 1, 50); // the upper limit could be adjusted
                case (byte)uscParameter.PARAMETER_SERVO0_SPEED:
                case (byte)uscParameter.PARAMETER_SERVO0_MAX:
                case (byte)uscParameter.PARAMETER_SERVO0_MIN:
                case (byte)uscParameter.PARAMETER_SERVO0_ACCELERATION:
                    return Range.u8;
            }

            throw new ArgumentException("Invalid parameterId " + parameterId.ToString() + ", can not determine the range of this parameter.");
        }

        protected struct Range
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

            internal static Range u32 = new Range(4, 0, 0x7FFFFFFF);
            internal static Range u16 = new Range(2, 0, 0xFFFF);
            internal static Range u12 = new Range(2, 0, 0x0FFF);
            internal static Range u10 = new Range(2, 0, 0x03FF);
            internal static Range u8 = new Range(1, 0, 0xFF);
            internal static Range u7 = new Range(1, 0, 0x7F);
            internal static Range boolean = new Range(1, 0, 1);
        }

        public void setPWM(ushort dutyCycle, ushort period)
        {
            controlTransfer(0x40, (byte)uscRequest.REQUEST_SET_PWM, dutyCycle, period);
        }

        public void disablePWM()
        {
            if (getProductID() == 0x008a)
                setTarget(8, 0);
            else
                setTarget(12, 0);
        }
    }
}

// Local Variables: **
// mode: java **
// c-basic-offset: 4 **
// tab-width: 4 **
// indent-tabs-mode: nil **
// end: **
