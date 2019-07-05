using System;
using System.IO;
using System.Collections.Generic;
using System.ComponentModel;
using Pololu.UsbWrapper;

namespace Pololu.SimpleMotorController.SmcCmd
{
    class Program
    {
        /// <summary>
        /// This enumerator helps us process the command-line arguments one at
        /// a time while keeping track of which argument we are currently processing.
        /// </summary>
        static IEnumerator<String> argEnumerator;

        /// <summary>
        /// True if the user specifies the "-f" option.
        /// </summary>
        static bool forceOption = false;

        delegate void Action();
        delegate void ActionOnDevice(Smc device);

        /// <summary>
        /// This help message is printed whenever the program is run without command-line
        /// arguments or with incorrect command-line arguments.
        /// </summary>
        static string helpMessage()
        {
            return "SmcCmd: Configuration and control utility for the " + Smc.name + ".\n" +
                "Version: " + System.Reflection.Assembly.GetExecutingAssembly().GetName().Version.ToString() + "\n" +
                "Options:\n" +
                // All lines here should be at most 80 characters.  Character 80 is here........!
                " -l, --list                   list available devices\n" +
                " -d, --device SERIALNUM       (optional) select device with given serial number\n" +
                " -s, --status                 display complete device status\n" +
                "     --stop                   stop the motor\n" +
                "     --resume                 allow motor to start\n" + 
                "     --speed NUM              set motor speed (-3200 to 3200)\n" +
                "     --brake NUM              stop motor with variable braking.  32=full brake\n" +
                "     --restoredefaults        restore factory settings\n" +
                "     --configure FILE         load settings file into device\n" +
                "     --getconf FILE           read device settings and write to file\n" +
                "     --bootloader             put device in bootloader (firmware upgrade) mode\n" +
                "Options for changing motor limits until next reset:\n" +
                "     --max-speed NUM          (3200 means no limit)\n" +
                "     --max-speed-forward NUM  (3200 means no limit)\n" +
                "     --max-speed-reverse NUM  (3200 means no limit)\n" +
                "     --max-accel NUM\n" +
                "     --max-accel-forward NUM\n" +
                "     --max-accel-reverse NUM\n" +
                "     --max-decel NUM\n" +
                "     --max-decel-forward NUM\n" +
                "     --max-decel-reverse NUM\n" +
                "     --brake-dur NUM          units are ms.  rounds up to nearest 4 ms\n" +
                "     --brake-dur-forward NUM  units are ms.  rounds up to nearest 4 ms\n" +
                "     --brake-dur-reverse NUM  units are ms.  rounds up to nearest 4 ms\n";
        }

        /// <summary>
        /// This is first function that runs when the program starts.
        /// </summary>
        static void Main(string[] args)
        {
            // If no arguments are given, just show the help message.
            if (args.Length == 0)
            {
                Console.Write(helpMessage());
                Environment.Exit(2);
            }

            try
            {
                MainWithExceptions(args);
            }
            catch (Exception exception)
            {
                printExceptionAndHelp(exception);
                Environment.Exit(1);
            }
        }

        /// <summary>
        /// This funnction does all the real work.  It will
        /// throw an exception if anything goes wrong.
        /// </summary>
        static void MainWithExceptions(string[] args)
        {
            // Parse all the command-line arguments, turning them in to a list of
            // actions to be taken.  No actions on the device will occur until
            // all the arguments have been processed and validated.
            List<Action> actions = new List<Action>();
            List<ActionOnDevice> actionsOnDevice = new List<ActionOnDevice>();

            // If the user specifies a device serial number, it will be stored here.
            String specifiedSerialNumber = null;

            // Create a list object because it is easier to work with.
            List<String> argList = new List<String>(args);

            // Search for the -f option.  This must be done ahead of time so that
            // it can be any place in the command line.
            forceOption = argList.Contains("-f");

            argEnumerator = argList.GetEnumerator();
            while(argEnumerator.MoveNext())
            {
                // Read the next argument and decide what to do.
                switch (argEnumerator.Current)
                {
                    case "-f":
                        // Ignore, because we already searched for "-f".
                        break;
                    case "-l":
                    case "--list":
                        actions.Add(listDevices);
                        break;
                    case "-d":
                    case "--device":
                        if (specifiedSerialNumber != null)
                        {
                            throw new Exception("Only one -d/--device argument is alllowed.");
                        }
                        // Remove the leading # sign.  It is not standard to put it there,
                        // but if someone writes it, this program should still work.
                        specifiedSerialNumber = nextArgument().TrimStart('#');
                        break;
                    case "-s":
                    case "--status":
                        actionsOnDevice.Add(printStatus);
                        break;
                    case "--stop":
                        actionsOnDevice.Add(stop);
                        break;
                    case "--resume":
                        actionsOnDevice.Add(resume);
                        break;
                    case "--speed":
                        actionsOnDevice.Add(laterSetSpeed());
                        break;
                    case "--brake":
                        actionsOnDevice.Add(laterSetBrake());
                        break;
                    case "--restoredefaults":
                        actionsOnDevice.Add(restoreDefaults);
                        break;
                    case "--configure":
                        actionsOnDevice.Add(laterConfigure());
                        break;
                    case "--getconf":
                        actionsOnDevice.Add(laterGetConf());
                        break;
                    case "--bootloader":
                        actionsOnDevice.Add(startBootloader);
                        break;
                    case "--max-speed":
                        actionsOnDevice.Add(laterSetMotorLimit(SmcMotorLimit.MaxSpeed));
                        break;
                    case "--max-accel":
                        actionsOnDevice.Add(laterSetMotorLimit(SmcMotorLimit.MaxAcceleration));
                        break;
                    case "--max-decel":
                        actionsOnDevice.Add(laterSetMotorLimit(SmcMotorLimit.MaxDeceleration));
                        break;
                    case "--brake-dur":
                        actionsOnDevice.Add(laterSetMotorLimit(SmcMotorLimit.BrakeDuration));
                        break;
                    case "--max-speed-forward":
                        actionsOnDevice.Add(laterSetMotorLimit(SmcMotorLimit.MaxSpeed | SmcMotorLimit.ForwardOnly));
                        break;
                    case "--max-accel-forward":
                        actionsOnDevice.Add(laterSetMotorLimit(SmcMotorLimit.MaxAcceleration | SmcMotorLimit.ForwardOnly));
                        break;
                    case "--max-decel-forward":
                        actionsOnDevice.Add(laterSetMotorLimit(SmcMotorLimit.MaxDeceleration | SmcMotorLimit.ForwardOnly));
                        break;
                    case "--brake-dur-forward":
                        actionsOnDevice.Add(laterSetMotorLimit(SmcMotorLimit.BrakeDuration | SmcMotorLimit.ForwardOnly));
                        break;
                    case "--max-speed-reverse":
                        actionsOnDevice.Add(laterSetMotorLimit(SmcMotorLimit.MaxSpeed | SmcMotorLimit.ReverseOnly));
                        break;
                    case "--max-accel-reverse":
                        actionsOnDevice.Add(laterSetMotorLimit(SmcMotorLimit.MaxAcceleration | SmcMotorLimit.ReverseOnly));
                        break;
                    case "--max-decel-reverse":
                        actionsOnDevice.Add(laterSetMotorLimit(SmcMotorLimit.MaxDeceleration | SmcMotorLimit.ReverseOnly));
                        break;
                    case "--brake-dur-reverse":
                        actionsOnDevice.Add(laterSetMotorLimit(SmcMotorLimit.BrakeDuration | SmcMotorLimit.ReverseOnly));
                        break;
                    default:
                        throw new ArgumentException("Unrecognized argument \"" + argEnumerator.Current + "\".");
                }
            }

            if (actions.Count == 0 && actionsOnDevice.Count == 0)
            {
                throw new ArgumentException("No actions specified.");
            }

            // Perform all actions that don't require being connected to a device.
            foreach (Action action in actions)
            {
                action();
            }

            if (actionsOnDevice.Count == 0)
            {
                // There are no actions that require a device, so exit successfully.
                return;
            }

            // Find the right device to connect to.
            List<DeviceListItem> list = Smc.getConnectedDevices();
            DeviceListItem item = null;

            if (specifiedSerialNumber == null)
            {
                // No serial number specified: connect to the first item in the list.
                if (list.Count > 0)
                {
                    item = list[0];
                }
            }
            else
            {
                // Find the device with the specified serial number.
                foreach (DeviceListItem checkItem in list)
                {
                    if (checkItem.serialNumber == specifiedSerialNumber)
                    {
                        item = checkItem;
                        break;
                    }
                }
            }

            if (item == null && actionsOnDevice.Count == 1 && actionsOnDevice[0] == startBootloader)
            {
                // The correct device was not found, but all the user wanted to do was enter
                // bootloader mode so we should see if the device is connected in bootloader
                // mode and report success if that is true.
                List<DeviceListItem> bootloaders = Smc.getConnectedBootloaders();

                if (specifiedSerialNumber == null)
                {
                    if (bootloaders.Count > 0)
                    {
                        item = bootloaders[0];
                    }
                }
                else
                {
                    // Find the device with the specified serial number.
                    foreach (DeviceListItem checkItem in bootloaders)
                    {
                        if (checkItem.serialNumber.Replace("-", "") == specifiedSerialNumber.Replace("-", ""))
                        {
                            item = checkItem;
                            break;
                        }
                    }
                }

                if (item == null)
                {
                    if (specifiedSerialNumber == null)
                    {
                        throw new Exception("No " + Smc.namePlural + " (or bootloaders) found.");
                    }
                    else
                    {
                        throw new Exception("Could not find a device or bootloader with serial number " + specifiedSerialNumber + ".\n" +
                            "To list devices, use the --list option.");
                    }
                }

                Console.WriteLine("The device is already in bootloader mode.");
                return;
            }

            if (item == null)
            {
                if (specifiedSerialNumber == null)
                {
                    throw new Exception("No " + Smc.namePlural + " found.");
                }
                else
                {
                    throw new Exception("Could not find a device with serial number " + specifiedSerialNumber + ".\n" +
                        "To list device, use the --list option.");
                }
            }

            // All the command-line arguments were good and a matching device was found, so
            // connect to it.
            Smc device = new Smc(item);

            // Perform all the previously computed actions on the device.
            foreach(ActionOnDevice action in actionsOnDevice)
            {
                action(device);
            }
        }

        private static ActionOnDevice laterConfigure()
        {
            // Read the entire file before connecting to the device.
            String filename = nextArgument();
            List<String> warnings = new List<string>();
            SmcSettings settings = SettingsFile.load(filename, warnings, 0);
            Smc.fixSettings(settings, warnings, 0);

            // Handle any warnings.
            if (warnings.Count != 0)
            {
                Console.WriteLine("There were problems with the settings file:");
                foreach (String warning in warnings)
                {
                    Console.WriteLine("  " + warning);
                }

                if (forceOption)
                {
                    Console.WriteLine("The -f option is selected, so the settings will be applied anyway.");
                }
                else
                {
                    throw new Exception("Aborted because of problems with settings file.\nUse the -f option to override and apply settings anyway.");
                }
            }

            // Return a delegate function that will get executed later after all
            // the command line arguments have been processed.
            return delegate(Smc device)
            {
                // This is the line that actually applies the settings to the device.
                device.setSmcSettings(settings);
            };
        }

        private static ActionOnDevice laterGetConf()
        {
            String filename = nextArgument();

            // Return a delegate function that will get executed later after all
            // the command line arguments have been processed.
            return delegate(Smc device)
            {
                // These are the lines that actually save a settings file to the disk.
                SmcSettings settings = device.getSmcSettings();
                SettingsFile.save(settings, filename, device);
            };
        }

        private static ActionOnDevice laterSetSpeed()
        {
            Int16 speed = nextArgumentAsS16();

            if (speed < -3200 || speed > 3200)
            {
                throw new ArgumentOutOfRangeException("--speed", "Speed must be between -3200 and 3200; " + speed + " is not valid.");
            }

            // Create and return a delegate function that will get executed later
            // after the command-line arguments have all been processed.
            return delegate(Smc device)
            {
                // This is the line that actually sets the speed.
                device.setSpeed(speed);
            };
        }

        private static ActionOnDevice laterSetBrake()
        {
            UInt16 brakeAmount = nextArgumentAsU16();
            if (brakeAmount > 32)
            {
                throw new ArgumentOutOfRangeException("--brake", "Brake amount must be between 0 and 32; " + brakeAmount + " is not valid.");
            }

            // Create and return a delegate function that will get executed later
            // after the command-line arguments have all been processed.
            return delegate(Smc device)
            {
                device.setBrake((Byte)brakeAmount);
            };
        }

        private static ActionOnDevice laterSetMotorLimit(SmcMotorLimit limit)
        {
            String previousArg = argEnumerator.Current;
            UInt16 value = nextArgumentAsU16();

            bool brakeDuration = (limit & (SmcMotorLimit)3) == SmcMotorLimit.BrakeDuration;

            if (!brakeDuration && value > 3200)
            {
                throw new ArgumentOutOfRangeException("Maximum value allowed for " + previousArg + " is 3200.");
            }

            bool directionSpecified = (limit & (SmcMotorLimit.ForwardOnly | SmcMotorLimit.ReverseOnly)) != 0;

            return delegate(Smc device)
            {
                // This is the line that actually sets the motor limit.
                SmcSetMotorLimitProblem problem = device.setMotorLimit(limit, value);

                bool forwardConflict = 0 != (problem & SmcSetMotorLimitProblem.ForwardConflict);
                bool reverseConflict = 0 != (problem & SmcSetMotorLimitProblem.ReverseConflict);
                if (directionSpecified)
                {
                    // The user specified a direction, so at most one of the tooDangerous booleans should be set.

                    if (forwardConflict || reverseConflict)
                    {
                        Console.Error.WriteLine("warning: specified value for " + previousArg + " conflicted with the hard limit, so the hard limit was used instead.");
                    }
                }
                else
                {
                    // The user specified no direction, so any of the tooDangerous booleans could be set.

                    if (reverseConflict)
                    {
                        Console.Error.WriteLine("warning: specified value for " + previousArg + " conflicted with the hard limit for reverse, so the hard limit for reverse was used instead.");
                    }
                    if (forwardConflict)
                    {
                        Console.Error.WriteLine("warning: specified value for " + previousArg + " conflicted with the hard limit for forward, so the hard limit for forward was used instead.");
                    }
                }
            };
        }

        public static String nextArgument()
        {
            String previousArg = argEnumerator.Current;
            argEnumerator.MoveNext();
            if (argEnumerator.Current == null)
            {
                throw new ArgumentException("Expected an argument after \"" + previousArg + "\".");
            }
            return argEnumerator.Current;
        }

        public static UInt16 nextArgumentAsU16()
        {
            String previousArg = argEnumerator.Current;
            try
            {
                return UInt16.Parse(nextArgument());
            }
            catch (Exception exception)
            {
                throw new ArgumentException("Expected a number after \"" + previousArg + "\" between 0 and 65535.", exception);
            }
        }

        public static Int16 nextArgumentAsS16()
        {
            String previousArg = argEnumerator.Current;
            try
            {
                return Int16.Parse(nextArgument());
            }
            catch (Exception exception)
            {
                throw new ArgumentException("Expected a number after \"" + previousArg + "\" between -32768 and 32767.", exception);
            }
        }

        public static void startBootloader(Smc device)
        {
            Console.WriteLine("Entering bootloader mode...");
            string serialNumber = device.getSerialNumber();
            device.startBootloader();
            device.Dispose();

            Console.WriteLine("Waiting for bootloader to connect...");
            int msElapsed = 0;
            while(true)
            {
                foreach(DeviceListItem dli in Smc.getConnectedBootloaders())
                {
                    if (dli.serialNumber.Replace("-", "") == serialNumber.Replace("-", ""))
                    {
                        Console.WriteLine("Successfully entered bootloader mode.");
                        return;
                    }
                }

                System.Threading.Thread.Sleep(20);
                msElapsed += 20;

                if (msElapsed > 8000)
                {
                    throw new Exception("Failed to enter bootloader mode: timeout elapsed.");
                }
            }
        }

        public static void stop(Smc device)
        {
            device.stop();
        }

        public static void resume(Smc device)
        {
            device.resume();
        }

        public static void restoreDefaults(Smc device)
        {
            Console.WriteLine("Restoring default settings...");
            device.resetSettings();
        }

        public static void printStatus(Smc device)
        {
            SmcVariables vars = device.getSmcVariables();

            Console.WriteLine("Model:            " + Smc.productIdToLongModelString(device.productId));
            Console.WriteLine("Serial Number:    " + device.getSerialNumber());
            Console.WriteLine("Firmware Version: " + device.getFirmwareVersionString());
            Console.WriteLine("Last Reset:       " + device.getResetCause());
            Console.WriteLine();

            printErrors(vars.errorStatus, "Errors currently stopping motor");
            printErrors(vars.errorOccurred, "Errors that occurred since last check");
            printSerialErrors(vars.serialErrorOccurred, "Serial errors that occurred since last check");
            printLimitStatus(vars.limitStatus, "Active limits");

            Console.WriteLine(channelStatusFormat, "Channel", "Unlimited", "Raw", "Scaled"); 
            printChannelStatus(vars.rc1, "RC 1");
            printChannelStatus(vars.rc2, "RC 2");
            printChannelStatus(vars.analog1, "Analog 1");
            printChannelStatus(vars.analog2, "Analog 2");
            Console.WriteLine();

            Console.WriteLine("Current Speed:  " + vars.speed);
            Console.WriteLine("Target Speed:   " + vars.targetSpeed);
            Console.WriteLine("Brake Amount:   " + (vars.brakeAmount == 0xFF ? "N/A" : vars.brakeAmount.ToString()));
            Console.WriteLine("VIN:            " + vars.vinMv.ToString() + " mV");
            Console.WriteLine("Temperature:    " + Smc.temperatureToString(vars.temperature));
            Console.WriteLine("RC Period:      " + Smc.rcPeriodToString(vars.rcPeriod));
            Console.WriteLine("Baud rate:      " + (vars.baudRateRegister == 0 ? "N/A" : (Smc.convertBaudRegisterToBps(vars.baudRateRegister).ToString() + " bps")));
            uint seconds = vars.timeMs / 1000;
            uint minutes = seconds / 60;
            uint hours = minutes / 60;
            Console.WriteLine("Up time:        {0}:{1:D2}:{2:D2}.{3:D3}", hours, minutes % 60, seconds % 60, vars.timeMs % 1000);
            Console.WriteLine();

            const string motorLimitFormat = "{0, -18} {1,9} {2,8}";
            Console.WriteLine(motorLimitFormat, "Limit", "Forward", "Reverse");
            Console.WriteLine(motorLimitFormat, "Max. speed", vars.forwardLimits.maxSpeed, vars.reverseLimits.maxSpeed);
            Console.WriteLine(motorLimitFormat, "Starting speed", vars.forwardLimits.startingSpeed, vars.reverseLimits.startingSpeed);
            Console.WriteLine(motorLimitFormat, "Max. acceleration",
                Smc.accelDecelToString(vars.forwardLimits.maxAcceleration),
                Smc.accelDecelToString(vars.reverseLimits.maxAcceleration));
            Console.WriteLine(motorLimitFormat, "Max. deceleration",
                Smc.accelDecelToString(vars.forwardLimits.maxDeceleration),
                Smc.accelDecelToString(vars.reverseLimits.maxDeceleration));
            Console.WriteLine(motorLimitFormat, "Brake duration", vars.forwardLimits.brakeDuration, vars.reverseLimits.brakeDuration);
        }

        const string channelStatusFormat = "{0,-8} {1,10} {2,8} {3,8}";

        private static void printChannelStatus(SmcChannelVariables vars, string channelName)
        {
            // Note: scaledValue is never 0xFFFF.  It is 0 when the channel is disconnected.
            Console.WriteLine(channelStatusFormat,
                channelName,
                vars.unlimitedRawValue == 0xFFFF ? "N/A" : vars.unlimitedRawValue.ToString(),
                vars.rawValue == 0xFFFF ? "N/A" : vars.rawValue.ToString(),
                vars.rawValue == 0xFFFF ? "N/A" : vars.scaledValue.ToString());
        }

        /// <summary>Prints a set of smc errors to the console in a user-friendly format.</summary>
        /// <param name="errors">The errors to print.</param>
        /// <param name="description">The description of this set of errors.</param>
        private static void printErrors(SmcError errors, string description)
        {
            if (errors == 0)
            {
                Console.WriteLine(description + ": None");
            }
            else
            {
                Console.WriteLine(description + ":");
                if (0 != (errors & SmcError.SafeStart)) Console.WriteLine("  Safe start violation");
                if (0 != (errors & SmcError.ChannelInvalid)) Console.WriteLine("  Required channel invalid");
                if (0 != (errors & SmcError.Serial)) Console.WriteLine("  Serial error");
                if (0 != (errors & SmcError.CommandTimeout)) Console.WriteLine("  Command timeout");
                if (0 != (errors & SmcError.LimitSwitch)) Console.WriteLine("  Limit/kill switch active");
                if (0 != (errors & SmcError.VinLow)) Console.WriteLine("  Low VIN");
                if (0 != (errors & SmcError.VinHigh)) Console.WriteLine("  High VIN");
                if (0 != (errors & SmcError.TemperatureHigh)) Console.WriteLine("  Over temperature");
                if (0 != (errors & SmcError.MotorDriverError)) Console.WriteLine("  Motor driver error");
                if (0 != (errors & SmcError.ErrLineHigh)) Console.WriteLine("  ERR line high");
            }
            Console.WriteLine();
        }

        /// <summary>Prints a set of smc serial errors to the console in a user-friendly format.</summary>
        /// <param name="errors">The serial errors to print.</param>
        /// <param name="description">The description of this set of serial errors.</param>
        private static void printSerialErrors(SmcSerialError serialErrors, string description)
        {
            if (serialErrors == 0)
            {
                Console.WriteLine(description + ": None");
            }
            else
            {
                Console.WriteLine(description + ":");
                if (0 != (serialErrors & SmcSerialError.Parity)) Console.WriteLine("  Parity");
                if (0 != (serialErrors & SmcSerialError.Frame)) Console.WriteLine("  Frame");
                if (0 != (serialErrors & SmcSerialError.Noise)) Console.WriteLine("  Noise");
                if (0 != (serialErrors & SmcSerialError.RxOverrun)) Console.WriteLine("  RX overrun");
                if (0 != (serialErrors & SmcSerialError.Format)) Console.WriteLine("  Format");
                if (0 != (serialErrors & SmcSerialError.Crc)) Console.WriteLine("  CRC");
            }
            Console.WriteLine();
        }

        /// <summary>Prints a set of smc limits to the console in a user-friendly format.</summary>
        /// <param name="errors">The limits to print.</param>
        /// <param name="description">The description of this set of limits.</param>
        private static void printLimitStatus(SmcLimitStatus limits, string description)
        {
            if (limits == 0)
            {
                Console.WriteLine(description + ": None");
            }
            else
            {
                Console.WriteLine(description + ":");
                if (0 != (limits & SmcLimitStatus.StartedState)) Console.WriteLine("  Motor not started");
                if (0 != (limits & SmcLimitStatus.Temperature)) Console.WriteLine("  Temperature");
                if (0 != (limits & SmcLimitStatus.MaxSpeed)) Console.WriteLine("  Max speed");
                if (0 != (limits & SmcLimitStatus.StartingSpeed)) Console.WriteLine("  Starting speed");
                if (0 != (limits & SmcLimitStatus.Acceleration)) Console.WriteLine("  Acceleration/Deceleration/Brake duration");
                if (0 != (limits & SmcLimitStatus.Rc1)) Console.WriteLine("  RC1 limit switch");
                if (0 != (limits & SmcLimitStatus.Rc2)) Console.WriteLine("  RC2 limit switch");
                if (0 != (limits & SmcLimitStatus.Analog1)) Console.WriteLine("  Analog1 limit switch");
                if (0 != (limits & SmcLimitStatus.Analog2)) Console.WriteLine("  Analog2 limit switch");
                if (0 != (limits & SmcLimitStatus.UsbKill)) Console.WriteLine("  USB kill switch (motor stopped by user)");
            }
            Console.WriteLine();
        }

        public static void listDevices()
        {
            List<DeviceListItem> list = Smc.getConnectedDevices();

            if (list.Count == 1)
            {
                Console.WriteLine("1 " + Smc.name + " found:");
            }
            else
            {
                Console.WriteLine(list.Count + " " + Smc.name + "s found:");
            }

            foreach (DeviceListItem item in list)
            {
                Console.WriteLine(item.text);
            }
        }

        /// <summary>
        /// Prints the exception and all its inner exceptions
        /// to the Console.  If one of them is an ArgumentException,
        /// prints the help message.
        /// </summary>
        private static void printExceptionAndHelp(Exception exception)
        {
            Console.Error.Write("Error: ");
            bool argumentException = false;
            while (exception != null)
            {
                Console.Error.WriteLine(exception.Message);

                if (exception is ArgumentException)
                {
                    argumentException = true;
                }

                if (exception is Win32Exception)
                {
                    Console.Error.WriteLine("Error code 0x" + ((Win32Exception)exception).NativeErrorCode.ToString("x") + ".");
                }

                exception = exception.InnerException;
            }

            if (argumentException)
            {
                Console.Error.WriteLine();
                Console.Error.Write(helpMessage());
            }
        }
    }
}
