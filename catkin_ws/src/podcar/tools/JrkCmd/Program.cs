using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.IO;
using System.Text.RegularExpressions;
using System.Threading;
using Pololu.Jrk;
using Pololu.UsbWrapper;

namespace Pololu.Jrk.JrkCmd
{
    class Program
    {
        static Jrk jrk;
        static Byte currentCalibrationForward;
        static Byte currentCalibrationReverse;

        static UInt32 streamLineCount = 0;
        static Nullable<UInt32> streamLineCountLimit = null;
        static DateTime streamStartTime;
        static UInt16 streamLastPidPeriodCount;
        static UInt32 streamTotalPidPeriodCount;
        static string streamFormat;

        static string helpMessage()
        {
            return "JrkCmd: Configuration and control utility for the Jrk Motor Controller.\n" +
                "Version: " + System.Reflection.Assembly.GetExecutingAssembly().GetName().Version.ToString() + "\n" +
                "Options:\n" +
                " -l, --list              list available devices\n" +
                " -d, --device SERIALNUM  (optional) select device with given serial number\n" +
                " -s, --status            display complete device status\n" +
                "     --target NUM        set the target value (if input mode is Serial)\n" +
                "     --stop              stop the motor\n" +
                "     --run               run the motor\n" +
                "     --clearerrors       clear the latched errors\n" +
                "     --restoredefaults   restore factory settings\n" +
                "     --configure FILE    load configuration file into device\n" +
                "     --getconf FILE      read device settings and write to file\n" + 
                "     --bootloader        put device in to bootloader (firmware upgrade) mode\n" +
                "Stream-related options:\n" +
                "     --stream            stream variables from Jrk\n" +
                "     --interval NUM      milliseconds between readings (default 20)\n" +
                "     --separator SEP     (optional) specifies the field separator:\n" +
                "                         can be \"space\", \"tab\", \"comma\" or arbitrary string\n" +
                "     --limit NUM         (optional) exit after printing NUM lines.\n" +
                "                         Omitting this option makes the stream unlimited.\n" +
                "     --noheader          don't print the header line.\n" +
                "     --nosleep           more reliable timing but more CPU usage.\n" +
                "     --format            (optional) specifies the output format as a Microsoft\n" +
                "                         Composite Formatting string.  Index numbers are 0-10:\n" +
                "                         0=Time, 1=Period#, 2=Input, 3=Target, 4=FB,\n" +
                "                         5=ScaledFB, 6=Integral, 7=DutyTarget, 8=Duty,\n" +
                "                         9=Current, 10=ErrorCode, 11=PID Period Exceeded\n" +
                "                         e.g: \"{0,6},{8,6},{9,6}\" prints Time, Duty, Current\n";
        }

        static void Main(string[] args)
        {
            try
            {
                MainWithExceptions(args);
            }
            catch (Exception exception)
            {
                bool argumentException = printException(exception);

                if (argumentException)
                {
                    Console.Error.WriteLine();
                    Console.Error.Write(helpMessage());
                }
                Environment.Exit(1);
            }
        }

        static void MainWithExceptions(string[] args)
        {
            // If no arguments are given, just show the help message.
            if (args.Length == 0)
            {
                Console.Write(helpMessage());
                Environment.Exit(2);
            }

            // Parse the arguments.
            Dictionary<String, String> opts = new Dictionary<string, string>();
            string name = null;
            foreach (string rawArg in args)
            {
                string arg = rawArg;

                // Transform the short names in to the long names.
                switch (arg)
                {
                    case "-l": arg = "--list"; break;
                    case "-d": arg = "--device"; break;
                    case "-s": arg = "--status"; break;
                }

                Match m = Regex.Match(arg, "^--(.*)");
                if (m.Success)
                {
                    name = m.Groups[1].ToString();
                    opts[name] = ""; // start it off with no string value
                }
                else if (name != null)
                {
                    // This argument is right after a -- argument, so this argument
                    // is its value.
                    opts[name] = arg;
                    name = null;
                }
                else
                {
                    throw new ArgumentException("Unexpected argument \"" + arg +"\".");
                }
            }

            if (opts.ContainsKey("list"))
            {
                if (args.Length > 1) { throw new ArgumentException("If --list is present, it must be the only option."); }
                listDevices();
                return;
            }

            // Otherwise, we have to connect to a device.

            List<DeviceListItem> list = Jrk.getConnectedDevices();

            // Make sure there is a device available..
            if (list.Count == 0)
            {
                throw new Exception("No jrks found.");
            }

            DeviceListItem item = null;

            if (!opts.ContainsKey("device"))
            {
                // No serial number specified: connect to the first item in the list.
                item = list[0];
            }
            else
            {
                // Serial number specified.

                // Remove the leading # sign.  It is not standard to put it there,
                // but if someone writes it, this program should still work.
                string check_serial_number = opts["device"].TrimStart('#');

                // Find the device with the specified serial number.
                foreach (DeviceListItem check_item in list)
                {
                    if (check_item.serialNumber == check_serial_number)
                    {
                        item = check_item;
                        break;
                    }
                }
                if (item == null)
                {
                    throw new Exception("Could not find a jrk with serial number " + opts["device"] + ".\n"+
                        "To list devices, use the --list option.");
                }
            }

            // Connect to the device.
            jrk = new Jrk(item);

            if (opts.ContainsKey("bootloader"))
            {
                jrk.startBootloader();
                return;
            }

            if (opts.ContainsKey("restoredefaults"))
            {
                jrk.setJrkParameter(jrkParameter.PARAMETER_INITIALIZED, 0xFF);
                jrk.reinitialize();
                Thread.Sleep(1000);
            }

            if (opts.ContainsKey("configure"))
            {
                string filename = opts["configure"];
                Stream stream = File.Open(filename, FileMode.Open);
                StreamReader sr = new StreamReader(stream);
                ConfigurationFile.load(sr, jrk);
                sr.Close();
                stream.Close();
                jrk.reinitialize();
            }

            if (opts.ContainsKey("getconf"))
            {
                string filename = opts["getconf"];
                Stream stream = File.Open(filename, FileMode.Create);
                StreamWriter sw = new StreamWriter(stream);
                ConfigurationFile.save(sw, jrk);
                sw.Close();
                stream.Close();
            }

            if (opts.ContainsKey("clearerrors"))
            {
                jrk.clearErrors();
            }

            if (opts.ContainsKey("target"))
            {
                UInt16 target = stringToU12(opts["target"]);
                jrk.setTarget(target);
            }

            if (opts.ContainsKey("run"))
            {
                UInt16 target = jrk.getVariables().target;
                jrk.setTarget(target);
            }

            if (opts.ContainsKey("stop"))
            {
                jrk.motorOff();
            }

            if (opts.ContainsKey("status"))
            {
                displayStatus(jrk);
            }

            if (opts.ContainsKey("stream"))
            {
                streamVariables(jrk, opts);
            }

            jrk.disconnect();
        }

        /// <summary>
        /// Prints the exception and all its inner exceptions
        /// to the Console.  Returns true if one of them was
        /// of type "ArgumentException" (indicating that the
        /// user probably did not understand what to type on
        /// the command line.)
        /// </summary>
        private static bool printException(Exception exception)
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
            return argumentException;
        }
        static UInt16 stringToU12(string input)
        {
            try
            {
                UInt16 value = UInt16.Parse(input);
                if (value > 4095)
                {
                    throw new ArgumentException("Maximum allowed value is 4095.");
                }
                return value;
            }
            catch (Exception exception)
            {
                throw new ArgumentException("Invalid argument \"" + input + "\".", exception);
            }
        }

        static void streamVariables(Jrk jrk, Dictionary<String, String> opts)
        {
            // Determine what interval to use (the time between each reading).
            int interval = 20;
            if (opts.ContainsKey("interval"))
            {
                try
                {
                    interval = int.Parse(opts["interval"]);

                    if (interval < 0)
                    {
                        throw new Exception("Value must be a non-negative whole number.");
                    }
                }
                catch (Exception exception)
                {
                    throw new Exception("Invalid interval parameter \"" + opts["interval"] + "\".", exception);
                }
            }

            // Determine the output format.
            if (opts.ContainsKey("format"))
            {
                // The user specified the format string completely.
                streamFormat = opts["format"].Replace("\\t", "\t");
            }
            else
            {
                string separator = ",";

                if (opts.ContainsKey("separator"))
                {
                    // The user just specified the separator
                    separator = opts["separator"];

                    switch (separator)
                    {
                        case "space": separator = " "; break;
                        case "\\t":
                        case "tab": separator = "\t"; break;
                        case "comma": separator = ","; break;
                    }
                }

                if (separator == "\t")
                {
                    // If the separator is a tab, then we don't need to have padding spaced.
                    streamFormat = "{0}\t{1}\t{2}\t{3}\t{4}\t{5}\t{6}\t{7}\t{8}\t{9}\t{10}";
                }
                else
                {
                    // If the separator is something else, then we want padding to make each
                    // field be a fixed width.

                    streamFormat = "{0,6}" + separator +
                        "{1,6}" + separator +
                        "{2,4}" + separator +
                        "{3,4}" + separator +
                        "{4,4}" + separator +
                        "{5,4}" + separator +
                        "{6,6}" + separator +
                        "{7,6}" + separator +
                        "{8,4}" + separator +
                        "{9,6}" + separator +
                        "{10,4}";
                }
            }

            
            // Determine the limit.
            if (opts.ContainsKey("limit"))
            {
                try
                {
                    streamLineCountLimit = UInt32.Parse(opts["limit"]);
                }
                catch(Exception exception)
                {
                    throw new Exception("Invalid limit parameter \"" + opts["limit"] + "\".", exception);
                }
            }

            // Print the header line if the user wanted it
            if (!opts.ContainsKey("noheader"))
            {
                Console.WriteLine(streamFormat,
                    "Time (ms)",
                    "PID Period Count",
                    "Input",
                    "Target",
                    "Feedback",
                    "Scaled feedback",
                    "Integral",
                    "Duty cycle target",
                    "Duty cycle",
                    "Current (mA)",
                    "Error code",
                    "PID period exceeded");
            }

            // Prepare to process the current readings the Jrk will be sending us.
            storeCurrentCalibration();

            if (interval == 0)
            {
                // Interval is zero so the user just wants the data as fast as possible.
                while (true)
                {
                    streamPrintReading(DateTime.Now, jrk.getVariables());
                }
            }
            else
            {
                // The three timer classes provided by the .NET framework do not
                // provide accurate timing so instead we poll DateTime.Now.
                // This takes more CPU power than the timer method, but we try
                // to minimize the CPU use by sleeping between readings.

                DateTime now = DateTime.Now;
                DateTime nextUpdateTime = now;
                while (true)
                {
                    if (nextUpdateTime <= now)
                    {
                        // Get the reading from the Jrk over USB (should take about .2 ms).
                        jrkVariables vars = jrk.getVariables();

                        streamPrintReading(now, vars);

                        while (nextUpdateTime <= DateTime.Now)
                        {
                            nextUpdateTime = nextUpdateTime.AddMilliseconds(interval);
                        }

                        // Conserve CPU power by sleeping
                        if (interval > 4 && !opts.ContainsKey("nosleep"))
                        {
                            Thread.Sleep(interval - 4);
                        }
                    }
                    now = DateTime.Now;
                }
            }
        }

        /// <summary>
        /// Call this function frequently, and it will take care of all the
        /// details printing the streamed variables from the Jrk.
        /// streamLineCount should be 0 before calling it for the first time.
        /// </summary>
        /// <param name="now">Should be approximately DateTime.Now.</param>
        static void streamPrintReading(DateTime now, jrkVariables vars)
        {
            try
            {
                uint time;

                if (streamLineCount == 0)
                {
                    // This will be the first line of the stream.

                    streamStartTime = now;
                    streamTotalPidPeriodCount = 0;
                    time = 0;
                }
                else
                {
                    // This is not the first line of the stream.

                    if (streamLastPidPeriodCount == vars.pidPeriodCount)
                    {
                        // We already printed this set of variables so skip it.
                        return;
                    }

                    UInt16 increment = (UInt16)(vars.pidPeriodCount - streamLastPidPeriodCount);
                    streamTotalPidPeriodCount += increment;

                    time = (uint)(now - streamStartTime).TotalMilliseconds;
                }
                streamLastPidPeriodCount = vars.pidPeriodCount;

                // Print the reading.
                Console.WriteLine(streamFormat,
                    time,
                    streamTotalPidPeriodCount,
                    vars.input,
                    vars.target,
                    vars.feedback,
                    vars.scaledFeedback,
                    vars.errorSum,
                    vars.dutyCycleTarget,
                    vars.dutyCycle,
                    currentToMilliamps(vars.current, vars.dutyCycle),
                    vars.errorFlagBits,
                    vars.pidPeriodExceeded
                    );

                // Keep track of how many liens we have printed.
                streamLineCount++;

                if (streamLineCountLimit.HasValue && streamLineCount >= streamLineCountLimit.Value)
                {
                    // We've printed all the lines that the user wanted, so quit.
                    Environment.Exit(0);
                }
            }
            catch (Exception exception)
            {
                printException(exception);
                Environment.Exit(3);
            }
        }

        static void displayStatus(Jrk jrk)
        {
            Console.Write(
                "Serial number:       " + jrk.getSerialNumber() + "\n" +
                "Firmware version:    " + jrk.firmwareVersionString + "\n"
            );

            storeCurrentCalibration();

            jrkVariables vars = jrk.getVariables();

            Console.Write(
                "Variables:\n" +
                "  Input:             " + vars.input + "\n" +
                "  Target:            " + vars.target + "\n" +
                "  Feedback:          " + vars.feedback + "\n" + 
                "  Scaled feedback:   " + vars.scaledFeedback + "\n" +
                "  Error:             " + (vars.scaledFeedback  - vars.target) + "\n" +
                "  Integral:          " + vars.errorSum + "\n" +
                "  Duty cycle target: " + vars.dutyCycleTarget + "\n" +
                "  Duty cycle:        " + vars.dutyCycle + "\n" +
                "  Current (mA):      " + currentToMilliamps(vars.current, vars.dutyCycle) + "\n"
            );

            if (vars.errorFlagBits == 0)
            {
                Console.WriteLine("No errors.");
            }
            else
            {
                Console.WriteLine("Errors:");
                if (1 == (1 & vars.errorFlagBits >> (byte)jrkError.ERROR_AWAITING_COMMAND)) Console.WriteLine("  Awaiting command");
                if (1 == (1 & vars.errorFlagBits >> (byte)jrkError.ERROR_NO_POWER)) Console.WriteLine("  No power");
                if (1 == (1 & vars.errorFlagBits >> (byte)jrkError.ERROR_MOTOR_DRIVER)) Console.WriteLine("  Motor driver error");
                if (1 == (1 & vars.errorFlagBits >> (byte)jrkError.ERROR_INPUT_INVALID)) Console.WriteLine("  Input invalid");
                if (1 == (1 & vars.errorFlagBits >> (byte)jrkError.ERROR_INPUT_DISCONNECT)) Console.WriteLine("  Input disconnect");
                if (1 == (1 & vars.errorFlagBits >> (byte)jrkError.ERROR_FEEDBACK_DISCONNECT)) Console.WriteLine("  Feedback disconnect");
                if (1 == (1 & vars.errorFlagBits >> (byte)jrkError.ERROR_MAXIMUM_CURRENT_EXCEEDED)) Console.WriteLine("  Max. current exceeded");
                if (1 == (1 & vars.errorFlagBits >> (byte)jrkError.ERROR_SERIAL_SIGNAL)) Console.WriteLine("  Serial signal error");
                if (1 == (1 & vars.errorFlagBits >> (byte)jrkError.ERROR_SERIAL_OVERRUN)) Console.WriteLine("  Serial overrun");
                if (1 == (1 & vars.errorFlagBits >> (byte)jrkError.ERROR_SERIAL_BUFFER_FULL)) Console.WriteLine("  Serial RX buffer full");
                if (1 == (1 & vars.errorFlagBits >> (byte)jrkError.ERROR_SERIAL_CRC)) Console.WriteLine("  Serial CRC error");
                if (1 == (1 & vars.errorFlagBits >> (byte)jrkError.ERROR_SERIAL_PROTOCOL)) Console.WriteLine("  Serial protocol error");
                if (1 == (1 & vars.errorFlagBits >> (byte)jrkError.ERROR_SERIAL_TIMEOUT)) Console.WriteLine("  Serial timeout error");
            };
        }

        static void storeCurrentCalibration()
        {
            currentCalibrationForward = (Byte)jrk.getJrkParameter(jrkParameter.PARAMETER_MOTOR_CURRENT_CALIBRATION_FORWARD);
            currentCalibrationReverse = (Byte)jrk.getJrkParameter(jrkParameter.PARAMETER_MOTOR_CURRENT_CALIBRATION_REVERSE);
        }

        /// <summary>
        /// Converts the raw, uncalibrated current reading from the Jrk in to milliamps.
        /// You must call storeCurrentCalibration() some time before calling this for
        /// the first time.
        /// </summary>
        /// <param name="rawCurrent">Raw reading form the jrk (jrk.getVariables().current).</param>
        /// <param name="dutyCycle">The duty cycle (-600 to 600).</param>
        /// <returns>The calibrated current in milliamps.  This is a signed number, the sign is equal
        /// to the sign of the duty cycle.</returns>
        static int currentToMilliamps(byte rawCurrent, short dutyCycle)
        {
            int current = rawCurrent;

            if (dutyCycle > 0)
            {
                current *= currentCalibrationForward;
            }
            else
            {
                current *= currentCalibrationReverse;
            }

            if (jrk.divideCurrent)
            {
                if (dutyCycle == 0)
                {
                    current = 0;
                }
                else
                {
                    current = (current * 600) / dutyCycle;
                }
            }
            return current;
        }

        /// <summary>
        /// Prints a list of the serial numbers of all Jrks connected to
        /// the computer.
        /// </summary>
        static void listDevices()
        {
            List<DeviceListItem> list = Jrk.getConnectedDevices();

            if (list.Count == 1)
            {
                Console.WriteLine("1 jrk found:");
            }
            else
            {
                Console.WriteLine(list.Count + " " + Jrk.shortProductName + "s found:");
            }

            foreach (DeviceListItem item in list)
            {
                Console.WriteLine(item.text);
            }
        }
    }
}
