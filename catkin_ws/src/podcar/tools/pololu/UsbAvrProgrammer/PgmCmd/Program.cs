using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Pololu.UsbAvrProgrammer;
using Pololu.UsbWrapper;
using System.Text.RegularExpressions;

namespace Pololu.UsbAvrProgrammer.PgmCmd
{
    static class Program
    {
        static string helpMessage()
        {
            return "PgmCmd: Configuration and status utility for the Pololu USB AVR Programmer.\n" +
                "Version: " + System.Reflection.Assembly.GetExecutingAssembly().GetName().Version.ToString() + "\n" +
                "Options:\n" +
                " -l, --list             list available devices\n" +
                " -d, --device SERIALNUM (optional) select device with given serial number\n" +
                " -s, --status           display complete device status\n" +
                "     --freq NUM         sets the ISP frequency (in units of kHz)\n" +
                "     --linea ID\n" +
                "       or --lineb ID    set serial control signal associated with line A or B.\n" +
                "                        Valid IDs are: none, cd, dsr, ri, dtr, rts.\n" +
                "                        Warning: dtr and rts are outputs: -f option is required\n" +
                "     --swminor HEXNUM   AVR ISP software version minor (in hex, e.g. A)\n" +
                "     --swmajor HEXNUM   AVR ISP software version major (in hex)\n" +
                "     --hw HEXNUM        AVR ISP software hardware version (in hex)\n" +
                "     --vddmin NUM       set minimum allowed target vdd (units of mV)\n" +
                "     --vddmaxrange NUM  set maximum allowed target vdd range (units of mV)\n" +
                "     --restoredefaults  restore factory settings\n" +
                "     --bootloader       put device in to bootloader (firmware upgrade) mode\n";
        }

        static void Main(string[] args)
        {
            try
            {
                MainWithExceptions(args);
            }
            catch (Exception exception)
            {
                Console.Write("Error: ");
                bool argumentException = false;
                while (exception != null)
                {
                    Console.WriteLine(exception.Message);

                    if (exception is ArgumentException)
                    {
                        argumentException = true;
                    }

                    exception = exception.InnerException;
                }

                if (argumentException)
                {
                    Console.WriteLine();
                    Console.Write(helpMessage());
                }
                Environment.Exit(1);
            }
        }

        static void MainWithExceptions(string[] args)
        {
            if (args.Length == 0)
            {
                Console.Write(helpMessage());
                Environment.Exit(1);
            }

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
                    case "-f": arg = "--force"; break;
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

            List<DeviceListItem> list = Programmer.getConnectedDevices();

            // Make sure there is a device available..
            if (list.Count == 0)
            {
                throw new Exception("No " + Programmer.englishName + "s found.");
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
                // but if someone writes it, the program should still work.
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
                    throw new Exception("Could not find a " + Programmer.englishName + " with serial number " + opts["device"] + ".\n"+
                        "To list devices, use the --list option.");
                }
            }

            Programmer programmer = new Programmer(item);

            if (opts.ContainsKey("bootloader"))
            {
                programmer.startBootloader();
                return;
            }

            if (opts.ContainsKey("restoredefaults"))
            {
                programmer.restoreDefaultSettings();
            }

            if (opts.ContainsKey("freq"))
            {
                setFrequency(programmer, opts["freq"]);
            }

            if (opts.ContainsKey("vddmin"))
            {
                programmer.setTargetVccAllowedMinimum(stringToMillivolts(opts["vddmin"]));
            }

            if (opts.ContainsKey("vddmaxrange"))
            {
                programmer.setTargetVccAllowedMaximumRange(stringToMillivolts(opts["vddmaxrange"]));
            }

            if (opts.ContainsKey("linea"))
            {
                programmer.setLineAIdentity(stringToLineIdentity(opts["linea"], opts.ContainsKey("force")));
            }

            if (opts.ContainsKey("lineb"))
            {
                programmer.setLineBIdentity(stringToLineIdentity(opts["lineb"], opts.ContainsKey("force")));
            }

            if (opts.ContainsKey("swminor"))
            {
                programmer.setSoftwareVersionMinor(stringToVersionNumber(opts["swminor"]));
            }

            if (opts.ContainsKey("swmajor"))
            {
                programmer.setSoftwareVersionMajor(stringToVersionNumber(opts["swmajor"]));
            }

            if (opts.ContainsKey("hw"))
            {
                programmer.setHardwareVersion(stringToVersionNumber(opts["hw"]));
            }

            if (opts.ContainsKey("status"))
            {
                displayStatus(programmer);
            }

            programmer.disconnect();
        }

        static byte stringToVersionNumber(string input)
        {
            try
            {
                return byte.Parse(input, System.Globalization.NumberStyles.HexNumber);
            }
            catch (Exception exception)
            {
                throw new ArgumentException("Invalid version number \"" + input + "\".", exception);
            }
        }

        static ushort stringToMillivolts(string input)
        {
            try
            {
                ushort value = ushort.Parse(input);
                if (value > 255 * 32)
                {
                    throw new ArgumentException("Voltage \"" + input + "\" is too high.  Max is 8160.");
                }
                return value;
            }
            catch (Exception exception)
            {
                throw new ArgumentException("Invalid voltage \"" + input + "\".", exception);
            }
        }

        static LineIdentity stringToLineIdentity(string id, bool forceArgumentPresent)
        {
            switch (id.ToLowerInvariant())
            {
                case "nothing":
                case "none": return LineIdentity.None;
                case "cd": return LineIdentity.CD;
                case "dsr": return LineIdentity.DSR;
                case "ri": return LineIdentity.RI;
                case "dtr":
                    if (!forceArgumentPresent) { throw forceArgumentNeeded(); }
                    return LineIdentity.DTR;
                case "rts":
                    if (!forceArgumentPresent) { throw forceArgumentNeeded(); }
                    return LineIdentity.RTS;
                default: throw new ArgumentException("Unrecognized line identity " + id.ToString() + ".");
            }
        }

        static ArgumentException forceArgumentNeeded()
        {
            return new ArgumentException("To set line A or line B as outputs, you must provide the -f option.\n" +
                "Make sure that setting A and B as outputs will not cause a short, then try\n" +
                "again with -f.");
        }

        static void setFrequency(Programmer programmer, string frequencyString)
        {
            // Convert the frequency string to a decimal.
            decimal frequency;
            try
            {
                frequency = decimal.Parse(frequencyString);
            }
            catch (Exception exception)
            {
                throw new ArgumentException("Invalid frequency \"" + frequencyString + "\".", exception);
            }

            // Convert the frequency to the SckDuration parameter.
            bool exactMatch;
            SckDuration sckDuration = Programmer.frequencyToSckDuration(frequency, out exactMatch);
            string sckDurationString = Programmer.sckDurationToString(sckDuration);
            if (!exactMatch)
            {
                Console.WriteLine("Frequency " + frequencyString + " kHz not supported, using " + sckDurationString + " instead.");
            }

            // Set the SckDuration.
            programmer.setSckDuration(sckDuration);
        }

        static void displayStatus(Programmer programmer)
        {
            Console.Write(
                "Serial number:                  " + programmer.getSerialNumber() + "\n" +
                "Firmware version:               " + programmer.firmwareVersionString + "\n"
            );

            ProgrammerSettings settings = programmer.getSettings();

            Console.Write(
                "Settings:\n" +
                "  ISP Frequency:                " + Programmer.sckDurationToString(settings.sckDuration) + "\n" +
                // TODO: test displayStatus thoroughly, line by line, making sure that every line displays the correct thing for each possible value
                "  Line A Identity:              " + Programmer.lineIdentityToString(settings.lineAIdentity) + "\n" +
                "  Line B Identity:              " + Programmer.lineIdentityToString(settings.lineBIdentity) + "\n" +
                "  AVR ISP hardware version:     " + settings.hardwareVersion.ToString("X") + "\n" +
                "  AVR ISP software version:     " + settings.softwareVersionMajor.ToString("X") + "." + settings.softwareVersionMinor.ToString("X") + "\n" +
                "  Target VDD allowed minimum:   " + settings.targetVccAllowedMinimum + " mV\n" +
                "  Target VDD allowed max range: " + settings.targetVccAllowedMaximumRange + " mV\n"
                );
            Console.Write(
                "Last programming:\n" +
                "  Error: "
            );

            switch (programmer.getProgrammingError())
            {
                case ProgrammingError.None:
                    Console.Write("None\n");
                    break;
                case ProgrammingError.TargetVccBad:
                    Console.Write(
                        "Target VDD bad\n"+
                        "    Target VDD either went too low or had too much range, so programming was\n" +
                        "    aborted.  Make sure that the target is powered on and its batteries are not\n" +
                        "    too low (if applicable).\n");
                    break;
                case ProgrammingError.Synch:
                    Console.Write(
                        "Synch failed\n"+
                        "    The SPI command for entering programming mode was sent, but the expected\n" +
                        "    response from the target was not received.  Make sure that the ISP\n" +
                        "    frequency setting is less than 1/6th of the target's clock frequency.\n");
                    break;
                case ProgrammingError.IdleForTooLong:
                    Console.Write("Idle for too long\n" +
                        "    The programmer received no programming commands from the computer for a\n" +
                        "    time longer than the timeout period, so it left programming mode.  Make\n" +
                        "    sure that the programming software does not wait too long between\n" +
                        "    successive programming commands.\n");
                    break;
                case ProgrammingError.UsbNotConfigured:
                    Console.Write("USB not configured\n" +
                        "    The computer's USB controller deconfigured the progammer, so programming was\n" +
                        "    aborted.\n");
                    break;
                case ProgrammingError.UsbSuspend:
                    Console.Write("USB suspend\n" +
                        "    The computer's USB controller put the programmer in suspend mode, so\n" +
                        "    programming was aborted.\n");
                    break;
            }

            ushort min = programmer.getTargetVccMeasuredMinimum();
            ushort max = programmer.getTargetVccMeasuredMaximum();

            string minString;
            string rangeString;
            if (min <= max)
            {
                minString = min.ToString() + " mV";
                if (min < settings.targetVccAllowedMinimum)
                {
                    minString += " !";
                }

                rangeString = (max - min).ToString() + " mV";

                // We do the exact same comparison here that is done in the firmware.
                if ((byte)(max/32) > (byte)((min/32) + (settings.targetVccAllowedMaximumRange/32)))
                {
                    rangeString += " !";
                }
            }
            else
            {
                // Programmer hasn't programmed since it was las powered on.
                minString = rangeString = "N/A";
            }

            Console.Write(
                "  Measured Target VDD Minimum:  " + minString + "\n" +
                "  Measured Target VDD Range:    " + rangeString + "\n"
            );

            SloscopeOutputState outputA;
            SloscopeOutputState outputB;
            programmer.getSloscopeOutputState(out outputA, out outputB);

            Console.Write(
                "SLO-scope:\n" +
                "  State:                        " + programmer.getSloscopeState() + "\n" +
                "  Line A output:                " + outputA + "\n" +
                "  Line B output:                " + outputB + "\n"
            );
        }

        static void listDevices()
        {
            List<DeviceListItem> list = Programmer.getConnectedDevices();

            if (list.Count == 1)
            {
                Console.WriteLine("1 " + Programmer.englishName + " found:");
            }
            else
            {
                Console.WriteLine(list.Count + " " + Programmer.englishName + "s found:");
            }

            foreach (DeviceListItem item in list)
            {
                Console.WriteLine(item.text);
            }
        }

    }
}
