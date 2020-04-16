using System;
using System.Collections.Generic;
using System.IO;
using System.Reflection;
using Pololu.UsbWrapper;
using Pololu.Usc.Bytecode;

/* CHANGELOG:
 * 2010-10-13: Changed assembly version to 1.3.2.1, and fixed a bug with the --sub command
 *   that was always changing the high byte to 0.
 */

namespace Pololu.Usc.UscCmd
{
    /// <summary>
    /// This class represents the executable commandline utility UscCmd.exe.
    /// </summary>
    class Program
    {
        static void Main(string[] args)
        {
            CommandOptions opts = new CommandOptions(Assembly.GetExecutingAssembly().GetName()+"\n"+
                "Select one of the following actions:\n"+
                "  --list                   list available devices\n"+
                "  --configure FILE         load configuration file into device\n"+
                "  --getconf FILE           read device settings and write configuration file\n"+
                "  --restoredefaults        restore factory settings\n"+
                "  --program FILE           compile and load bytecode program\n"+
                "  --status                 display complete device status\n"+
                "  --bootloader             put device into bootloader (firmware upgrade) mode\n"+
                "  --stop                   stops the script running on the device\n"+
                "  --start                  starts the script running on the device\n"+
                "  --restart                restarts the script at the beginning\n"+
                "  --step                   runs a single instruction of the script\n"+
                "  --sub NUM                calls subroutine n (can be hex or decimal)\n"+
                "  --sub NUM,PARAMETER      calls subroutine n with a parameter (hex or decimal)\n"+
                "                           placed on the stack\n"+
                "  --servo NUM,TARGET       sets the target of servo NUM in units of\n" +
                "                           1/4 microsecond\n"+
                "  --speed NUM,SPEED        sets the speed limit of servo NUM\n"+      
                "  --accel NUM,ACCEL        sets the acceleration of servo NUM to a value 0-255\n"+
                "Select which device to perform the action on (optional):\n"+
                "  --device 00001430        (optional) select device #00001430\n",
                args);

            if (opts["list"] != null)
            {
                if (opts.Count > 1)
                    opts.error();
                listDevices();
                return;
            }

            if (opts.Count == 0)
                opts.error();

            // otherwise, they must connect to a device

            List<DeviceListItem> list = Usc.getConnectedDevices();

            if (list.Count == 0)
            {
                System.Console.WriteLine("No " + Usc.englishName + " devices found.");
                return;
            }

            DeviceListItem item = null;

            // see if the device they specified was in the list
            if (opts["device"] == null)
            {
                 // Conenct to the first item in the list.
                item = list[0];
            }
            else
            {
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
                    Console.WriteLine("Could not find a " + Usc.englishName + " device with serial number " + opts["device"] + ".");
                    Console.WriteLine("To list devices, use the --list option.");
                    return;
                }
            }

            Usc usc = new Usc(item);

            if (opts["bootloader"] != null)
            {
                if (opts.Count > 2)
                    opts.error();

                usc.startBootloader();
                return;
            }
            else if (opts["status"] != null)
            {
                if (opts["status"] != "")
                    opts.error();
                displayStatus(usc);
            }
            else if (opts["getconf"] != null)
            {
                getConf(usc, opts["getconf"]);
            }
            else if (opts["configure"] != null)
            {
                configure(usc, opts["configure"]);
            }
            else if (opts["restoredefaults"] != null)
            {
                if (opts["restoredefaults"] != "")
                    opts.error();
                restoreDefaultConfiguration(usc);
            }
            else if (opts["program"] != null)
            {
                program(usc, opts["program"]);
            }
            else if (opts["stop"] != null)
            {
                setScriptDone(usc, 1);
            }
            else if (opts["start"] != null)
            {
                setScriptDone(usc, 0);
            }
            else if (opts["restart"] != null)
            {
                System.Console.Write("Restarting script...");
                usc.restartScript();
                usc.setScriptDone(0);
                System.Console.WriteLine("");
            }
            else if (opts["step"] != null)
            {
                setScriptDone(usc, 2);
            }
            else if (opts["servo"] != null)
            {
                string[] parts = opts["servo"].Split(',');
                if(parts.Length != 2)
                    opts.error("Wrong number of commas in the argument to servo.");
                byte servo=0;
                ushort target=0;
                try
                {
                    servo = byte.Parse(parts[0]);
                    target = ushort.Parse(parts[1]);
                }
                catch(FormatException)
                {
                    opts.error();
                }
                Console.Write("Setting target of servo "+servo+" to "+target+"...");
                usc.setTarget(servo, target);
                Console.WriteLine("");
            }
            else if (opts["speed"] != null)
            {
                string[] parts = opts["speed"].Split(',');
                if(parts.Length != 2)
                    opts.error("Wrong number of commas in the argument to speed.");
                byte servo=0;
                ushort speed=0;
                try
                {
                    servo = byte.Parse(parts[0]);
                    speed = ushort.Parse(parts[1]);
                }
                catch(FormatException)
                {
                    opts.error();
                }
                Console.Write("Setting speed of servo "+servo+" to "+speed+"...");
                usc.setSpeed(servo, speed);
                Console.WriteLine("");
            }
            else if (opts["accel"] != null)
            {
                string[] parts = opts["accel"].Split(',');
                if(parts.Length != 2)
                    opts.error("Wrong number of commas in the argument to accel.");
                byte servo=0;
                byte acceleration=0;
                try
                {
                    servo = byte.Parse(parts[0]);
                    acceleration = byte.Parse(parts[1]);
                }
                catch(FormatException)
                {
                    opts.error();
                }
                Console.Write("Setting acceleration of servo "+servo+" to "+acceleration+"...");
                usc.setAcceleration(servo, acceleration);
                Console.WriteLine("");
            }
            else if (opts["sub"] != null)
            {
                string[] parts = opts["sub"].Split(new char[] {','});
                if(parts.Length > 2)
                    opts.error("Too many commas in the argument to sub.");
                byte address=0;
                short parameter=0;
                try
                {
                    if(parts[0].StartsWith("0x"))
                        address = byte.Parse(parts[0].Substring(2),System.Globalization.NumberStyles.AllowHexSpecifier);
                    else
                        address = byte.Parse(parts[0]);
                }
                catch(FormatException)
                {
                    opts.error();
                }
                if(parts.Length == 2)
                {
                    try
                    {
                        if(parts[1].StartsWith("0x"))
                            parameter = short.Parse(parts[1].Substring(2),System.Globalization.NumberStyles.AllowHexSpecifier);
                        else
                            parameter = short.Parse(parts[1]);
                    }
                    catch(FormatException)
                    {
                        opts.error();
                    }

                    Console.Write("Restarting at subroutine "+address+" with parameter "+parameter+"...");
                    usc.restartScriptAtSubroutineWithParameter(address, parameter);
                    usc.setScriptDone(0);
                }
                else
                {
                    Console.Write("Restarting at subroutine "+address+"...");
                    usc.restartScriptAtSubroutine(address);
                    usc.setScriptDone(0);
                }
                Console.WriteLine("");
            }
            else opts.error();
        }

        static void setScriptDone(Usc usc, byte value)
        {
            Console.Write("Setting script state to "+value+"...");
            usc.setScriptDone(value);
            Console.WriteLine("");
        }

        static void restoreDefaultConfiguration(Usc usc)
        {
            Console.Write("Restoring default settings...");
            usc.restoreDefaultConfiguration();
            Console.WriteLine("");
        }

        static void listDevices()
        {
            List<DeviceListItem> list = Usc.getConnectedDevices();

            if (list.Count == 1)
                Console.WriteLine("1 "+Usc.englishName+" device found:");
            else
                Console.WriteLine(list.Count + " "+Usc.englishName + " devices found:");

            foreach (DeviceListItem item in list)
            {
                Console.WriteLine(item.text);
            }
        }

        static unsafe void displayStatus(Usc usc)
        {
            MaestroVariables variables;
            ServoStatus[] servos;
            short[] stack;
            ushort[] call_stack;
            usc.getVariables(out variables, out stack, out call_stack, out servos);

            Console.WriteLine(" #  target   speed   accel     pos");
            for (int i = 0; i < usc.servoCount; i++)
            {
                Console.WriteLine("{0,2}{1,8}{2,8}{3,8}{4,8}", i,
                    servos[i].target, servos[i].speed,
                    servos[i].acceleration, servos[i].position);
            }

            Console.Write("errors: 0x"+variables.errors.ToString("X4"));
            foreach (var error in Enum.GetValues(typeof(uscError)))
            {
                if((variables.errors & (1<<(int)(uscError)error)) != 0)
                {
                    Console.Write(" "+error.ToString());
                }
            }
            usc.clearErrors();
            Console.WriteLine(); // end the line

            if(variables.scriptDone == 0)
                Console.WriteLine("SCRIPT RUNNING");
            else
                Console.WriteLine("SCRIPT DONE");

            Console.WriteLine("program counter: "+variables.programCounter.ToString());

            Console.Write("stack:          ");
            for (int i = 0; i < variables.stackPointer; i++)
            {
                if(i > stack.Length)
                {
                    Console.Write(" OVERFLOW (stack pointer = "+variables.stackPointer+")");
                    break;
                }
                Console.Write(stack[i].ToString().PadLeft(8, ' '));
            }
            Console.WriteLine();

            Console.Write("call stack:     ");
            for (int i = 0; i < variables.callStackPointer; i++)
            {
                if(i > call_stack.Length)
                {
                    Console.Write(" OVERFLOW (call stack pointer = "+variables.callStackPointer+")");
                    break;
                }
                Console.Write(call_stack[i].ToString().PadLeft(8, ' '));
            }
            Console.WriteLine();
        }

        static void getConf(Usc usc, string filename)
        {
            Stream file = File.Open(filename, FileMode.Create);
            StreamWriter sw = new StreamWriter(file);
            ConfigurationFile.save(usc.getUscSettings(), sw);
            sw.Close();
            file.Close();
        }

        static void configure(Usc usc, string filename)
        {
            Stream file = File.Open(filename, FileMode.Open);
            StreamReader sr = new StreamReader(file);
            List<String> warnings = new List<string>();
            UscSettings settings = ConfigurationFile.load(sr, warnings);
            usc.fixSettings(settings, warnings);
            usc.setUscSettings(settings, true);
            sr.Close();
            file.Close();
            usc.reinitialize();
        }

        static void program(Usc usc, string filename)
        {
            string text = (new StreamReader(filename)).ReadToEnd();
            BytecodeProgram program = BytecodeReader.Read(text, usc.servoCount != 6);
            BytecodeReader.WriteListing(program,filename+".lst");

            usc.setScriptDone(1);
            usc.eraseScript();

            List<byte> byteList = program.getByteList();
            if (byteList.Count > usc.maxScriptLength)
            {
                throw new Exception("Script too long for device (" + byteList.Count + " bytes)");
            }
            if (byteList.Count < usc.maxScriptLength)
            {
                byteList.Add((byte)Opcode.QUIT);
            }

            System.Console.WriteLine("Setting up subroutines...");
            usc.setSubroutines(program.subroutineAddresses, program.subroutineCommands);
            
            System.Console.WriteLine("Loading "+byteList.Count+" bytes...");

            usc.writeScript(byteList);
            System.Console.WriteLine("Restarting...");
            usc.reinitialize();
        }
    }
}

// Local Variables: **
// mode: java **
// c-basic-offset: 4 **
// tab-width: 4 **
// indent-tabs-mode: nil **
// end: **
