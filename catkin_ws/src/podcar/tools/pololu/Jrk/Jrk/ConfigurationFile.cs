using System;
using System.Collections.Generic;
using Pololu.Jrk;
using System.Text;
using System.IO;

namespace Pololu.Jrk
{
    public static class ConfigurationFile
    {
        public static void load(StreamReader sr, IJrkParameterHolder parameterDestination)
        {
            // set up dictionaries for each of the enums (couldn't this be done somewhere else?)
            string[] names = Enum.GetNames(typeof(jrkParameter));
            byte[] values = (byte[])Enum.GetValues(typeof(jrkParameter));

            var dictionary = new Dictionary<string, byte>();
            int i;
            for (i = 0; i < names.Length; i++)
            {
                dictionary[names[i]] = values[i];
            }

            names = Enum.GetNames(typeof(jrkFeedbackMode));
            values = (byte[])Enum.GetValues(typeof(jrkFeedbackMode));
            var feedback_mode_dictionary = new Dictionary<string, byte>();
            for (i = 0; i < names.Length; i++)
            {
                feedback_mode_dictionary[names[i]] = values[i];
            }

            names = Enum.GetNames(typeof(jrkInputMode));
            values = (byte[])Enum.GetValues(typeof(jrkInputMode));
            var input_mode_dictionary = new Dictionary<string, byte>();
            for (i = 0; i < names.Length; i++)
            {
                input_mode_dictionary[names[i]] = values[i];
            }

            names = Enum.GetNames(typeof(jrkSerialMode));
            values = (byte[])Enum.GetValues(typeof(jrkSerialMode));
            var serial_mode_dictionary = new Dictionary<string, byte>();
            for (i = 0; i < names.Length; i++)
            {
                serial_mode_dictionary[names[i]] = values[i];
            }

            string line;                
            int line_number = 0;
            while (!sr.EndOfStream)
            {
                line = sr.ReadLine();
                line_number ++;
                string[] parts = line.Split(new char[] { ' ', '\t' });
                string parameter_name="", parameter_value="";

                // break up the string into a name and value pair
                foreach (string part in parts)
                {
                    if (part != "")
                    {
                        if (parameter_name == "")
                            parameter_name = part;
                        else if (parameter_value == "")
                            parameter_value = part;
                        else
                        {
                            throw new Exception("Error reading file: too many words on line " + line_number.ToString());
                        }
                    }
                }

                if (parameter_name == "")
                    continue; // blank line
                
                if(parameter_value == "")
                {
                    throw new Exception("Error reading file: missing value on line " + line_number.ToString());
                }
                
                // try to get the parameter and its value
                jrkParameter parameter;
                uint uint_value;
                try
                {
                    parameter = (jrkParameter)dictionary["PARAMETER_"+parameter_name];

                    switch (parameter)
                    {
                        case jrkParameter.PARAMETER_FEEDBACK_MODE:
                            uint_value = feedback_mode_dictionary["FEEDBACK_MODE_"+parameter_value];
                            break;
                        case jrkParameter.PARAMETER_INPUT_MODE:
                            uint_value = input_mode_dictionary["INPUT_MODE_"+parameter_value];
                            break;
                        case jrkParameter.PARAMETER_SERIAL_MODE:
                            uint_value = serial_mode_dictionary["SERIAL_MODE_"+parameter_value];
                            break;
                        default:
                            uint_value = uint.Parse(parameter_value);
                            break;
                    }
                }
                catch (KeyNotFoundException)
                {
                    throw new Exception("Error reading file: did not understand line " + line_number.ToString());
                }
                catch (FormatException)
                {
                    throw new Exception("Error reading file: did not understand a value on line " + line_number.ToString());
                }

                // now we can set it
                parameterDestination.setJrkParameter(parameter, uint_value);
            }

        }

        public static void save(StreamWriter sw, IJrkParameterHolder parameterSource)
        {
            string[] names = Enum.GetNames(typeof(jrkParameter));
            byte[] values = (byte[])Enum.GetValues(typeof(jrkParameter));

            int i;
            for (i = 0; i < names.Length; i++)
            {
                string parameter_name = names[i];
                byte parameter = values[i];
                uint value = parameterSource.getJrkParameter((jrkParameter)parameter);
                string value_string;
                switch(parameter)
                {
                    case (byte)jrkParameter.PARAMETER_FEEDBACK_MODE:
                        value_string = Enum.GetName(typeof(jrkFeedbackMode), value).Substring("FEEDBACK_MODE_".Length);
                        break;
                    case (byte)jrkParameter.PARAMETER_INPUT_MODE:
                        value_string = Enum.GetName(typeof(jrkInputMode), value).Substring("INPUT_MODE_".Length);
                        break;
                    case (byte)jrkParameter.PARAMETER_SERIAL_MODE:
                        value_string = Enum.GetName(typeof(jrkSerialMode), value).Substring("SERIAL_MODE_".Length);
                        break;
                    default:
                        value_string = value.ToString();
                        break;
                }

                // remove PARAMETER_, since it is always there (of course, if it isn't, this will create an unreadable file!)
                if (parameter_name.StartsWith("PARAMETER_"))
                    parameter_name = parameter_name.Substring(10);

                // write the name and value on a line
                sw.WriteLine(parameter_name+"\t"+value_string);
            }
        }
    }
}
