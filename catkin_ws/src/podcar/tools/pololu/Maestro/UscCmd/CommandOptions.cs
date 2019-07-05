using System;
using System.Collections.Generic;
using System.Text.RegularExpressions;
using System.Text;

namespace Pololu.Usc.UscCmd
{
    class CommandOptions
    {
        private string helpMessage;

        private Dictionary<string, string> privateArgs = new Dictionary<string, string>();
        public CommandOptions(string help_message, string[] args)
        {
            string name = "";
            helpMessage = help_message;
            foreach (string arg in args)
            {
                Match m = Regex.Match(arg, "^--(.*)");
                if (m.Success)
                {
                    name = m.Groups[1].ToString();
                    privateArgs[name] = ""; // start it off with no string value
                    continue;
                }

                // got a string value for the last arg
                if (name == "")
                    error();

                privateArgs[name] = arg;
            }

            if (privateArgs.Count == 0)
                error();
        }

        public void error()
        {
            Console.Error.WriteLine(helpMessage);
            Environment.Exit(1);
        }

        public void error(string message)
        {
            Console.Error.WriteLine(message);
            Console.Error.WriteLine(helpMessage);
            Environment.Exit(1);
        }

        /// <summary>
        /// Returns the value of an argument, which is "" for arguments with no supplied parameter, or null if the argument was not supplied.
        /// </summary>
        /// <param name="index"></param>
        /// <returns></returns>
        public string this[string index]
        {
            get
            {
                if (privateArgs.ContainsKey(index))
                    return privateArgs[index];
                return null;
            }
        }

        /// <summary>
        /// returns the number of arguments
        /// </summary>
        /// <returns></returns>
        public int Count
        {
            get
            {
                return privateArgs.Count;
            }
        }
    }

}
