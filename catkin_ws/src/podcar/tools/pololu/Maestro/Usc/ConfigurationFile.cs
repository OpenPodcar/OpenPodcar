using System;
using System.Collections.Generic;
using Pololu.Usc;
using System.Text;
using System.IO;
using System.Xml;
using Pololu.Usc.Sequencer;

namespace Pololu.Usc
{
    /// <summary>
    /// A static class with methods for reading and writing Maestro
    /// configuration files.
    /// </summary>
    public static class ConfigurationFile
    {
        /// <summary>
        /// Parses a saved configuration file and returns a UscSettings object.
        /// </summary>
        /// <param name="warnings">A list of warnings.  Whenever something goes
        /// wrong with the file loading, a warning will be added to this list.
        /// The warnings are not fatal; if the function returns it will return
        /// a valid UscSettings object.
        /// </param>
        /// <param name="sr">The file to read from.</param>
        /// <remarks>This function is messy.  Maybe I should have tried the XPath
        /// library.</remarks>
        public static UscSettings load(StreamReader sr, List<String> warnings)
        {
            XmlReader reader = XmlReader.Create(sr);

            UscSettings settings = new UscSettings();

            string script = "";

            // The x prefix means "came directly from XML"
            Dictionary<String, String> xParams = new Dictionary<string, string>();

            // Only read the data inside the UscSettings element.
            reader.ReadToFollowing("UscSettings");
            readAttributes(reader, xParams);
            reader = reader.ReadSubtree();

            // Check the version number
            if (!xParams.ContainsKey("version"))
            {
                warnings.Add("This file has no version number, so it might have been read incorrectly.");   
            }
            else if (xParams["version"] != "1")
            {
                warnings.Add("Unrecognized settings file version \"" + xParams["version"] + "\".");
            }

            reader.Read(); // this is needed, otherwise the first tag inside uscSettings doesn't work work (not sure why)
           
            while (reader.Read())
            {
                if (reader.NodeType == XmlNodeType.Element && reader.Name == "Channels")
                {
                    // We found the Channels tag.

                    // Read the ServosAvailable and ServoPeriod attributes from it in to our collection.
                    readAttributes(reader, xParams);

                    // Make a reader that can only read the stuff inside the Channels tag.
                    var channelsReader = reader.ReadSubtree();

                    // For each Channel tag...
                    while(channelsReader.ReadToFollowing("Channel"))
                    {
                        // Read all the attributes.
                        Dictionary<String, String> xChannel = new Dictionary<string, string>();
                        readAttributes(channelsReader, xChannel);

                        // Transform the attributes in to a ChannelSetting object.
                        ChannelSetting cs = new ChannelSetting();
                        if (assertKey("name", xChannel, warnings))
                        {
                            cs.name = xChannel["name"];
                        }

                        if (assertKey("mode", xChannel, warnings))
                        {
                            switch (xChannel["mode"].ToLowerInvariant())
                            {
                                case "servomultiplied": cs.mode = ChannelMode.ServoMultiplied; break;
                                case "servo": cs.mode = ChannelMode.Servo; break;
                                case "input": cs.mode = ChannelMode.Input; break;
                                case "output": cs.mode = ChannelMode.Output; break;
                                default: warnings.Add("Invalid mode \"" + xChannel["mode"] + "\"."); break;
                            }
                        }

                        if (assertKey("homemode", xChannel, warnings))
                        {
                            switch (xChannel["homemode"].ToLowerInvariant())
                            {
                                case "goto": cs.homeMode = HomeMode.Goto; break;
                                case "off": cs.homeMode = HomeMode.Off; break;
                                case "ignore": cs.homeMode = HomeMode.Ignore; break;
                                default: warnings.Add("Invalid homemode \"" + xChannel["homemode"] + "\"."); break;
                            }
                        }

                        if (assertKey("min", xChannel, warnings)) { parseU16(xChannel["min"], ref cs.minimum, "min", warnings); }
                        if (assertKey("max", xChannel, warnings)) { parseU16(xChannel["max"], ref cs.maximum, "max", warnings); }
                        if (assertKey("home", xChannel, warnings)) { parseU16(xChannel["home"], ref cs.home, "home", warnings); }
                        if (assertKey("speed", xChannel, warnings)) { parseU16(xChannel["speed"], ref cs.speed, "speed", warnings); }
                        if (assertKey("acceleration", xChannel, warnings)) { parseU8(xChannel["acceleration"], ref cs.acceleration, "acceleration", warnings); }
                        if (assertKey("neutral", xChannel, warnings)) { parseU16(xChannel["neutral"], ref cs.neutral, "neutral", warnings); }
                        if (assertKey("range", xChannel, warnings)) { parseU16(xChannel["range"], ref cs.range, "range", warnings); }

                        settings.channelSettings.Add(cs);
                    }

                    if (channelsReader.ReadToFollowing("Channel"))
                    {
                        warnings.Add("More than " + settings.servoCount + " channel elements were found.  The extra elements have been discarded.");
                    }

                }
                else if (reader.NodeType == XmlNodeType.Element && reader.Name == "Sequences")
                {
                    // We found the Sequences tag.

                    // For each Sequence tag in this sequence...
                    var sequencesReader = reader.ReadSubtree();
                    while (sequencesReader.ReadToFollowing("Sequence"))
                    {
                        // Create a new sequence.
                        Sequence sequence = new Sequence();
                        settings.sequences.Add(sequence);

                        // Read the sequence tag attributes (should just be "name").
                        Dictionary<String, String> sequenceAttributes = new Dictionary<string, string>();
                        readAttributes(sequencesReader, sequenceAttributes);

                        if (sequenceAttributes.ContainsKey("name"))
                        {
                            sequence.name = sequenceAttributes["name"];
                        }
                        else
                        {
                            sequence.name = "Sequence " + settings.sequences.Count;
                            warnings.Add("No name found for sequence " + sequence.name + ".");
                        }

                        // For each frame tag in this sequence...
                        var framesReader = reader.ReadSubtree();
                        while (framesReader.ReadToFollowing("Frame"))
                        {
                            // Create a new frame.
                            Frame frame = new Frame();
                            sequence.frames.Add(frame);

                            // Read the frame attributes from XML (name, duration)
                            Dictionary<String, String> frameAttributes = new Dictionary<string, string>();
                            readAttributes(framesReader, frameAttributes);

                            if (frameAttributes.ContainsKey("name"))
                            {
                                frame.name = frameAttributes["name"];
                            }
                            else
                            {
                                frame.name = "Frame " + sequence.frames.Count;
                                warnings.Add("No name found for " + frame.name + " in sequence \"" + sequence.name + "\".");
                            }

                            if (frameAttributes.ContainsKey("duration"))
                            {
                                parseU16(frameAttributes["duration"], ref frame.length_ms,
                                    "Duration for frame \"" + frame.name + "\" in sequence \"" + sequence.name + "\".", warnings);
                            }
                            else
                            {
                                frame.name = "Frame " + sequence.frames.Count;
                                warnings.Add("No duration found for frame \"" + frame.name + "\" in sequence \"" + sequence.name + "\".");
                            }

                            frame.setTargetsFromString(reader.ReadElementContentAsString(), settings.servoCount);
                        }
                    }
                }
                else if (reader.NodeType == XmlNodeType.Element && reader.Name == "Script")
                {
                    // We found the <Script> tag.

                    // Get the ScriptDone attribute in to our dictionary.
                    readAttributes(reader, xParams);

                    // Read the script.
                    script = reader.ReadElementContentAsString();
                }
                else if (reader.NodeType == XmlNodeType.Element)
                {
                    // Read the miscellaneous parameters that come in element tags, like <NeverSuspend>false</NeverSuspend>.
                    try
                    {
                        xParams[reader.Name] = reader.ReadElementContentAsString();
                    }
                    catch (XmlException e)
                    {
                        warnings.Add("Unable to parse element \"" + reader.Name + "\": " + e.Message);
                    }
                }
            }
            reader.Close();
            
            //// Step 2: Put the data in to the settings object.

            try
            {
                settings.setAndCompileScript(script);
            }
            catch (Exception e)
            {
                warnings.Add("Error compiling script from XML file: " + e.Message);
                settings.scriptInconsistent = true;
            }

            if (assertKey("NeverSuspend", xParams, warnings))
            {
                parseBool(xParams["NeverSuspend"], ref settings.neverSuspend, "NeverSuspend", warnings);
            }

            if (assertKey("SerialMode", xParams, warnings))
            {
                switch (xParams["SerialMode"])
                {
                    default: settings.serialMode = uscSerialMode.SERIAL_MODE_UART_DETECT_BAUD_RATE; break;
                    case "UART_FIXED_BAUD_RATE": settings.serialMode = uscSerialMode.SERIAL_MODE_UART_FIXED_BAUD_RATE; break;
                    case "USB_DUAL_PORT": settings.serialMode = uscSerialMode.SERIAL_MODE_USB_DUAL_PORT; break;
                    case "USB_CHAINED": settings.serialMode = uscSerialMode.SERIAL_MODE_USB_CHAINED; break;
                }
            }
            
            if (assertKey("FixedBaudRate", xParams, warnings))
            {
                parseU32(xParams["FixedBaudRate"], ref settings.fixedBaudRate, "FixedBaudRate", warnings);
            }

            if (assertKey("SerialTimeout", xParams, warnings))
            {
                parseU16(xParams["SerialTimeout"], ref settings.serialTimeout, "SerialTimeout", warnings);
            }

            if (assertKey("EnableCrc", xParams, warnings))
            {
                parseBool(xParams["EnableCrc"], ref settings.enableCrc, "EnableCrc", warnings);
            }

            if (assertKey("SerialDeviceNumber", xParams, warnings))
            {
                parseU8(xParams["SerialDeviceNumber"], ref settings.serialDeviceNumber, "SerialDeviceNumber", warnings);
            }

            if (assertKey("SerialMiniSscOffset", xParams, warnings))
            {
                parseU8(xParams["SerialMiniSscOffset"], ref settings.miniSscOffset, "SerialMiniSscOffset", warnings);
            }

            if (assertKey("ScriptDone", xParams, warnings))
            {
                parseBool(xParams["ScriptDone"], ref settings.scriptDone, "ScriptDone", warnings);
            }

            // These parameters are optional because they don't apply to all Maestros.
            if (xParams.ContainsKey("ServosAvailable"))
            {
                parseU8(xParams["ServosAvailable"], ref settings.servosAvailable, "ServosAvailable", warnings);
            }

            if (xParams.ContainsKey("ServoPeriod"))
            {
                parseU8(xParams["ServoPeriod"], ref settings.servoPeriod, "ServoPeriod", warnings);
            }

            if (xParams.ContainsKey("EnablePullups"))
            {
                parseBool(xParams["EnablePullups"], ref settings.enablePullups, "EnablePullups", warnings);
            }

            if (xParams.ContainsKey("MiniMaestroServoPeriod"))
            {
                parseU32(xParams["MiniMaestroServoPeriod"], ref settings.miniMaestroServoPeriod, "MiniMaestroServoPeriod", warnings);
            }

            if (xParams.ContainsKey("ServoMultiplier"))
            {
                parseU16(xParams["ServoMultiplier"], ref settings.servoMultiplier, "ServoMultiplier", warnings);
            }

            return settings;
        }

        /// <summary>
        /// If the XmlReader is at an element that has attributes, this will read all those
        /// attributes in to the dictionary.
        /// </summary>
        private static void readAttributes(XmlReader reader, Dictionary<String, String> attributes)
        {
            if (reader.HasAttributes)
            {
                while (reader.MoveToNextAttribute())
                {
                    attributes[reader.Name] = reader.ReadContentAsString();
                }
            }

            // Move back to the element (so we can call ReadSubtree if needed)
            reader.MoveToElement();
        }

        private static void parseBool(string input, ref Boolean output, string name, List<string> warnings)
        {
            Boolean result;
            if (Boolean.TryParse(input, out result))
            {
                output = result;
            }
            else
            {
                warnings.Add(name + ": Invalid integer value \"" + input + "\".");
            }
        }

        private static void parseU8(string input, ref Byte output, string name, List<string> warnings)
        {
            Byte result;
            if (Byte.TryParse(input, out result))
            {
                output = result;
            }
            else
            {
                warnings.Add(name + ": Invalid integer value \"" + input + "\".");
            }
        }

        private static void parseU16(string input, ref UInt16 output, string name, List<string> warnings)
        {
            UInt16 result;
            if (UInt16.TryParse(input, out result))
            {
                output = result;
            }
            else
            {
                warnings.Add(name + ": Invalid integer value \"" + input + "\".");
            }
        }

        private static void parseU32(string input, ref UInt32 output, string name, List<string> warnings)
        {
            UInt32 result;
            if (UInt32.TryParse(input, out result))
            {
                output = result;
            }
            else
            {
                warnings.Add(name + ": Invalid integer value \"" + input + "\".");
            }
        }

        private static bool assertKey(string key, Dictionary<string, string> params_from_xml, List<string> warnings)
        {
            if (params_from_xml.ContainsKey(key))
            {
                return true;
            }
            else
            {
                warnings.Add("The " + key + " setting was missing.");
                return false;
            }
        }

        /// <summary>
        /// Saves a UscSettings object to a textfile.
        /// </summary>
        /// <param name="settings">The settings to read from.</param>
        /// <param name="sw">The file to write to.</param>
        public static void save(UscSettings settings, StreamWriter sw)
        {
            XmlTextWriter writer = new XmlTextWriter(sw);
            writer.Formatting = Formatting.Indented;
            writer.WriteComment("Pololu Maestro servo controller settings file, http://www.pololu.com/catalog/product/1350");
            writer.WriteStartElement("UscSettings");
            writer.WriteAttributeString("version", "1"); // XML file version, so that we can parse old XML types in the future
            writer.WriteElementString("NeverSuspend", settings.neverSuspend ? "true" : "false");
            writer.WriteElementString("SerialMode", settings.serialMode.ToString().Replace("SERIAL_MODE_",""));
            writer.WriteElementString("FixedBaudRate", settings.fixedBaudRate.ToString());
            writer.WriteElementString("SerialTimeout", settings.serialTimeout.ToString());
            writer.WriteElementString("EnableCrc", settings.enableCrc ? "true" : "false");
            writer.WriteElementString("SerialDeviceNumber", settings.serialDeviceNumber.ToString());
            writer.WriteElementString("SerialMiniSscOffset", settings.miniSscOffset.ToString());

            if (settings.servoCount > 18)
            {
                writer.WriteElementString("EnablePullups", settings.enablePullups ? "true" : "false");
            }

            writer.WriteStartElement("Channels");

            // Attributes of the Channels tag
            if (settings.servoCount == 6)
            {
                writer.WriteAttributeString("ServosAvailable", settings.servosAvailable.ToString());
                writer.WriteAttributeString("ServoPeriod", settings.servoPeriod.ToString());
            }
            else
            {
                writer.WriteAttributeString("MiniMaestroServoPeriod", settings.miniMaestroServoPeriod.ToString());
                writer.WriteAttributeString("ServoMultiplier", settings.servoMultiplier.ToString());
            }
            writer.WriteComment("Period = " + (settings.periodInMicroseconds / 1000M).ToString() + " ms");

            for (byte i = 0; i < settings.servoCount; i++)
            {
                ChannelSetting setting = settings.channelSettings[i];
                writer.WriteComment("Channel " + i.ToString());
                writer.WriteStartElement("Channel");
                writer.WriteAttributeString("name", setting.name);
                writer.WriteAttributeString("mode", setting.mode.ToString());
                writer.WriteAttributeString("min", setting.minimum.ToString());
                writer.WriteAttributeString("max", setting.maximum.ToString());
                writer.WriteAttributeString("homemode", setting.homeMode.ToString());
                writer.WriteAttributeString("home", setting.home.ToString());
                writer.WriteAttributeString("speed", setting.speed.ToString());
                writer.WriteAttributeString("acceleration", setting.acceleration.ToString());
                writer.WriteAttributeString("neutral", setting.neutral.ToString());
                writer.WriteAttributeString("range", setting.range.ToString());
                writer.WriteEndElement();
            }
            writer.WriteEndElement();

            writer.WriteStartElement("Sequences");
            foreach (Sequence sequence in settings.sequences)
            {
                writer.WriteStartElement("Sequence");
                writer.WriteAttributeString("name", sequence.name);
                foreach (Frame frame in sequence.frames)
                {
                    frame.writeXml(writer);
                }
                writer.WriteEndElement();
            }
            writer.WriteEndElement(); // end sequences


            writer.WriteStartElement("Script");
            writer.WriteAttributeString("ScriptDone", settings.scriptDone ? "true" : "false");
            writer.WriteString(settings.script);
            writer.WriteEndElement(); // end script

            writer.WriteEndElement(); // End UscSettings tag.
        }
    }
}