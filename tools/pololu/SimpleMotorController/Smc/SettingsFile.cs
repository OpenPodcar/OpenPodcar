using System;
using System.Collections.Generic;
using System.IO;
using System.Xml;

namespace Pololu.SimpleMotorController
{
    /// <summary>
    /// A static class with methods for reading and writing configuration files.
    /// </summary>
    public static class SettingsFile
    {
        /// <summary>
        /// XML tags that are expected to have children.
        /// </summary>
        static List<String> complexTags = new List<String>{"Rc1", "Rc2", "Analog1", "Analog2", "Error", "Input", "InputNeutral", "OverTemp", "ForwardLimits", "ReverseLimits", "PulsePeriod"};

        private class Tag
        {
            public Dictionary<String, String> values = new Dictionary<String, String>();
            public Dictionary<String, Tag> children = new Dictionary<String, Tag>();

            public static Tag readFrom(XmlReader reader, List<String> warnings)
            {
                reader = reader.ReadSubtree();
                reader.Read();

                Tag t = new Tag();
                if (reader.HasAttributes)
                {
                    while (reader.MoveToNextAttribute())
                    {
                        t.values[reader.Name] = reader.ReadContentAsString();
                    }
                }

                while (reader.Read())
                {
                    if (reader.NodeType == XmlNodeType.Element)
                    {
                        if (SettingsFile.complexTags.Contains(reader.Name))
                        {
                            if (t.children.ContainsKey(reader.Name))
                            {
                                warnings.Add("More than one '" + reader.Name + "' tags are present within the same tag.  All but the last will be ignored.");
                            }

                            try
                            {
                                t.children.Add(reader.Name, Tag.readFrom(reader, warnings));
                            }
                            catch (Exception exception)
                            {
                                warnings.Add("Error reading '" + reader.Name + "' tag: " + exception.Message + ".");
                            }
                        }
                        else
                        {
                            if (t.values.ContainsKey(reader.Name))
                            {
                                warnings.Add("More than one '" + reader.Name + "' attribute/tag is present within the same tag.  All but the last will be ignored.");
                            }

                            try
                            {
                                t.values.Add(reader.Name, reader.ReadElementContentAsString());
                            }
                            catch(XmlException exception)
                            {
                                warnings.Add("Error reading '" + reader.Name + "' value: " + exception.Message + ".");
                            }
                        }
                    }
                }
                return t;
            }
        }

        /// <summary>
        /// Parses a saved configuration file and returns a SmcSettings object.
        /// </summary>
        /// <param name="warnings">A list of warnings.  Whenever something goes
        /// wrong with the file loading, a warning will be added to this list.
        /// The warnings are not fatal; if the function returns it will return
        /// a valid SmcSettings object.
        /// </param>
        /// <param name="filename">The file to read from.</param>
        /// <param name="productId">The product id of the device we are loading these
        /// settings for.  Pass 0 if it is unknown at this time.</param>
        public static SmcSettings load(String filename, List<String> warnings, UInt16 productId)
        {
            using (FileStream settingsFile = File.Open(filename, FileMode.Open))
            {
                using (StreamReader sr = new StreamReader(settingsFile))
                {
                    return SettingsFile.load(sr, warnings, productId);
                }
            }
        }

        /// <summary>
        /// Parses a saved configuration file and returns a SmcSettings object.
        /// </summary>
        /// <param name="warnings">A list of warnings.  Whenever something goes
        /// wrong with the file loading, a warning will be added to this list.
        /// The warnings are not fatal; if the function returns it will return
        /// a valid SmcSettings object.
        /// </param>
        /// <param name="sr">The file to read from.</param>
        /// <param name="productId">The product id of the device we are loading these
        /// settings for.  Pass 0 if it is unknown at this time.</param>
        public static SmcSettings load(StreamReader sr, List<String> warnings, UInt16 productId)
        {
            XmlReader reader = XmlReader.Create(sr);
            reader.ReadToFollowing("SmcSettings");
            Tag settingsTag = Tag.readFrom(reader, warnings);
            reader.Close();

            SmcSettings settings = new SmcSettings(productId);

            // [Add-new-settings-here]

            if (assertValue(settingsTag, "InputMode", warnings))
            {
                try
                {
                    settings.inputMode = (SmcInputMode)Enum.Parse(typeof(SmcInputMode), settingsTag.values["InputMode"]);
                }
                catch
                {
                    warnings.Add("Invalid InputMode value \"" + settingsTag.values["InputMode"] + "\".");
                }
            }

            if (assertValue(settingsTag, "MixingMode", warnings))
            {
                try
                {
                    settings.mixingMode = (SmcMixingMode)Enum.Parse(typeof(SmcMixingMode), settingsTag.values["MixingMode"]);
                }
                catch
                {
                    warnings.Add("Invalid MixingMode value \"" + settingsTag.values["MixingMode"] + "\".");
                }
            }

            parseBool(settingsTag, "DisableSafeStart", ref settings.disableSafeStart, warnings);
            parseBool(settingsTag, "IgnorePotDisconnect", ref settings.ignorePotDisconnect, warnings);
            parseBool(settingsTag, "IgnoreErrLineHigh", ref settings.ignoreErrLineHigh, warnings);
            parseBool(settingsTag, "NeverSuspend", ref settings.neverSuspend, warnings);
            parseBool(settingsTag, "TempLimitGradual", ref settings.tempLimitGradual, warnings);
            parseRange(settingsTag, "OverTemp", ref settings.overTempMin, ref settings.overTempMax, warnings);
            parseU16(settingsTag, "LowVinShutoffTimeout", ref settings.lowVinShutoffTimeout, warnings);
            parseU16(settingsTag, "LowVinShutoffMv", ref settings.lowVinShutoffMv, warnings);
            parseU16(settingsTag, "LowVinStartupMv", ref settings.lowVinStartupMv, warnings);
            parseU16(settingsTag, "HighVinShutoffMv", ref settings.highVinShutoffMv, warnings);

            if (assertValue(settingsTag, "SerialMode", warnings))
            {
                try
                {
                    settings.serialMode = (SmcSerialMode)Enum.Parse(typeof(SmcSerialMode), settingsTag.values["SerialMode"]);
                }
                catch
                {
                    warnings.Add("Invalid SmcSerialMode value \"" + settingsTag.values["SerialMode"] + "\".");
                }
            }

            parseU8(settingsTag, "SerialDeviceNumber", ref settings.serialDeviceNumber, warnings);
            parseU16(settingsTag, "CommandTimeout", ref settings.commandTimeout, warnings);

            if (assertValue(settingsTag, "CrcMode", warnings))
            {
                try
                {
                    settings.crcMode = (SmcCrcMode)Enum.Parse(typeof(SmcCrcMode), settingsTag.values["CrcMode"]);
                }
                catch
                {
                    warnings.Add("Invalid CrcMode value \"" + settingsTag.values["CrcMode"] + "\".");
                }
            }

            parseBool(settingsTag, "UartResponseDelay", ref settings.uartResponseDelay, warnings);
            parseBool(settingsTag, "UseFixedBaudRate", ref settings.useFixedBaudRate, warnings);
            parseU32(settingsTag, "FixedBaudRate", ref settings.fixedBaudRateBps, warnings);

            parseChannel(settingsTag, SmcChannel.Rc1, "Rc1", settings.rc1, warnings);
            parseChannel(settingsTag, SmcChannel.Rc2, "Rc2", settings.rc2, warnings);
            parseChannel(settingsTag, SmcChannel.Analog1, "Analog1", settings.analog1, warnings);
            parseChannel(settingsTag, SmcChannel.Analog2, "Analog2", settings.analog2, warnings);

            parseU8(settingsTag, "PwmPeriodFactor", ref settings.pwmPeriodFactor, warnings);
            parseBool(settingsTag, "MotorInvert", ref settings.motorInvert, warnings);
            parseU8(settingsTag, "SpeedZeroBrakeAmount", ref settings.speedZeroBrakeAmount, warnings);
            parseU16(settingsTag, "SpeedUpdatePeriod", ref settings.speedUpdatePeriod, warnings);

            parseLimits(settingsTag, "ForwardLimits", settings.forwardLimits, warnings);
            parseLimits(settingsTag, "ReverseLimits", settings.reverseLimits, warnings);

            parseRange(settingsTag, "PulsePeriod", ref settings.minPulsePeriod, ref settings.maxPulsePeriod, warnings);
            parseU16(settingsTag, "RcTimeout", ref settings.rcTimeout, warnings);
            parseU8(settingsTag, "ConsecGoodPulses", ref settings.consecGoodPulses, warnings);
            parseS16(settingsTag, "VinMultiplierOffset", ref settings.vinMultiplierOffset, warnings);

            return settings;
        }

        private static bool assertValue(Tag tag, string name, List<string> warnings)
        {
            if (tag.values.ContainsKey(name))
            {
                return true;
            }
            else
            {
                warnings.Add("The " + name + " setting was missing.");
                return false;
            }
        }

        private static bool assertChild(Tag tag, string name, List<string> warnings)
        {
            if (tag.children.ContainsKey(name))
            {
                return true;
            }
            else
            {
                warnings.Add("The " + name + " tag was missing.");
                return false;
            }
        }

        private static void parseRange(Tag tag, string name, ref UInt16 min, ref UInt16 max, List<string> warnings)
        {
            if (assertChild(tag, name, warnings))
            {
                Tag rangeTag = tag.children[name];
                parseU16(rangeTag, "Min", ref min, warnings);
                parseU16(rangeTag, "Max", ref max, warnings);
            }
        }

        private static void parseChannel(Tag tag, SmcChannel channel, string name, SmcChannelSettings cs, List<string> warnings)
        {
            if (assertChild(tag, name, warnings))
            {
                Tag channelTag = tag.children[name];
                parseBool(channelTag, "Invert", ref cs.invert, warnings);
                if (assertValue(channelTag, "AlternateUse", warnings))
                {
                    try
                    {
                        cs.alternateUse = (SmcChannelAlternateUse)Enum.Parse(typeof(SmcChannelAlternateUse), channelTag.values["AlternateUse"]);
                    }
                    catch
                    {
                        warnings.Add("Invalid AlternateUse value \"" + channelTag.values["AlternateUse"] + "\".");
                    }
                }
                if (channel.type() == SmcChannelType.Analog && assertValue(channelTag, "PinMode", warnings))
                {
                    try
                    {
                        cs.pinMode = (SmcPinMode)Enum.Parse(typeof(SmcPinMode), channelTag.values["PinMode"]);
                    }
                    catch
                    {
                        warnings.Add("Invalid PinMode value \"" + channelTag.values["PinMode"] + "\".");
                    }
                }
                parseU8(channelTag, "ScalingDegree", ref cs.scalingDegree, warnings);
                parseRange(channelTag, "Error", ref cs.errorMin, ref cs.errorMax, warnings);
                parseRange(channelTag, "Input", ref cs.inputMin, ref cs.inputMax, warnings);
                parseRange(channelTag, "InputNeutral", ref cs.inputNeutralMin, ref cs.inputNeutralMax, warnings);
            }
        }

        private static void parseLimits(Tag tag, string name, SmcMotorLimits ml, List<string> warnings)
        {
            if (assertChild(tag, name, warnings))
            {
                Tag limitsTag = tag.children[name];
                parseU16(limitsTag, "MaxSpeed", ref ml.maxSpeed, warnings);
                parseU16(limitsTag, "MaxAcceleration", ref ml.maxAcceleration, warnings);
                parseU16(limitsTag, "MaxDeceleration", ref ml.maxDeceleration, warnings);
                parseU16(limitsTag, "BrakeDuration", ref ml.brakeDuration, warnings);
                parseU16(limitsTag, "StartingSpeed", ref ml.startingSpeed, warnings);
            }
        }

        private static void parseBool(Tag tag, string name, ref Boolean output, List<string> warnings)
        {
            if (assertValue(tag, name, warnings))
            {
                Boolean result;
                if (Boolean.TryParse(tag.values[name], out result))
                {
                    output = result;
                }
                else
                {
                    warnings.Add(name + ": Invalid boolean value \"" + tag.values[name] + "\".");
                }
            }
        }

        private static void parseU8(Tag tag, string name, ref Byte output, List<string> warnings)
        {
            if (assertValue(tag, name, warnings))
            {
                Byte result;
                if (Byte.TryParse(tag.values[name], out result))
                {
                    output = result;
                }
                else
                {
                    warnings.Add(name + ": Invalid integer value \"" + tag.values[name] + "\".");
                }
            }
        }

        private static void parseU16(Tag tag, string name, ref UInt16 output, List<string> warnings)
        {
            if (assertValue(tag, name, warnings))
            {
                UInt16 result;
                if (UInt16.TryParse(tag.values[name], out result))
                {
                    output = result;
                }
                else
                {
                    warnings.Add(name + ": Invalid integer value \"" + tag.values[name] + "\".");
                }
            }
        }

        private static void parseS16(Tag tag, string name, ref Int16 output, List<string> warnings)
        {
            if (assertValue(tag, name, warnings))
            {
                Int16 result;
                if (Int16.TryParse(tag.values[name], out result))
                {
                    output = result;
                }
                else
                {
                    warnings.Add(name + ": Invalid integer value \"" + tag.values[name] + "\".");
                }
            }
        }

        private static void parseU32(Tag tag, string name, ref UInt32 output, List<string> warnings)
        {
            if (assertValue(tag, name, warnings))
            {
                UInt32 result;
                if (UInt32.TryParse(tag.values[name], out result))
                {
                    output = result;
                }
                else
                {
                    warnings.Add(name + ": Invalid integer value \"" + tag.values[name] + "\".");
                }
            }
        }

        /// <summary>
        /// Adds an element that holds a boolean value.
        /// For example: "<foo>true</foo>".
        /// </summary>
        private static void WriteElementBool(this XmlWriter writer, String localName, Boolean value)
        {
            writer.WriteElementString(localName, value ? "true" : "false");
        }

        private static void WriteElementU32(this XmlWriter writer, String localName, UInt32 value)
        {
            writer.WriteElementString(localName, value.ToString());
        }

        /// <summary>
        /// Adds an element that represents a range.
        /// For example: '<foo min="234" max="5435"/>'
        /// </summary>
        private static void WriteElementRange(this XmlWriter writer, String localName, UInt32 min, UInt32 max)
        {
            writer.WriteStartElement(localName);
            writer.WriteAttributeString("Min", min.ToString());
            writer.WriteAttributeString("Max", max.ToString());
            writer.WriteEndElement();
        }

        private static void WriteElementS32(this XmlWriter writer, String localName, Int32 value)
        {
            writer.WriteElementString(localName, value.ToString());
        }


        /// <summary>
        /// Writes a SmcSettings object as text to the specified streamwriter.
        /// </summary>
        /// <param name="settings">The settings to read from.</param>
        /// <param name="filename">The file to write to.</param>
        /// <param name="device">The device that these settings came from (optional!).  Adds extra comments to the file.</param>
        public static void save(SmcSettings settings, string filename, Smc device)
        {
            using (FileStream settingsFile = File.Open(filename, FileMode.Create))
            {
                using (StreamWriter sw = new StreamWriter(settingsFile))
                {
                    save(settings, sw, device);
                }
            }
        }

        /// <summary>
        /// Writes a SmcSettings object as text to the specified streamwriter.
        /// </summary>
        /// <param name="settings">The settings to read from.</param>
        /// <param name="sw">The file to write to.</param>
        /// <param name="device">The device that these settings came from (optional!).  Adds extra comments to the file.</param>
        public static void save(SmcSettings settings, StreamWriter sw, Smc device)
        {
            XmlTextWriter writer = new XmlTextWriter(sw);
            writer.Formatting = Formatting.Indented;
            writer.WriteComment("Pololu Simple Motor Controller settings file. http://www.pololu.com/docs/0J44");
            writer.WriteComment("Created on: " + DateTime.Now.ToString("u").TrimEnd('Z')); // save local time in a sortable format
            if (device != null)
            {
                writer.WriteComment("Device model: " + Smc.productIdToLongModelString(device.productId));
                writer.WriteComment("Device serial number: " + device.getSerialNumber());
                writer.WriteComment("Device firmware version: " + device.getFirmwareVersionString());
            }
            writer.WriteStartElement("SmcSettings");

            // XML file version, so that we can parse old XML types in the future and old versions of the
            // configuration utility don't try to read newer settings files that have a different format.
            writer.WriteAttributeString("version", "1");

            // [Add-new-settings-here]

            writer.WriteElementString("InputMode", settings.inputMode.ToString());
            writer.WriteElementString("MixingMode", settings.mixingMode.ToString());

            writer.WriteElementBool("DisableSafeStart", settings.disableSafeStart);
            writer.WriteElementBool("IgnorePotDisconnect", settings.ignorePotDisconnect);
            writer.WriteElementBool("IgnoreErrLineHigh", settings.ignoreErrLineHigh);
            writer.WriteElementBool("NeverSuspend", settings.neverSuspend);

            writer.WriteElementBool("TempLimitGradual", settings.tempLimitGradual);
            writer.WriteElementRange("OverTemp", settings.overTempMin, settings.overTempMax);
            writer.WriteElementU32("LowVinShutoffTimeout", settings.lowVinShutoffTimeout);
            writer.WriteElementU32("LowVinShutoffMv", settings.lowVinShutoffMv);
            writer.WriteElementU32("LowVinStartupMv", settings.lowVinStartupMv);
            writer.WriteElementU32("HighVinShutoffMv", settings.highVinShutoffMv);

            writer.WriteElementString("SerialMode", settings.serialMode.ToString()); 
            writer.WriteElementU32("SerialDeviceNumber", settings.serialDeviceNumber);
            writer.WriteElementU32("CommandTimeout", settings.commandTimeout);
            writer.WriteElementString("CrcMode", settings.crcMode.ToString());
            writer.WriteElementBool("UartResponseDelay", settings.uartResponseDelay);
            writer.WriteElementBool("UseFixedBaudRate", settings.useFixedBaudRate);
            writer.WriteElementU32("FixedBaudRate", settings.fixedBaudRateBps);

            writer.WriteComment("Input Settings");
            writer.WriteElementChannelSettings(SmcChannel.Rc1, settings.rc1);
            writer.WriteElementChannelSettings(SmcChannel.Rc2, settings.rc2);
            writer.WriteElementChannelSettings(SmcChannel.Analog1, settings.analog1);
            writer.WriteElementChannelSettings(SmcChannel.Analog2, settings.analog2);

            writer.WriteComment("Motor Settings");
            writer.WriteElementU32("PwmPeriodFactor", settings.pwmPeriodFactor);
            writer.WriteElementBool("MotorInvert", settings.motorInvert);
            writer.WriteElementU32("SpeedZeroBrakeAmount", settings.speedZeroBrakeAmount);
            writer.WriteElementU32("SpeedUpdatePeriod", settings.speedUpdatePeriod);
            writer.WriteElementMotorLimits("ForwardLimits", settings.forwardLimits);
            writer.WriteElementMotorLimits("ReverseLimits", settings.reverseLimits);

            writer.WriteComment("Advanced Settings");
            writer.WriteElementRange("PulsePeriod", settings.minPulsePeriod, settings.maxPulsePeriod);
            writer.WriteElementU32("RcTimeout", settings.rcTimeout);
            writer.WriteElementU32("ConsecGoodPulses", settings.consecGoodPulses);
            writer.WriteElementS32("VinMultiplierOffset", settings.vinMultiplierOffset);

            writer.WriteEndElement(); // End SmcSettings tag.
        }

        private static void WriteElementChannelSettings(this XmlWriter writer, SmcChannel channel, SmcChannelSettings cs)
        {
            writer.WriteStartElement(channel.ToString());
            writer.WriteElementString("AlternateUse", cs.alternateUse.ToString());
            if (channel.type() == SmcChannelType.Analog)
            {
                writer.WriteElementString("PinMode", cs.pinMode.ToString());
            }
            writer.WriteElementBool("Invert", cs.invert);
            writer.WriteElementU32("ScalingDegree", cs.scalingDegree);
            writer.WriteElementRange("Error", cs.errorMin, cs.errorMax);
            writer.WriteElementRange("Input", cs.inputMin, cs.inputMax);
            writer.WriteElementRange("InputNeutral", cs.inputNeutralMin, cs.inputNeutralMax);
            writer.WriteEndElement();
        }

        private static void WriteElementMotorLimits(this XmlWriter writer, String localName, SmcMotorLimits ml)
        {
            writer.WriteStartElement(localName);
            writer.WriteElementU32("MaxSpeed", ml.maxSpeed);
            writer.WriteElementU32("MaxAcceleration", ml.maxAcceleration);
            writer.WriteElementU32("MaxDeceleration", ml.maxDeceleration);
            writer.WriteElementU32("BrakeDuration", ml.brakeDuration);
            writer.WriteElementU32("StartingSpeed", ml.startingSpeed);
            writer.WriteEndElement();
        }
    }
}