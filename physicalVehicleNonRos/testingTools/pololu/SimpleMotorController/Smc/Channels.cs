// This file contains an enum used to identify a channel (SmcChannel) and
// various functions for doing things with that enum.

using System;

namespace Pololu.SimpleMotorController
{
    /// <summary>
    /// Specifies an input channel (e.g. Analog Channel 2).
    /// </summary>
    public enum SmcChannel
    {
        /// <summary>
        /// No channel specified.
        /// </summary>
        None = 0,
        /// <summary>
        /// Analog Channel 1.
        /// </summary>
        Analog1,
        /// <summary>
        /// Analog Channel 2.
        /// </summary>
        Analog2,
        /// <summary>
        /// RC Channel 1
        /// </summary>
        Rc1,
        /// <summary>
        /// RC Channel 2
        /// </summary>
        Rc2
    }

    /// <summary>
    /// Specifies a channel type: RC or Analog.
    /// </summary>
    public enum SmcChannelType
    {
        /// <summary>
        /// RC: this channel measures the width of pulses received.
        /// </summary>
        RC,

        /// <summary>
        /// Analog: this channel measures voltage.
        /// </summary>
        Analog
    }

    /// <summary>
    /// Specifies a channel use, including all Alternate uses (limit switches)
    /// and primary uses (controlling the motor speed).
    /// </summary>
    public enum SmcChannelUse : byte
    {
        // Alternate Uses: (copied from SmcChannelAlternateUse)

        /// <summary>
        /// None: This channel is not used for anything special but its value
        /// can be read using Serial or USB commands.
        /// </summary>
        None = 0,

        /// <summary>
        /// Forward Limit Switch: if this channel is active (Scaled Value >= 1600),
        /// then the motor is not allowed to move forward.
        /// </summary>
        LimitForward = 1,

        /// <summary>
        /// Reverse Limit Switch: if this channel is active (Scaled Value >= 1600),
        /// then the motor is not allowed to move in reverse.
        /// </summary>
        LimitReverse = 2,

        /// <summary>
        /// Kill Switch: if this channel is active (Scale Value >= 1600), then the
        /// motor is not allowed to move.
        /// </summary>
        KillSwitch = 3,

        // Main uses:

        /// <summary>
        /// Throttle: this channel is the motor throttle (reverse/forward) channel.
        /// Its Scaled Value is used to determine the motor's Target Speed.
        /// </summary>
        Throttle = 4,

        /// <summary>
        /// Throttle: this channel is the motor steering (left/right) channel.
        /// Its Scaled Value is used to determine the motor's Target Speed.
        /// </summary>
        Steering = 5
    }

    /// <summary>
    /// This static class contains extension methods that deal with identifying
    /// channels and figuring out what they are used for in the current settings.
    /// </summary>
    static public class SmcChannelUtil
    {
        /// <summary>
        /// Returns a user friendly name for the channel (e.g. "Analog Channel 1").
        /// </summary>
        public static string name(this SmcChannel channel)
        {
            switch (channel)
            {
                case SmcChannel.Analog1: return "Analog Channel 1";
                case SmcChannel.Analog2: return "Analog Channel 2";
                case SmcChannel.Rc1: return "RC Channel 1";
                case SmcChannel.Rc2: return "RC Channel 2";
                default: throw new Exception("Unknown Channel: " + channel.ToString());
            }
        }

        /// <summary>
        /// Returns a short user friendly name for the channel (e.g. "Analog 1").
        /// </summary>
        public static string shortName(this SmcChannel channel)
        {
            switch (channel)
            {
                case SmcChannel.Analog1: return "Analog 1";
                case SmcChannel.Analog2: return "Analog 2";
                case SmcChannel.Rc1: return "RC 1";
                case SmcChannel.Rc2: return "RC 2";
                default: throw new Exception("Unknown Channel: " + channel.ToString());
            }
        }

        /// <summary>
        /// Returns the type of the channel: RC or Analog.
        /// </summary>
        public static SmcChannelType type(this SmcChannel channel)
        {
            switch (channel)
            {
                case SmcChannel.Analog1:
                case SmcChannel.Analog2: return SmcChannelType.Analog;
                case SmcChannel.Rc1:
                case SmcChannel.Rc2: return SmcChannelType.RC;
                default: throw new Exception("Unknown Channel: " + channel.ToString());
            }
        }

        /// <summary>
        /// Converts the Raw Value (or Unlimited Raw Value) of a channel in to a user
        /// friendly string that expresses the value in standard units.
        /// For example: "1501 us" or "1452 mV".  It uses a non-ascii character to
        /// encode the Greek letter mu in microseconds.
        /// </summary>
        public static String rawValueToStandardUnitsString(this SmcChannel channel, UInt16 rawValue)
        {
            switch(channel.type())
            {
                case SmcChannelType.RC: return ((rawValue + 2) / 4).ToString() + " \u03BCs";
                case SmcChannelType.Analog: return (rawValue * 3300 / 4095).ToString() + " mV";
                default: return rawValue.ToString(); // should not happen
            }

        }

        /// <summary>
        /// Gets the state of the specified channel.
        /// </summary>
        /// <param name="vars">The state of the device.</param>
        /// <param name="channel">Specifies what channel to fetch.</param>
        /// <returns>The state of the specified channel.</returns>
        public static SmcChannelVariables getChannel(this SmcVariables vars, SmcChannel channel)
        {
            switch (channel)
            {
                case SmcChannel.Analog1: return vars.analog1;
                case SmcChannel.Analog2: return vars.analog2;
                case SmcChannel.Rc1: return vars.rc1;
                case SmcChannel.Rc2: return vars.rc2;
                default: throw new Exception("Unknown Channel: " + channel.ToString());
            }
        }

        /// <summary>
        /// Gets the settings for the specified channel.
        /// </summary>
        /// <param name="settings">The settings of the device.</param>
        /// <param name="channel">Specifies what channel to fetch.</param>
        /// <returns>The settings of the specified channel.</returns>
        public static SmcChannelSettings getChannelSettings(this SmcSettings settings, SmcChannel channel)
        {
            switch (channel)
            {
                case SmcChannel.Analog1: return settings.analog1;
                case SmcChannel.Analog2: return settings.analog2;
                case SmcChannel.Rc1: return settings.rc1;
                case SmcChannel.Rc2: return settings.rc2;
                default: throw new Exception("Unknown Channel: " + channel.ToString());
            }
        }

        /// <summary>
        /// Changes the settings for a specified channel.
        /// </summary>
        /// <param name="settings">The settings of the device. This object will be modified.</param>
        /// <param name="channel">Specifies the channel to change.</param>
        /// <param name="channelSettings">The new settings for the channel.</param>
        public static void setChannelSettings(this SmcSettings settings, SmcChannel channel, SmcChannelSettings channelSettings)
        {
            switch (channel)
            {
                case SmcChannel.Analog1: settings.analog1 = channelSettings; break;
                case SmcChannel.Analog2: settings.analog2 = channelSettings; break;
                case SmcChannel.Rc1: settings.rc1 = channelSettings; break;
                case SmcChannel.Rc2: settings.rc2 = channelSettings; break;
                default: throw new Exception("Unknown Channel: " + channel.ToString());
            }
        }

        /// <summary>
        /// Determines whether the specified channel is directly used to control the
        /// motor speed (i.e. it is a Throttle or Steering input).
        /// </summary>
        /// <param name="channel">Specifies the channel.</param>
        /// <param name="inputMode">The input mode of the device (Serial/USB, Analog, RC).</param>
        /// <param name="mixingMode">The mixing mode of the device (None, Left, Right).</param>
        /// <returns>True if and only if the channel controls the motor speed.</returns>
        public static bool controlsMotorSpeed(this SmcChannel channel, SmcInputMode inputMode, SmcMixingMode mixingMode)
        {
            if (inputMode == SmcInputMode.Analog)
            {
                return (channel == SmcChannel.Analog1 || (mixingMode != SmcMixingMode.None && channel == SmcChannel.Analog2));
            }
            else if (inputMode == SmcInputMode.RC)
            {
                return (channel == SmcChannel.Rc1 || (mixingMode != SmcMixingMode.None && channel == SmcChannel.Rc2));
            }
            else
            {
                // inputMode should be serial/USB
                return false;
            }
        }

        /// <summary>
        /// Determines what the specified channel is used for (e.g. limit switch or
        /// controlling the motor speed).
        /// </summary>
        /// <param name="channel">Specifies the channel.</param>
        /// <param name="inputMode">The input mode of the device (Serial/USB, Analog, RC).</param>
        /// <param name="mixingMode">The mixing mode of the device (None, Left, Right).</param>
        /// <param name="alternateUse">The alternate use setting for the channel.</param>
        /// <returns>What the channel is used for.</returns>
        public static SmcChannelUse use(this SmcChannel channel, SmcInputMode inputMode, SmcMixingMode mixingMode, SmcChannelAlternateUse alternateUse)
        {
            if (channel.controlsMotorSpeed(inputMode, mixingMode))
            {
                if (channel == SmcChannel.Analog2 || channel == SmcChannel.Rc2)
                {
                    return SmcChannelUse.Steering;
                }
                else
                {
                    return SmcChannelUse.Throttle;
                }
            }
            else
            {
                switch (alternateUse)
                {
                    default: return SmcChannelUse.None;
                    case SmcChannelAlternateUse.KillSwitch: return SmcChannelUse.KillSwitch;
                    case SmcChannelAlternateUse.LimitForward: return SmcChannelUse.LimitForward;
                    case SmcChannelAlternateUse.LimitReverse: return SmcChannelUse.LimitReverse;
                }
            }
        }

        /// <summary>
        /// Returns true if the specified channel is a limit or kill switch and
        /// it is currently active.  This information comes from the limitStatus
        /// register.
        /// </summary>
        public static bool switchActive(this SmcVariables vars, SmcChannel channel)
        {
            switch (channel)
            {
                case SmcChannel.Analog1: return (vars.limitStatus & SmcLimitStatus.Analog1) != 0;
                case SmcChannel.Analog2: return (vars.limitStatus & SmcLimitStatus.Analog2) != 0;
                case SmcChannel.Rc1: return (vars.limitStatus & SmcLimitStatus.Rc1) != 0;
                case SmcChannel.Rc2: return (vars.limitStatus & SmcLimitStatus.Rc2) != 0;
                default: throw new Exception("Unknown Channel: " + channel.ToString());
            }
        }
    }
}
