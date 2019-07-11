using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace Pololu.UsbAvrProgrammer
{
    /// <summary>
    /// A class that represents all of the programmer variables that are
    /// stored in persistent memory.
    /// </summary>
    /// <remarks>The default value of the ProgrammerSettings object is the
    /// same as the settings that the programmer is shipped with.
    /// See Programmer.restoreDefaultSettings.</remarks>
    public class ProgrammerSettings
    {
        /// <summary>
        /// The lowest the target VCC is allowed to be, in units of mV.
        /// This is stored on the device in units of 32 mV.
        /// Valid range is 0-8160.
        /// </summary>
        public ushort targetVccAllowedMinimum = 4384;

        /// <summary>
        /// The largest acceptable range (max-min) of the target VCC, in units of mV.
        /// This is stored on the device in units of 32 mV.
        /// Valid range is 0-8160.
        /// </summary>
        public ushort targetVccAllowedMaximumRange = 512;

        /// <summary>The RS-232 control line that line A is associated with.</summary>
        public LineIdentity lineAIdentity = LineIdentity.None;

        /// <summary>The RS-232 control line that line B is associated with.</summary>
        public LineIdentity lineBIdentity = LineIdentity.None;

        /// <summary>
        /// A representation of the amount of time that the SCK line is low/high
        /// during programming.  This determines the ISP frequency.
        /// If the frequency of your AVR (instruction cycles per second) is less than
        /// 12 MHZ, then the ISP frequency must be less than 1/4 the AVR's frequency.
        /// Otherwise, the ISP frequency must be less than 1/6 the AVR's frequency.
        /// </summary>
        public SckDuration sckDuration = SckDuration.Frequency200;

        /// <summary>
        /// A number reported to AVR Studio that must be what AVR Studio expects
        /// or it will complain.
        /// </summary>
        public byte softwareVersionMajor = 0x02;

        /// <summary>
        /// A number reported to AVR Studio that must be what AVR Studio expects
        /// or it will prompt the user for a firmware upgrade.
        /// </summary>
        public byte softwareVersionMinor = 0x0A;

        /// <summary>
        /// A number reported to AVR Studio that must be what AVR Studio expects
        /// or it will prompt the user for a firmware upgrade.
        /// </summary>
        public byte hardwareVersion = 0x0F;
    }

}
