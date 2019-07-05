using System;
using System.Collections.Generic;
using System.Text;
using Pololu.Usc;

namespace Pololu.Usc
{
    /// <summary>
    /// An interface to be used by anything that can store usc parameter values.
    /// This includes MainWindow and Usc itself.
    /// </summary>
    /// <remarks>
    /// When implementing setUscSettings and getUscSettings in your class, look
    /// at a saved configuration file to make sure you have handled every setting.
    /// </remarks>
    public interface IUscSettingsHolder
    {
        void setUscSettings(UscSettings settings, bool newScript);
        UscSettings getUscSettings();
    }
}