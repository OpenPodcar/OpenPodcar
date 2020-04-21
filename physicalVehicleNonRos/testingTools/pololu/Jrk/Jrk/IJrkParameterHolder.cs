using System;
using System.Collections.Generic;
using System.Text;
using Pololu.Jrk;

namespace Pololu.Jrk
{
    /// <summary>
    /// An interface to be used by anything that can store jrk parameter values.  This includes MainWindow as well as the Jrk itself.
    /// </summary>
    public interface IJrkParameterHolder
    {
        void setJrkParameter(jrkParameter parameter, uint value);
        uint getJrkParameter(jrkParameter parameter);
    }
}
