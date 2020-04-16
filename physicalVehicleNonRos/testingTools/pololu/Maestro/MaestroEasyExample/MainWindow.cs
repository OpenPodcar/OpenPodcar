/*  MaestroEasyExample:
 *    Simple example GUI for the Maestro USB Servo Controller, written in
 *    Visual C#.
 *    
 *    Features:
 *       Temporary native USB connection using Usc class
 *       Button for disabling channel 0.
 *       Button for setting target of channel 0 to 1000 us.
 *       Button for setting target of channel 0 to 2000 us.
 * 
 *  NOTE: Channel 0 should be configured as a servo channel for this program
 *  to work.  You must also connect USB and servo power, and connect a servo
 *  to channel 0.  If this program does not work, use the Maestro Control
 *  Center to check what errors are occurring.
 */

using Pololu.Usc;
using Pololu.UsbWrapper;

using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Text;
using System.Windows.Forms;

namespace Pololu.Usc.MaestroEasyExample
{
    public partial class MainWindow : Form
    {
        public MainWindow()
        {
            InitializeComponent();
        }

        /// <summary>
        /// This functions runs when the user clicks the Target=1000us button.
        /// </summary>
        void Button1000_Click(object sender, EventArgs e)
        {
            TrySetTarget(0, 1000 * 4);  // Set the target of channel 0 to 1000 microseconds.
        }

        /// <summary>
        /// This functions runs when the user clicks the Target=2000us button.
        /// </summary>
        void Button2000_Click(object sender, EventArgs e)
        {
            TrySetTarget(0, 2000 * 4);  // Set the target of channel 0 to 2000 microseconds.
        }

        /// <summary>
        /// This function runs when the user clicks the Disable button.
        /// </summary>
        void ButtonDisable_Click(object sender, EventArgs e)
        {
            // Set target of channel 0 to 0.  This tells the Maestro to stop
            // transmitting pulses on that channel.  Any servo connected to it
            // should stop trying to maintain its position.
            TrySetTarget(0, 0);
        }
        
        /// <summary>
        /// Attempts to set the target (width of pulses sent) of a channel.
        /// </summary>
        /// <param name="channel">Channel number from 0 to 23.</param>
        /// <param name="target">
        ///   Target, in units of quarter microseconds.  For typical servos,
        ///   6000 is neutral and the acceptable range is 4000-8000.
        /// </param>
        void TrySetTarget(Byte channel, UInt16 target)
        {
            try
            {
                using (Usc device = connectToDevice())  // Find a device and temporarily connect.
                {
                    device.setTarget(channel, target);

                    // device.Dispose() is called automatically when the "using" block ends,
                    // allowing other functions and processes to use the device.
                }
            }
            catch (Exception exception)  // Handle exceptions by displaying them to the user.
            {
                displayException(exception);
            }
        }

        /// <summary>
        /// Connects to a Maestro using native USB and returns the Usc object
        /// representing that connection.  When you are done with the
        /// connection, you should close it using the Dispose() method so that
        /// other processes or functions can connect to the device later.  The
        /// "using" statement can do this automatically for you.
        /// </summary>
        Usc connectToDevice()
        {
            // Get a list of all connected devices of this type.
            List<DeviceListItem> connectedDevices = Usc.getConnectedDevices();

            foreach (DeviceListItem dli in connectedDevices)
            {
                // If you have multiple devices connected and want to select a particular
                // device by serial number, you could simply add a line like this:
                //   if (dli.serialNumber != "00012345"){ continue; }

                Usc device = new Usc(dli); // Connect to the device.
                return device;             // Return the device.
            }
            throw new Exception("Could not find device.  Make sure it is plugged in to USB " +
                "and check your Device Manager (Windows) or run lsusb (Linux).");
        }

        /// <summary>
        /// Displays an exception to the user by popping up a message box.
        /// </summary>
        void displayException(Exception exception)
        {
            StringBuilder stringBuilder = new StringBuilder();
            do
            {
                stringBuilder.Append(exception.Message + "  ");
                if (exception is Win32Exception)
                {
                    stringBuilder.Append("Error code 0x" + ((Win32Exception)exception).NativeErrorCode.ToString("x") + ".  ");
                }
                exception = exception.InnerException;
            }
            while (exception != null);
            MessageBox.Show(stringBuilder.ToString(), this.Text, MessageBoxButtons.OK, MessageBoxIcon.Error);
        }
    }
}
