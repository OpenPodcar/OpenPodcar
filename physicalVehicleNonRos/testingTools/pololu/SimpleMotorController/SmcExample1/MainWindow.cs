/*  SmcExample1:
 *    Simple example GUI for the Pololu Simple Motor Controller,
 *    written in Visual C#.
 *    
 *    Features:
 *       Native USB connection using Smc class
 *       Forward button
 *       Reverse button
 *       Stop button
 * 
 *  NOTE: The Input Mode of your Simple Motor Controller must be set to Serial/USB
 *  for this program to work properly.  You must also connect USB, motor power,
 *  and your motor.  If this program does not work, use the Pololu Simple Motor
 *  Control Center to check what errors are occurring.
 */

using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Text;
using System.Windows.Forms;
using Pololu.UsbWrapper;

namespace Pololu.SimpleMotorController.SmcExample1
{
    public partial class MainWindow : Form
    {
        public MainWindow()
        {
            InitializeComponent();
        }

        /// <summary>
        /// This function runs when the user clicks the Forward button.
        /// </summary>
        void forwardButton_Click(object sender, EventArgs e)
        {
            try
            {
                using (Smc device = connectToDevice())  // Find a device and temporarily connect.
                {
                    device.resume();         // Clear as many errors as possible.
                    device.setSpeed(3200);   // Set the speed to full forward (+100%).
                }
            }
            catch (Exception exception)  // Handle exceptions by displaying them to the user.
            {
                displayException(exception);
            }
        }

        /// <summary>
        /// This function runs when the user clicks the Reverse button.
        /// </summary>
        void reverseButton_Click(object sender, EventArgs e)
        {
            try
            {
                using (Smc device = connectToDevice())  // Find a device and temporarily connect.
                {
                    device.resume();          // Clear as many errors as possible.
                    device.setSpeed(-3200);   // Set the speed to full reverse (-100%).
                }
            }
            catch (Exception exception)  // Handle exceptions by displaying them to the user.
            {
                displayException(exception);
            }
        }

        /// <summary>
        /// This function runs when the user clicks the Stop button.
        /// </summary>
        void stopButton_Click(object sender, EventArgs e)
        {
            try
            {
                using (Smc device = connectToDevice())  // Find a device and temporarily connect.
                {
                    device.stop();  // Activate the USB kill switch

                    // Alternatively you can set the speed to 0 to stop the motor,
                    // but that will only stop the motor if the input mode is Serial/USB:
                    //    device.setSpeed(0);
                }
            }
            catch (Exception exception)  // Handle exceptions by displaying them to the user.
            {
                displayException(exception);
            }
        }

        /// <summary>
        /// Connects to a Simple Motor Controller using native USB and returns the
        /// Smc object representing that connection.  When you are done with the 
        /// connection, you should close it using the Dispose() method so that other
        /// processes or functions can connect to the device later.  The "using"
        /// statement can do this automatically for you.
        /// </summary>
        Smc connectToDevice()
        {
            // Get a list of all connected devices of this type.
            List<DeviceListItem> connectedDevices = Smc.getConnectedDevices();

            foreach(DeviceListItem dli in connectedDevices)
            {
                // If you have multiple devices connected and want to select a particular
                // device by serial number, you could simply add a line like this:
                //   if (dli.serialNumber != "39FF-7406-3054-3036-4923-0743"){ continue; }

                Smc device = new Smc(dli); // Connect to the device.
                return device;             // Return the device.
            }
            throw new Exception("Could not find device.  Make sure it is plugged in to USB " +
                "and check your Device Manager (Windows) or run lsusb (Linux).");
        }

        /// <summary>
        /// Displays an exception to the user by popping up a message box.
        /// </summary>
        public void displayException(Exception exception)
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
