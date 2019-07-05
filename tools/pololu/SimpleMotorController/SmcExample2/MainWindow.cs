/*  SmcExample2:
 *    Simple example GUI for the Pololu Simple Motor Controller,
 *    written in Visual C#.
 *    
 *    Features:
 *       Persistent native USB connection using Smc class
 *       Scroll bar to set speed
 *       Stop button
 *       Resume button to clear errors
 * 
 *  Troubleshooting:
 * 
 *    The Input Mode of your Simple Motor Controller must be set to Serial/USB
 *    for this program to work properly.  You must also connect USB, motor power,
 *    and your motor.
 *  
 *    If the label says "Disconnected", make sure your device is plugged in to USB,
 *    check your Device Manager (Windows) or run lsusb (Linux), and close any other
 *    programs using the device.
 * 
 *    If the label says "Connected" but the scroll bar does not work, try pressing
 *    the Resume button.  If that does not work, use the Pololu Simple Motor Control
 *    Center to check what errors are occurring.
 *  
 *    The scroll bar will NOT work if the red LED of your motor controller is on
 *    (i.e. whenever there are errors stopping the motor).
 * 
 */

using Pololu.SimpleMotorController;
using Pololu.UsbWrapper;

using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Windows.Forms;

namespace Pololu.SimpleMotorController.SmcExample2
{
    public partial class MainWindow : Form
    {
        public MainWindow()
        {
            InitializeComponent();
        }

        #region User Interface
        
        /// <summary>
        /// This function runs when the user clicks the Stop button.
        /// </summary>
        void StopButton_Click(object sender, EventArgs e)
        {
            if (device != null)
            {
                try
                {
                    device.stop();  // Activate the USB kill switch

                    // Alternatively you can set the speed to 0 to stop the motor,
                    // but that will only stop the motor if the input mode is Serial/USB:
                    //    device.setSpeed(0);
                }
                catch (Exception exception)
                {
                    DisplayException(exception);
                }
            }
        }

        /// <summary>
        /// This function runs when the user clicks the Resume button.
        /// </summary>
        void ResumeButton_Click(object sender, EventArgs e)
        {
            if (device != null)
            {
                try
                {
                    device.resume(); // Clear as many errors as possible.
                }
                catch (Exception exception)
                {
                    DisplayException(exception);
                }
            }
        }

        /// <summary>
        /// This is called whenever the user moves the scroll bar.
        /// </summary>
        void ScrollBar_ValueChanged(object sender, EventArgs e)
        {
            SetSpeed();
        }

        /// <summary>
        /// Reads the value of the scroll bar and sends it to the
        /// device using the setSpeed command.
        /// </summary>
        void SetSpeed()
        {
            if (device != null)
            {
                try
                {
                    Int16 scrolledSpeed = ComputeScrolledSpeed();  // speed is from -3200 to 3200
                    device.setSpeed(scrolledSpeed);
                }
                catch (Exception exception)
                {
                    DisplayException(exception);
                }
            }
        }

        /// <summary>
        /// Returns the speed selected by the user as number from -3200 to 3200.
        /// </summary>
        Int16 ComputeScrolledSpeed()
        {
            if (ScrollBar.Value < -3200)
            {
                return -3200;
            }
            else if (ScrollBar.Value > 3200)
            {
                return 3200;
            }
            else
            {
                return (Int16)ScrollBar.Value;
            }
        }

        #endregion

        #region Maintaining a USB Connection

        /// <summary>
        /// This object represents our current USB connection to a specific
        /// Simple Motor Controller.  If it is null, then we are not connected.
        /// 
        /// When you are done with the connection, you should close it using the
        /// Dispose() method so that other processes or functions can connect to
        /// the device later.  (The "using" statement can do this automatically
        /// for you, as demonstrated in SmcExample1.)  This, however, is a simple
        /// program that will hog the USB connection until the device is
        /// disconnected or the program is terminated.  In Windows this means that
        /// no other devices will be able to use the native USB interface of the
        /// device while the program is running.
        /// </summary>
        Smc device = null;

        /// <summary>
        /// Connects to a device using native USB.  Throws an exception if it fails.
        /// </summary>
        void Connect()
        {
            if (device != null)
            {
                // We are already connected.
                return;
            }

            // Get a list of all connected devices of this type.
            List<DeviceListItem> connectedDevices = Smc.getConnectedDevices();

            foreach (DeviceListItem dli in connectedDevices)
            {
                // If you have multiple devices connected and want to select a particular
                // device by serial number, you could simply add a line like this:
                //   if (dli.serialNumber != "39FF-7406-3054-3036-4923-0743"){ continue; }

                this.device = new Smc(dli); // Connect to the device.
                StatusLabel.Text = "Connected";
                StatusLabel.ForeColor = SystemColors.ControlText;
                return;
            }
            throw new Exception("Could not find device.  Make sure it is plugged in to USB " +
                "and check your Device Manager (Windows) or run lsusb (Linux).");
        }

        /// <summary>
        /// Closes our connection to the device.  Does not throw exceptions.
        /// </summary>
        void Disconnect()
        {
            if (device == null)
            {
                // We are already disconnected.
                return;
            }

            StatusLabel.Text = "Disconnected";
            StatusLabel.ForeColor = Color.Red;

            try
            {
                device.Dispose();  // Disconnect
            }
            catch    // Ignore exceptions.
            {
            }
            finally  // Do this no matter what.
            {
                device = null;
            }
        }

        /// <summary>
        /// Detects whether the device we are connected to is still plugged in
        /// to USB.  If it is unplugged, we close the connection.
        /// </summary>
        void DetectDisconnect()
        {
            // Get a list of all connected devices of this type.
            List<DeviceListItem> connectedDevices = Smc.getConnectedDevices();

            foreach (DeviceListItem dli in connectedDevices)
            {
                if (dli.serialNumber == device.getSerialNumber())
                {
                    // The device we are connected to is still plugged in.
                    return;
                }
            }

            // The device we are connected to is not plugged in, so disconnect.
            Disconnect();
        }

        private void UpdateConnectionStatus()
        {
            if (device != null)
            {
                // We are currently connected to a device, so check to see if it is
                // still plugged in to USB.
                try
                {
                    DetectDisconnect();
                }
                catch // Ignore exceptions
                {
                }
            }

            if (device == null)
            {
                // We are not currently connected to a device, so try to connect.
                try
                {
                    Connect();
                }
                catch // Ignore exceptions (e.g. no devices found, or access denied when connecting)
                {
                }
            }
        }

        /// <summary>
        /// This function is called when the program starts.
        /// </summary>
        void MainWindow_Load(object sender, EventArgs e)
        {
            if (Usb.supportsNotify)
            {
                // This program is running in Windows, which supports sending a
                // notification to the program whenever particular USB devices are
                // connected or disconnected.  In Windows, this is the preferred
                // method of detecting device connection and disconnection, because
                // it is faster and uses less CPU time than polling.  Either method
                // will work in Windows (if you want to use polling in Windows, just
                // change the if statement above to "if (false)". 

                // Use notifications (WndProc).
                Usb.notificationRegister(Smc.deviceInterfaceGuid, this.Handle);

                UpdateConnectionStatus();
            }
            else
            {
                // Use polling (UpdateTimer_Tick).
                UpdateTimer.Start();
            }
        }
        
        /// <summary>
        /// This function is called whenever a message is received from windows.  If you
        /// are using notifications (only available in Windows) to maintain the status
        /// of your USB connection, then this function will receive messages about USB
        /// device changes, and will call UpdateConnectionStatu whenever such a message
        /// is received.  See MainWindow_Load for details.
        /// </summary>
        protected override void WndProc(ref Message m)
        {
            if (m.Msg == Usb.WM_DEVICECHANGE)
            {
                UpdateConnectionStatus();
            }
            base.WndProc(ref m);
        }

        /// <summary>
        /// If you are using polling to maintain the status of your USB connection, then
        /// this function will be called regularly (whenever UpdateTimer ticks).  See
        /// MainWindow_Load for details.
        /// </summary>
        void UpdateTimer_Tick(object sender, EventArgs e)
        {
            UpdateConnectionStatus();
        }

        #endregion

        /// <summary>
        /// Displays an exception to the user by popping up a message box.
        /// </summary>
        public void DisplayException(Exception exception)
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
