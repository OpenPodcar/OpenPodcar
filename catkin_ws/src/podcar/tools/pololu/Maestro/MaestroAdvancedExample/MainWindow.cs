using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Text;
using System.Windows.Forms;
using Pololu.Usc;
using Pololu.UsbWrapper;
using System.Threading;

namespace Pololu.Usc.MaestroAdvancedExample
{
    public partial class MainWindow : Form
    {
        public MainWindow()
        {
            InitializeComponent();
        }

        #region events
        /// <summary>
        /// Get the serial number of the first connected Maestro, since this is
        /// probably what you want to connect to.
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void MainWindow_Shown(object sender, EventArgs e)
        {
            var device_list = Usc.getConnectedDevices();
            if (device_list.Count > 0)
            {
                SerialNumberTextBox.Text = device_list[0].serialNumber;
                ConnectButton.Focus();
            }
            else
            {
                SerialNumberTextBox.Focus();
            }
        }

        private void ConnectButton_Click(object sender, EventArgs e)
        {
            SerialNumberTextBox.Enabled = false;
            Log("Connecting...");
            UpdateTimer.Start();
            StopConnectingButton.Enabled = true;
            ConnectButton.Enabled = false;
        }

        private void StopConnectingButton_Click(object sender, EventArgs e)
        {
            SerialNumberTextBox.Enabled = true;
            TryToDisconnect();
            UpdateTimer.Stop();
            StopConnectingButton.Enabled = false;
            ConnectButton.Enabled = true;
            PositionTextBox.Text = "(disconnected)";
        }
        #endregion

        #region logging
        private void Log(Exception e)
        {
            Log(e.Message);
        }

        private void Log(string text)
        {
            if (LogTextBox.Text != "")
                LogTextBox.Text += Environment.NewLine;
            LogTextBox.Text += DateTime.Now.ToString() + "\t" + text;
            LogTextBox.SelectionStart = LogTextBox.Text.Length;
            LogTextBox.ScrollToCaret();
        }
        #endregion

        #region the Maestro connection

        private Usc usc = null;

        /// <summary>
        /// Connects to the device if it is found in the device list.
        /// </summary>
        private void TryToReconnect()
        {
            foreach (DeviceListItem d in Usc.getConnectedDevices())
            {
                if (d.serialNumber == SerialNumberTextBox.Text)
                {
                    usc = new Usc(d);
                    Log("Connected to #" + SerialNumberTextBox.Text + ".");
                    return;
                }
            }
        }

        private void TryToDisconnect()
        {
            if (usc == null)
            {
                Log("Connecting stopped.");
                return;
            }

            try
            {
                Log("Disconnecting...");
                usc.Dispose();  // Disconnect
            }
            catch (Exception e)
            {
                Log(e);
                Log("Failed to disconnect cleanly.");
            }
            finally
            {
                // do this no matter what
                usc = null;
                Log("Disconnected from #" + SerialNumberTextBox.Text + ".");
            }
        }

        /// <summary>
        /// Displays the position of servo 0 in the text box.
        /// </summary>
        private void DisplayPosition()
        {
            ServoStatus[] servos;
            usc.getVariables(out servos);

            PositionTextBox.Text = servos[0].position.ToString();
        }

        private int sequence_counter = 0;

        /// <summary>
        /// Steps through a simple sequence, sending servo 0 to 4000, 6000, then 8000, with 1 s between frames.
        /// </summary>
        private void RunSequence()
        {
            if (sequence_counter < 10)
            {
                usc.setTarget(0, 4000);
            }
            else if (sequence_counter < 20)
            {
                usc.setTarget(0, 6000);
            }
            else if (sequence_counter < 30)
            {
                usc.setTarget(0, 8000);
            }
            else
            {
                sequence_counter = 0;
            }

            // increment the counter by 1 every 100 ms
            sequence_counter += 1;
        }

        #endregion

        #region updating

        /// <summary>
        /// This function will be called once every 100 ms to do an update.
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void UpdateTimer_Tick(object sender, EventArgs e)
        {
            if (usc == null)
            {
                // Display a message in the position box
                PositionTextBox.Text = "(disconnected)";

                // Try connecting to a device.
                try
                {
                    TryToReconnect();
                }
                catch (Exception e2)
                {
                    Log(e2);
                    Log("Failed connecting to #" + SerialNumberTextBox.Text + ".");
                    usc = null;
                }
            }
            else
            {
                // Update the GUI and the device.
                try
                {
                    DisplayPosition();
                    if (ActivateCheckBox.Checked)
                        RunSequence();
                }
                catch (Exception e2)
                {
                    // If any exception occurs, log it, set usc to null, and keep trying..
                    Log(e2);
                    Log("Disconnected from #"+SerialNumberTextBox.Text+".");
                    usc = null;
                }
            }
        }

        #endregion

    }
}
