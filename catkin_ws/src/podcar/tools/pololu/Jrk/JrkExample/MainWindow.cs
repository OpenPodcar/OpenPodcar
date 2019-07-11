using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Text;
using System.Windows.Forms;
using Pololu.Jrk;
using Pololu.UsbWrapper;
using System.Threading;

namespace Pololu.Jrk.JrkExample
{
    public partial class MainWindow : Form
    {
        public MainWindow()
        {
            InitializeComponent();
        }

        #region events
        /// <summary>
        /// When the program starts, this function is called.
        /// It gets the serial number of the first connected Jrk,
        /// since this is probably what you want to connect to.
        /// </summary>
        void MainWindow_Shown(object sender, EventArgs e)
        {
            var deviceList = Jrk.getConnectedDevices();
            if (deviceList.Count > 0)
            {
                SerialNumberTextBox.Text = deviceList[0].serialNumber;
                ConnectButton.Focus();
            }
            else
            {
                SerialNumberTextBox.Focus();
            }
        }

        void ConnectButton_Click(object sender, EventArgs e)
        {
            SerialNumberTextBox.Enabled = false;
            Log("Connecting...");
            UpdateTimer.Start();
            StopConnectingButton.Enabled = true;
            ConnectButton.Enabled = false;
        }

        void StopConnectingButton_Click(object sender, EventArgs e)
        {
            SerialNumberTextBox.Enabled = true;
            TryToDisconnect();
            UpdateTimer.Stop();
            StopConnectingButton.Enabled = false;
            ConnectButton.Enabled = true;
            TargetTextBox.Text = "(disconnected)";
            ScaledFeedbackTextBox.Text = "(disconnected)";
        }
        #endregion

        #region logging
        void Log(Exception e)
        {
            Log(e.Message);
        }

        void Log(string text)
        {
            if (LogTextBox.Text != "")
                LogTextBox.Text += Environment.NewLine;
            LogTextBox.Text += DateTime.Now.ToString() + "\t" + text;
            LogTextBox.SelectionStart = LogTextBox.Text.Length;
            LogTextBox.ScrollToCaret();
        }
        #endregion

        #region the Jrk connection

        Jrk jrk = null;

        /// <summary>
        /// Connects to the device if it is found in the device list.
        /// </summary>
        void TryToReconnect()
        {
            foreach (DeviceListItem d in Jrk.getConnectedDevices())
            {
                if (d.serialNumber == SerialNumberTextBox.Text)
                {
                    jrk = new Jrk(d);
                    Log("Connected to #" + SerialNumberTextBox.Text + ".");
                    return;
                }
            }
        }

        void TryToDisconnect()
        {
            if (jrk == null)
            {
                Log("Connecting stopped.");
                return;
            }

            try
            {
                Log("Disconnecting...");
                jrk.disconnect();
            }
            catch (Exception e)
            {
                Log(e);
                Log("Failed to disconnect cleanly.");
            }
            finally
            {
                // do this no matter what
                jrk = null;
                Log("Disconnected from #" + SerialNumberTextBox.Text + ".");
            }
        }

        /// <summary>
        /// Reads information from the Jrk and displays it in the textboxes.
        /// </summary>
        void DisplayStatus()
        {
            jrkVariables vars = jrk.getVariables();

            TargetTextBox.Text = vars.target.ToString();
            ScaledFeedbackTextBox.Text = vars.scaledFeedback.ToString();
        }

        int sequenceCounter = 0;

        /// <summary>
        /// Steps through a simple sequence, setting the target to 
        /// 1500, 2000, and 2500 with 1 second between frames.
        /// This function is run every 100 ms when the motion sequence
        /// is activated.
        /// </summary>
        void RunSequence()
        {
            if (sequenceCounter < 10)
            {
                jrk.setTarget(1500);
            }
            else if (sequenceCounter < 20)
            {
                jrk.setTarget(2000);
            }
            else if (sequenceCounter < 30)
            {
                jrk.setTarget(2500);
            }
            else
            {
                sequenceCounter = 0;
            }

            // increment the counter by 1 every 100 ms
            sequenceCounter += 1;
        }

        #endregion

        #region updating

        /// <summary>
        /// This function will be called once every 100 ms to do an update.
        /// </summary>
        void UpdateTimer_Tick(object sender, EventArgs eventArgs)
        {
            if (jrk == null)
            {
                // Display a message in the textboxes.
                TargetTextBox.Text = "(disconnected)";
                ScaledFeedbackTextBox.Text = "(disconnected)";

                // Try connecting to a device.
                try
                {
                    TryToReconnect();
                }
                catch (Exception e)
                {
                    Log(e);
                    Log("Failed connecting to #" + SerialNumberTextBox.Text + ".");
                    jrk = null;
                }
            }
            else
            {
                // Update the GUI and the device.
                try
                {
                    DisplayStatus();
                    if (ActivateCheckBox.Checked)
                        RunSequence();
                }
                catch (Exception e)
                {
                    // If any exception occurs, log it, set jrk to null, and keep trying..
                    Log(e);
                    Log("Disconnected from #"+SerialNumberTextBox.Text+".");
                    jrk = null;
                }
            }
        }

        #endregion

    }
}
