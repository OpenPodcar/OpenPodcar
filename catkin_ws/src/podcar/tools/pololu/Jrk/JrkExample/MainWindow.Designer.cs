namespace Pololu.Jrk.JrkExample
{
    partial class MainWindow
    {
        /// <summary>
        /// Required designer variable.
        /// </summary>
        private System.ComponentModel.IContainer components = null;

        /// <summary>
        /// Clean up any resources being used.
        /// </summary>
        /// <param name="disposing">true if managed resources should be disposed; otherwise, false.</param>
        protected override void Dispose(bool disposing)
        {
            if (disposing && (components != null))
            {
                components.Dispose();
            }
            base.Dispose(disposing);
        }

        #region Windows Form Designer generated code

        /// <summary>
        /// Required method for Designer support - do not modify
        /// the contents of this method with the code editor.
        /// </summary>
        private void InitializeComponent()
        {
            this.components = new System.ComponentModel.Container();
            this.ConnectButton = new System.Windows.Forms.Button();
            this.StopConnectingButton = new System.Windows.Forms.Button();
            this.ActivateCheckBox = new System.Windows.Forms.CheckBox();
            this.SerialNumberTextBox = new System.Windows.Forms.TextBox();
            this.serialNumberLabel = new System.Windows.Forms.Label();
            this.LogTextBox = new System.Windows.Forms.TextBox();
            this.targetLabel = new System.Windows.Forms.Label();
            this.TargetTextBox = new System.Windows.Forms.TextBox();
            this.logLabel = new System.Windows.Forms.Label();
            this.UpdateTimer = new System.Windows.Forms.Timer(this.components);
            this.scaledFeedbackLabel = new System.Windows.Forms.Label();
            this.ScaledFeedbackTextBox = new System.Windows.Forms.TextBox();
            this.SuspendLayout();
            // 
            // ConnectButton
            // 
            this.ConnectButton.Location = new System.Drawing.Point(371, 11);
            this.ConnectButton.Name = "ConnectButton";
            this.ConnectButton.Size = new System.Drawing.Size(113, 23);
            this.ConnectButton.TabIndex = 0;
            this.ConnectButton.Text = "&Connect";
            this.ConnectButton.UseVisualStyleBackColor = true;
            this.ConnectButton.Click += new System.EventHandler(this.ConnectButton_Click);
            // 
            // StopConnectingButton
            // 
            this.StopConnectingButton.Enabled = false;
            this.StopConnectingButton.Location = new System.Drawing.Point(371, 40);
            this.StopConnectingButton.Name = "StopConnectingButton";
            this.StopConnectingButton.Size = new System.Drawing.Size(113, 23);
            this.StopConnectingButton.TabIndex = 1;
            this.StopConnectingButton.Text = "&Stop connecting";
            this.StopConnectingButton.UseVisualStyleBackColor = true;
            this.StopConnectingButton.Click += new System.EventHandler(this.StopConnectingButton_Click);
            // 
            // ActivateCheckBox
            // 
            this.ActivateCheckBox.AutoSize = true;
            this.ActivateCheckBox.Location = new System.Drawing.Point(15, 40);
            this.ActivateCheckBox.Name = "ActivateCheckBox";
            this.ActivateCheckBox.Size = new System.Drawing.Size(277, 17);
            this.ActivateCheckBox.TabIndex = 2;
            this.ActivateCheckBox.Text = "Activate motion sequence (input mode must be serial)";
            this.ActivateCheckBox.UseVisualStyleBackColor = true;
            // 
            // SerialNumberTextBox
            // 
            this.SerialNumberTextBox.Location = new System.Drawing.Point(103, 14);
            this.SerialNumberTextBox.Name = "SerialNumberTextBox";
            this.SerialNumberTextBox.Size = new System.Drawing.Size(116, 20);
            this.SerialNumberTextBox.TabIndex = 3;
            // 
            // serialNumberLabel
            // 
            this.serialNumberLabel.AutoSize = true;
            this.serialNumberLabel.Location = new System.Drawing.Point(12, 17);
            this.serialNumberLabel.Name = "serialNumberLabel";
            this.serialNumberLabel.Size = new System.Drawing.Size(61, 13);
            this.serialNumberLabel.TabIndex = 4;
            this.serialNumberLabel.Text = "Jrk serial #:";
            // 
            // LogTextBox
            // 
            this.LogTextBox.Anchor = ((System.Windows.Forms.AnchorStyles)((((System.Windows.Forms.AnchorStyles.Top | System.Windows.Forms.AnchorStyles.Bottom)
                        | System.Windows.Forms.AnchorStyles.Left)
                        | System.Windows.Forms.AnchorStyles.Right)));
            this.LogTextBox.Location = new System.Drawing.Point(15, 137);
            this.LogTextBox.Multiline = true;
            this.LogTextBox.Name = "LogTextBox";
            this.LogTextBox.ReadOnly = true;
            this.LogTextBox.ScrollBars = System.Windows.Forms.ScrollBars.Vertical;
            this.LogTextBox.Size = new System.Drawing.Size(469, 137);
            this.LogTextBox.TabIndex = 5;
            // 
            // targetLabel
            // 
            this.targetLabel.AutoSize = true;
            this.targetLabel.Location = new System.Drawing.Point(14, 66);
            this.targetLabel.Name = "targetLabel";
            this.targetLabel.Size = new System.Drawing.Size(74, 13);
            this.targetLabel.TabIndex = 7;
            this.targetLabel.Text = "Current target:";
            // 
            // TargetTextBox
            // 
            this.TargetTextBox.Location = new System.Drawing.Point(146, 63);
            this.TargetTextBox.Name = "TargetTextBox";
            this.TargetTextBox.ReadOnly = true;
            this.TargetTextBox.Size = new System.Drawing.Size(73, 20);
            this.TargetTextBox.TabIndex = 6;
            this.TargetTextBox.TextAlign = System.Windows.Forms.HorizontalAlignment.Right;
            // 
            // logLabel
            // 
            this.logLabel.AutoSize = true;
            this.logLabel.Location = new System.Drawing.Point(14, 121);
            this.logLabel.Name = "logLabel";
            this.logLabel.Size = new System.Drawing.Size(28, 13);
            this.logLabel.TabIndex = 8;
            this.logLabel.Text = "Log:";
            // 
            // UpdateTimer
            // 
            this.UpdateTimer.Tick += new System.EventHandler(this.UpdateTimer_Tick);
            // 
            // scaledFeedbackLabel
            // 
            this.scaledFeedbackLabel.AutoSize = true;
            this.scaledFeedbackLabel.Location = new System.Drawing.Point(14, 92);
            this.scaledFeedbackLabel.Name = "scaledFeedbackLabel";
            this.scaledFeedbackLabel.Size = new System.Drawing.Size(126, 13);
            this.scaledFeedbackLabel.TabIndex = 10;
            this.scaledFeedbackLabel.Text = "Current scaled feedback:";
            // 
            // ScaledFeedbackTextBox
            // 
            this.ScaledFeedbackTextBox.Location = new System.Drawing.Point(146, 89);
            this.ScaledFeedbackTextBox.Name = "ScaledFeedbackTextBox";
            this.ScaledFeedbackTextBox.ReadOnly = true;
            this.ScaledFeedbackTextBox.Size = new System.Drawing.Size(73, 20);
            this.ScaledFeedbackTextBox.TabIndex = 9;
            this.ScaledFeedbackTextBox.TextAlign = System.Windows.Forms.HorizontalAlignment.Right;
            // 
            // MainWindow
            // 
            this.AutoScaleDimensions = new System.Drawing.SizeF(6F, 13F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.ClientSize = new System.Drawing.Size(496, 286);
            this.Controls.Add(this.scaledFeedbackLabel);
            this.Controls.Add(this.ScaledFeedbackTextBox);
            this.Controls.Add(this.logLabel);
            this.Controls.Add(this.targetLabel);
            this.Controls.Add(this.TargetTextBox);
            this.Controls.Add(this.LogTextBox);
            this.Controls.Add(this.serialNumberLabel);
            this.Controls.Add(this.SerialNumberTextBox);
            this.Controls.Add(this.ActivateCheckBox);
            this.Controls.Add(this.StopConnectingButton);
            this.Controls.Add(this.ConnectButton);
            this.Name = "MainWindow";
            this.Text = "Jrk Example GUI";
            this.Shown += new System.EventHandler(this.MainWindow_Shown);
            this.ResumeLayout(false);
            this.PerformLayout();

        }

        #endregion

        private System.Windows.Forms.Button ConnectButton;
        private System.Windows.Forms.Button StopConnectingButton;
        private System.Windows.Forms.CheckBox ActivateCheckBox;
        private System.Windows.Forms.TextBox SerialNumberTextBox;
        private System.Windows.Forms.Label serialNumberLabel;
        private System.Windows.Forms.TextBox LogTextBox;
        private System.Windows.Forms.Label targetLabel;
        private System.Windows.Forms.TextBox TargetTextBox;
        private System.Windows.Forms.Label logLabel;
        private System.Windows.Forms.Timer UpdateTimer;
        private System.Windows.Forms.Label scaledFeedbackLabel;
        private System.Windows.Forms.TextBox ScaledFeedbackTextBox;
    }
}

