namespace Pololu.Usc.MaestroAdvancedExample
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
            this.label1 = new System.Windows.Forms.Label();
            this.LogTextBox = new System.Windows.Forms.TextBox();
            this.label2 = new System.Windows.Forms.Label();
            this.PositionTextBox = new System.Windows.Forms.TextBox();
            this.label3 = new System.Windows.Forms.Label();
            this.UpdateTimer = new System.Windows.Forms.Timer(this.components);
            this.SuspendLayout();
            // 
            // ConnectButton
            // 
            this.ConnectButton.Location = new System.Drawing.Point(225, 12);
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
            this.StopConnectingButton.Location = new System.Drawing.Point(225, 41);
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
            this.ActivateCheckBox.Size = new System.Drawing.Size(149, 17);
            this.ActivateCheckBox.TabIndex = 2;
            this.ActivateCheckBox.Text = "Activate motion sequence";
            this.ActivateCheckBox.UseVisualStyleBackColor = true;
            // 
            // SerialNumberTextBox
            // 
            this.SerialNumberTextBox.Location = new System.Drawing.Point(103, 14);
            this.SerialNumberTextBox.Name = "SerialNumberTextBox";
            this.SerialNumberTextBox.Size = new System.Drawing.Size(116, 20);
            this.SerialNumberTextBox.TabIndex = 3;
            // 
            // label1
            // 
            this.label1.AutoSize = true;
            this.label1.Location = new System.Drawing.Point(12, 17);
            this.label1.Name = "label1";
            this.label1.Size = new System.Drawing.Size(85, 13);
            this.label1.TabIndex = 4;
            this.label1.Text = "Maestro serial #:";
            // 
            // LogTextBox
            // 
            this.LogTextBox.Anchor = ((System.Windows.Forms.AnchorStyles)((((System.Windows.Forms.AnchorStyles.Top | System.Windows.Forms.AnchorStyles.Bottom)
                        | System.Windows.Forms.AnchorStyles.Left)
                        | System.Windows.Forms.AnchorStyles.Right)));
            this.LogTextBox.Location = new System.Drawing.Point(15, 105);
            this.LogTextBox.Multiline = true;
            this.LogTextBox.Name = "LogTextBox";
            this.LogTextBox.ReadOnly = true;
            this.LogTextBox.ScrollBars = System.Windows.Forms.ScrollBars.Vertical;
            this.LogTextBox.Size = new System.Drawing.Size(405, 147);
            this.LogTextBox.TabIndex = 5;
            // 
            // label2
            // 
            this.label2.AutoSize = true;
            this.label2.Location = new System.Drawing.Point(14, 66);
            this.label2.Name = "label2";
            this.label2.Size = new System.Drawing.Size(83, 13);
            this.label2.TabIndex = 7;
            this.label2.Text = "Current position:";
            // 
            // PositionTextBox
            // 
            this.PositionTextBox.Location = new System.Drawing.Point(101, 63);
            this.PositionTextBox.Name = "PositionTextBox";
            this.PositionTextBox.ReadOnly = true;
            this.PositionTextBox.Size = new System.Drawing.Size(99, 20);
            this.PositionTextBox.TabIndex = 6;
            this.PositionTextBox.TextAlign = System.Windows.Forms.HorizontalAlignment.Right;
            // 
            // label3
            // 
            this.label3.AutoSize = true;
            this.label3.Location = new System.Drawing.Point(14, 89);
            this.label3.Name = "label3";
            this.label3.Size = new System.Drawing.Size(28, 13);
            this.label3.TabIndex = 8;
            this.label3.Text = "Log:";
            // 
            // UpdateTimer
            // 
            this.UpdateTimer.Tick += new System.EventHandler(this.UpdateTimer_Tick);
            // 
            // MainWindow
            // 
            this.AutoScaleDimensions = new System.Drawing.SizeF(6F, 13F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.ClientSize = new System.Drawing.Size(432, 264);
            this.Controls.Add(this.label3);
            this.Controls.Add(this.label2);
            this.Controls.Add(this.PositionTextBox);
            this.Controls.Add(this.LogTextBox);
            this.Controls.Add(this.label1);
            this.Controls.Add(this.SerialNumberTextBox);
            this.Controls.Add(this.ActivateCheckBox);
            this.Controls.Add(this.StopConnectingButton);
            this.Controls.Add(this.ConnectButton);
            this.Name = "MainWindow";
            this.Text = "Maestro Example GUI";
            this.Shown += new System.EventHandler(this.MainWindow_Shown);
            this.ResumeLayout(false);
            this.PerformLayout();

        }

        #endregion

        private System.Windows.Forms.Button ConnectButton;
        private System.Windows.Forms.Button StopConnectingButton;
        private System.Windows.Forms.CheckBox ActivateCheckBox;
        private System.Windows.Forms.TextBox SerialNumberTextBox;
        private System.Windows.Forms.Label label1;
        private System.Windows.Forms.TextBox LogTextBox;
        private System.Windows.Forms.Label label2;
        private System.Windows.Forms.TextBox PositionTextBox;
        private System.Windows.Forms.Label label3;
        private System.Windows.Forms.Timer UpdateTimer;
    }
}

