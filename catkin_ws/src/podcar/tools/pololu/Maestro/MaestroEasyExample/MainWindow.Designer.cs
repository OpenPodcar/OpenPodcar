namespace Pololu.Usc.MaestroEasyExample
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
            this.ButtonDisable = new System.Windows.Forms.Button();
            this.ChannelLabel = new System.Windows.Forms.Label();
            this.Button1000 = new System.Windows.Forms.Button();
            this.Button2000 = new System.Windows.Forms.Button();
            this.SuspendLayout();
            // 
            // ButtonDisable
            // 
            this.ButtonDisable.Location = new System.Drawing.Point(92, 25);
            this.ButtonDisable.Name = "ButtonDisable";
            this.ButtonDisable.Size = new System.Drawing.Size(80, 23);
            this.ButtonDisable.TabIndex = 0;
            this.ButtonDisable.Text = "&Disable";
            this.ButtonDisable.UseVisualStyleBackColor = true;
            this.ButtonDisable.Click += new System.EventHandler(this.ButtonDisable_Click);
            // 
            // ChannelLabel
            // 
            this.ChannelLabel.AutoSize = true;
            this.ChannelLabel.Location = new System.Drawing.Point(12, 30);
            this.ChannelLabel.Name = "ChannelLabel";
            this.ChannelLabel.Size = new System.Drawing.Size(58, 13);
            this.ChannelLabel.TabIndex = 1;
            this.ChannelLabel.Text = "Channel 0:";
            // 
            // Button1000
            // 
            this.Button1000.Location = new System.Drawing.Point(178, 25);
            this.Button1000.Name = "Button1000";
            this.Button1000.Size = new System.Drawing.Size(118, 23);
            this.Button1000.TabIndex = 2;
            this.Button1000.Text = "Target=&1000us";
            this.Button1000.UseVisualStyleBackColor = true;
            this.Button1000.Click += new System.EventHandler(this.Button1000_Click);
            // 
            // Button2000
            // 
            this.Button2000.Location = new System.Drawing.Point(302, 25);
            this.Button2000.Name = "Button2000";
            this.Button2000.Size = new System.Drawing.Size(118, 23);
            this.Button2000.TabIndex = 3;
            this.Button2000.Text = "Target=&2000us";
            this.Button2000.UseVisualStyleBackColor = true;
            this.Button2000.Click += new System.EventHandler(this.Button2000_Click);
            // 
            // MainWindow
            // 
            this.AutoScaleDimensions = new System.Drawing.SizeF(6F, 13F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.ClientSize = new System.Drawing.Size(453, 75);
            this.Controls.Add(this.Button2000);
            this.Controls.Add(this.Button1000);
            this.Controls.Add(this.ChannelLabel);
            this.Controls.Add(this.ButtonDisable);
            this.FormBorderStyle = System.Windows.Forms.FormBorderStyle.FixedSingle;
            this.Name = "MainWindow";
            this.Text = "MaestroEasyExample in C#";
            this.ResumeLayout(false);
            this.PerformLayout();

        }

        #endregion

        private System.Windows.Forms.Button ButtonDisable;
        private System.Windows.Forms.Label ChannelLabel;
        private System.Windows.Forms.Button Button1000;
        private System.Windows.Forms.Button Button2000;
    }
}

