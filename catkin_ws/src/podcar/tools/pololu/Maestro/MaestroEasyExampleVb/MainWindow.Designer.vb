<Global.Microsoft.VisualBasic.CompilerServices.DesignerGenerated()> _
Partial Class MainWindow
    Inherits System.Windows.Forms.Form

    'Form overrides dispose to clean up the component list.
    <System.Diagnostics.DebuggerNonUserCode()> _
    Protected Overrides Sub Dispose(ByVal disposing As Boolean)
        Try
            If disposing AndAlso components IsNot Nothing Then
                components.Dispose()
            End If
        Finally
            MyBase.Dispose(disposing)
        End Try
    End Sub

    'Required by the Windows Form Designer
    Private components As System.ComponentModel.IContainer

    'NOTE: The following procedure is required by the Windows Form Designer
    'It can be modified using the Windows Form Designer.  
    'Do not modify it using the code editor.
    <System.Diagnostics.DebuggerStepThrough()> _
    Private Sub InitializeComponent()
        Me.Button2000 = New System.Windows.Forms.Button
        Me.Button1000 = New System.Windows.Forms.Button
        Me.ChannelLabel = New System.Windows.Forms.Label
        Me.ButtonDisable = New System.Windows.Forms.Button
        Me.SuspendLayout()
        '
        'Button2000
        '
        Me.Button2000.Location = New System.Drawing.Point(302, 25)
        Me.Button2000.Name = "Button2000"
        Me.Button2000.Size = New System.Drawing.Size(118, 23)
        Me.Button2000.TabIndex = 7
        Me.Button2000.Text = "Target=&2000us"
        Me.Button2000.UseVisualStyleBackColor = True
        '
        'Button1000
        '
        Me.Button1000.Location = New System.Drawing.Point(178, 25)
        Me.Button1000.Name = "Button1000"
        Me.Button1000.Size = New System.Drawing.Size(118, 23)
        Me.Button1000.TabIndex = 6
        Me.Button1000.Text = "Target=&1000us"
        Me.Button1000.UseVisualStyleBackColor = True
        '
        'ChannelLabel
        '
        Me.ChannelLabel.AutoSize = True
        Me.ChannelLabel.Location = New System.Drawing.Point(12, 30)
        Me.ChannelLabel.Name = "ChannelLabel"
        Me.ChannelLabel.Size = New System.Drawing.Size(58, 13)
        Me.ChannelLabel.TabIndex = 5
        Me.ChannelLabel.Text = "Channel 0:"
        '
        'ButtonDisable
        '
        Me.ButtonDisable.Location = New System.Drawing.Point(92, 25)
        Me.ButtonDisable.Name = "ButtonDisable"
        Me.ButtonDisable.Size = New System.Drawing.Size(80, 23)
        Me.ButtonDisable.TabIndex = 4
        Me.ButtonDisable.Text = "&Disable"
        Me.ButtonDisable.UseVisualStyleBackColor = True
        '
        'MainWindow
        '
        Me.AutoScaleDimensions = New System.Drawing.SizeF(6.0!, 13.0!)
        Me.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font
        Me.ClientSize = New System.Drawing.Size(453, 75)
        Me.Controls.Add(Me.Button2000)
        Me.Controls.Add(Me.Button1000)
        Me.Controls.Add(Me.ChannelLabel)
        Me.Controls.Add(Me.ButtonDisable)
        Me.FormBorderStyle = System.Windows.Forms.FormBorderStyle.FixedSingle
        Me.Name = "MainWindow"
        Me.Text = "MaestroEasyExample in Visual Basic"
        Me.ResumeLayout(False)
        Me.PerformLayout()

    End Sub
    Private WithEvents Button2000 As System.Windows.Forms.Button
    Private WithEvents Button1000 As System.Windows.Forms.Button
    Private WithEvents ChannelLabel As System.Windows.Forms.Label
    Private WithEvents ButtonDisable As System.Windows.Forms.Button

End Class
