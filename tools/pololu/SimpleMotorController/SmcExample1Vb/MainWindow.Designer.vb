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
        Me.reverseButton = New System.Windows.Forms.Button
        Me.forwardButton = New System.Windows.Forms.Button
        Me.stopButton = New System.Windows.Forms.Button
        Me.SuspendLayout()
        '
        'reverseButton
        '
        Me.reverseButton.Location = New System.Drawing.Point(12, 37)
        Me.reverseButton.Name = "reverseButton"
        Me.reverseButton.Size = New System.Drawing.Size(111, 23)
        Me.reverseButton.TabIndex = 5
        Me.reverseButton.Text = "&Reverse"
        Me.reverseButton.UseVisualStyleBackColor = True
        '
        'forwardButton
        '
        Me.forwardButton.Location = New System.Drawing.Point(273, 37)
        Me.forwardButton.Name = "forwardButton"
        Me.forwardButton.Size = New System.Drawing.Size(111, 23)
        Me.forwardButton.TabIndex = 4
        Me.forwardButton.Text = "&Forward"
        Me.forwardButton.UseVisualStyleBackColor = True
        '
        'stopButton
        '
        Me.stopButton.Location = New System.Drawing.Point(144, 37)
        Me.stopButton.Name = "stopButton"
        Me.stopButton.Size = New System.Drawing.Size(111, 23)
        Me.stopButton.TabIndex = 3
        Me.stopButton.Text = "&Stop"
        Me.stopButton.UseVisualStyleBackColor = True
        '
        'MainWindow
        '
        Me.AutoScaleDimensions = New System.Drawing.SizeF(6.0!, 13.0!)
        Me.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font
        Me.ClientSize = New System.Drawing.Size(399, 98)
        Me.Controls.Add(Me.reverseButton)
        Me.Controls.Add(Me.forwardButton)
        Me.Controls.Add(Me.stopButton)
        Me.Name = "MainWindow"
        Me.Text = "SmcExample1 in Visual Basic .NET"
        Me.ResumeLayout(False)

    End Sub
    Private WithEvents reverseButton As System.Windows.Forms.Button
    Private WithEvents forwardButton As System.Windows.Forms.Button
    Private WithEvents stopButton As System.Windows.Forms.Button

End Class
