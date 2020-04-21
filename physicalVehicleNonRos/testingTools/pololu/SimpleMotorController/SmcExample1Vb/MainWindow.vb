' SmcExample1Vb:
'   Simple example GUI for the Pololu Simple Motor Controller,
'   written in Visual Basic .NET.
' 
'   Features:
'       Native USB connection using Smc class
'       Forward button
'       Reverse button
'       Stop button
'
' NOTE: The Input Mode of your Simple Motor Controller must be set to Serial/USB
' for this program to work properly.  You must also connect USB, motor power,
' and your motor.  If this program does not work, use the Pololu Simple Motor
' Control Center to check what errors are occurring.

Imports Pololu.UsbWrapper
Imports Pololu.SimpleMotorController
Imports System
Imports System.Text
Imports System.ComponentModel

Public Class MainWindow
    ''' <summary>
    ''' This function runs when the user clicks the Forward button.
    ''' </summary>
    Sub forwardButton_Click(ByVal sender As Object, ByVal e As EventArgs) Handles forwardButton.Click
        Try
            Using device As Smc = connectToDevice() ' Find a device and temporarily connect.
                device.resume()                     ' Clear as many errors as possible.
                device.setSpeed(3200)               ' Set the speed to full forward (+100%).
            End Using
        Catch exception As Exception     ' Handle exceptions by displaying them to the user.
            displayException(exception)
        End Try
    End Sub

    ''' <summary>
    ''' This function runs when the user clicks the Reverse button.
    ''' </summary>
    Sub reverseButton_Click(ByVal sender As Object, ByVal e As EventArgs) Handles reverseButton.Click
        Try
            Using device As Smc = connectToDevice() ' Find a device and temporarily connect.
                device.resume()                     ' Clear as many errors as possible.
                device.setSpeed(-3200)              ' Set the speed to full reverse (-100%).
            End Using
        Catch exception As Exception     ' Handle exceptions by displaying them to the user.
            displayException(exception)
        End Try
    End Sub

    ''' <summary>
    ''' This function runs when the user clicks the Stop button.
    ''' </summary>
    Sub stopButton_Click(ByVal sender As Object, ByVal e As EventArgs) Handles stopButton.Click
        Try
            Using device As Smc = connectToDevice() ' Find a device and temporarily connect.
                device.stop()  ' Activate the USB kill switch

                ' Alternatively you can set the speed to 0 to stop the motor,
                ' but that will only stop the motor if the input mode is Serial/USB:
                '     device.setSpeed(0)
            End Using
        Catch exception As Exception     ' Handle exceptions by displaying them to the user.
            displayException(exception)
        End Try
    End Sub

    ''' <summary>
    ''' Connects to a Simple Motor Controller using native USB and returns the
    ''' Smc object representing that connection.  When you are done with the 
    ''' connection, you should close it using the Dispose() method so that other
    ''' processes or functions can connect to the device later.  The "Using"
    ''' statement can do this automatically for you.
    ''' </summary>
    Function connectToDevice() As Smc
        ' Get a list of all connected devices of this type.
        Dim connectedDevices As List(Of DeviceListItem) = Smc.getConnectedDevices()

        For Each dli As DeviceListItem In connectedDevices
            ' If you have multiple devices connected and want to select a particular
            ' device by serial number, you could simply add some code like this:
            '    If dli.serialNumber <> "39FF-6806-3054-3036-1128-0743" Then
            '        Continue For
            '    End If

            Dim device As Smc = New Smc(dli)  ' Connect to the device.
            Return device                     ' Return the device.
        Next

        Throw New Exception("Could not find device.  Make sure it is plugged in to USB " & _
            "and check your Device Manager.")
        End
    End Function

    ''' <summary>
    ''' Displays an exception (error) to the user by popping up a message box.
    ''' </summary>
    Sub displayException(ByVal exception As Exception)
        Dim stringBuilder As StringBuilder = New StringBuilder()
        Do
            stringBuilder.Append(exception.Message & "  ")
            If TypeOf exception Is Win32Exception Then
                Dim win32Exception As Win32Exception = DirectCast(exception, Win32Exception)
                stringBuilder.Append("Error code 0x" + win32Exception.NativeErrorCode.ToString("x") + ".  ")
            End If
            exception = exception.InnerException
        Loop Until (exception Is Nothing)
        MessageBox.Show(stringBuilder.ToString(), Text, MessageBoxButtons.OK, MessageBoxIcon.Error)
    End Sub

End Class
