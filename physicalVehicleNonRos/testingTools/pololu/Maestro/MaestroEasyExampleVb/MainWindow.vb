'  MaestroEasyExampleVb:
'    Simple example GUI for the Maestro USB Servo Controller, written in
'    Visual Basic .NET.
'    
'    Features:
'       Temporary native USB connection using Usc class.
'       Button for disabling channel 0.
'       Button for setting target of channel 0 to 1000 us.
'       Button for setting target of channel 0 to 2000 us.
' 
'  NOTE: Channel 0 should be configured as a servo channel for this program
'  to work.  You must also connect USB and servo power, and connect a servo
'  to channel 0.  If this program does not work, use the Maestro Control
'  Center to check what errors are occurring.

Imports Pololu.UsbWrapper
Imports Pololu.Usc

Imports System
Imports System.Text
Imports System.ComponentModel

Public Class MainWindow
    ''' <summary>
    ''' This subroutine runs when the user clicks the Target=1000us button.
    ''' </summary>
    Sub Button1000_Click(ByVal sender As Object, ByVal e As EventArgs) Handles Button1000.Click
        TrySetTarget(0, 1000 * 4) ' Set the target of channel 0 to 1000 microseconds.
    End Sub

    ''' <summary>
    ''' This subroutine runs when the user clicks the Target=2000us button.
    ''' </summary>
    Sub Button2000_Click(ByVal sender As Object, ByVal e As EventArgs) Handles Button2000.Click
        TrySetTarget(0, 2000 * 4) ' Set the target of channel 0 to 2000 microseconds.
    End Sub

    ''' <summary>
    ''' This function runs when the user clicks the Disable button.
    ''' </summary>
    Sub ButtonDisable_Click(ByVal sender As Object, ByVal e As EventArgs) Handles ButtonDisable.Click
        ' Set target of channel 0 to 0.  This tells the Maestro to stop
        ' transmitting pulses on that channel.  Any servo connected to it
        ' should stop trying to maintain its position.
        TrySetTarget(0, 0)
    End Sub

    ''' <summary>
    ''' Attempts to set the target of 
    ''' </summary>
    ''' <param name="channel">Channel number from 0 to 23.</param>
    ''' <param name="target">
    '''   Target, in units of quarter microseconds.  For typical servos,
    '''   6000 is neutral and the acceptable range is 4000-8000.
    ''' </param>
    Sub TrySetTarget(ByVal channel As Byte, ByVal target As UInt16)
        Try
            Using device As Usc = connectToDevice() ' Find a device and temporarily connect.
                device.setTarget(channel, target)
                ' device.Dispose() is called automatically when the "Using" block ends,
                ' allowing other functions and processes to use the device.
            End Using
        Catch exception As Exception  ' Handle exceptions by displaying them to the user.
            displayException(exception)
        End Try
    End Sub

    ''' <summary>
    ''' Connects to a Maestro using native USB and returns the Usc object
    ''' representing that connection.  When you are done with the
    ''' connection, you should close it using the Dispose() method so that
    ''' other processes or functions can connect to the device later.  The
    ''' "Using" statement can do this automatically for you.
    ''' </summary>
    Function connectToDevice() As Usc
        ' Get a list of all connected devices of this type.
        Dim connectedDevices As List(Of DeviceListItem) = Usc.getConnectedDevices()

        For Each dli As DeviceListItem In connectedDevices
            ' If you have multiple devices connected and want to select a particular
            ' device by serial number, you could simply add some code like this:
            '    If dli.serialNumber <> "00012345" Then
            '        Continue For
            '    End If

            Dim device As Usc = New Usc(dli)  ' Connect to the device.
            Return device                     ' Return the device.
        Next

        Throw New Exception("Could not find device.  Make sure it is plugged in to " & _
            "USB and check your Device Manager.")
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
