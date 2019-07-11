/*  MaestroEasyExampleCpp:
 *    Simple example GUI for the Maestro USB Servo Controller, written in
 *    Visual C++.
 *    
 *    Features:
 *       Temporary native USB connection using Usc class
 *       Button for disabling channel 0.
 *       Button for setting target of channel 0 to 1000 us.
 *       Button for setting target of channel 0 to 2000 us.
 * 
 *  NOTE: Channel 0 should be configured as a servo channel for this program
 *  to work.  You must also connect USB and servo power, and connect a servo
 *  to channel 0.  If this program does not work, use the Maestro Control
 *  Center to check what errors are occurring.
 */

#pragma once

namespace Pololu {
namespace Usc {
namespace MaestroEasyExampleCpp {

	using namespace Pololu::UsbWrapper;
	using namespace Pololu::Usc;

	using namespace System;
	using namespace System::ComponentModel;
	using namespace System::Collections;
	using namespace System::Collections::Generic;
	using namespace System::Windows::Forms;
	using namespace System::Data;
	using namespace System::Drawing;
	using namespace System::Text;

	/// <summary>
	/// WARNING: If you change the name of this class, you will need to change the
	///          'Resource File Name' property for the managed resource compiler tool
	///          associated with all .resx files this class depends on.  Otherwise,
	///          the designers will not be able to interact properly with localized
	///          resources associated with this form.
	/// </summary>
	public ref class MainWindow : public System::Windows::Forms::Form
	{
	public:
		MainWindow(void)
		{
			InitializeComponent();
			//
			//TODO: Add the constructor code here
			//
		}

	protected:
		/// <summary>
		/// Clean up any resources being used.
		/// </summary>
		~MainWindow()
		{
			if (components)
			{
				delete components;
			}
		}
	private: System::Windows::Forms::Button^  Button2000;
	protected: 
	private: System::Windows::Forms::Button^  Button1000;
	private: System::Windows::Forms::Label^  ChannelLabel;
	private: System::Windows::Forms::Button^  ButtonDisable;

	private:
		/// <summary>
		/// Required designer variable.
		/// </summary>
		System::ComponentModel::Container ^components;

#pragma region Windows Form Designer generated code
		/// <summary>
		/// Required method for Designer support - do not modify
		/// the contents of this method with the code editor.
		/// </summary>
		void InitializeComponent(void)
		{
			this->Button2000 = (gcnew System::Windows::Forms::Button());
			this->Button1000 = (gcnew System::Windows::Forms::Button());
			this->ChannelLabel = (gcnew System::Windows::Forms::Label());
			this->ButtonDisable = (gcnew System::Windows::Forms::Button());
			this->SuspendLayout();
			// 
			// Button2000
			// 
			this->Button2000->Location = System::Drawing::Point(302, 25);
			this->Button2000->Name = L"Button2000";
			this->Button2000->Size = System::Drawing::Size(118, 23);
			this->Button2000->TabIndex = 7;
			this->Button2000->Text = L"Target=&2000us";
			this->Button2000->UseVisualStyleBackColor = true;
			this->Button2000->Click += gcnew System::EventHandler(this, &MainWindow::Button2000_Click);
			// 
			// Button1000
			// 
			this->Button1000->Location = System::Drawing::Point(178, 25);
			this->Button1000->Name = L"Button1000";
			this->Button1000->Size = System::Drawing::Size(118, 23);
			this->Button1000->TabIndex = 6;
			this->Button1000->Text = L"Target=&1000us";
			this->Button1000->UseVisualStyleBackColor = true;
			this->Button1000->Click += gcnew System::EventHandler(this, &MainWindow::Button1000_Click);
			// 
			// ChannelLabel
			// 
			this->ChannelLabel->AutoSize = true;
			this->ChannelLabel->Location = System::Drawing::Point(12, 30);
			this->ChannelLabel->Name = L"ChannelLabel";
			this->ChannelLabel->Size = System::Drawing::Size(58, 13);
			this->ChannelLabel->TabIndex = 5;
			this->ChannelLabel->Text = L"Channel 0:";
			// 
			// ButtonDisable
			// 
			this->ButtonDisable->Location = System::Drawing::Point(92, 25);
			this->ButtonDisable->Name = L"ButtonDisable";
			this->ButtonDisable->Size = System::Drawing::Size(80, 23);
			this->ButtonDisable->TabIndex = 4;
			this->ButtonDisable->Text = L"&Disable";
			this->ButtonDisable->UseVisualStyleBackColor = true;
			this->ButtonDisable->Click += gcnew System::EventHandler(this, &MainWindow::ButtonDisable_Click);
			// 
			// MainWindow
			// 
			this->AutoScaleDimensions = System::Drawing::SizeF(6, 13);
			this->AutoScaleMode = System::Windows::Forms::AutoScaleMode::Font;
			this->ClientSize = System::Drawing::Size(453, 75);
			this->Controls->Add(this->Button2000);
			this->Controls->Add(this->Button1000);
			this->Controls->Add(this->ChannelLabel);
			this->Controls->Add(this->ButtonDisable);
			this->FormBorderStyle = System::Windows::Forms::FormBorderStyle::FixedSingle;
			this->Name = L"MainWindow";
			this->Text = L"MaestroEasyExample in C++";
			this->ResumeLayout(false);
			this->PerformLayout();

		}
#pragma endregion

        /// <summary>
        /// This functions runs when the user clicks the Target=1000us button.
        /// </summary>
        Void Button1000_Click(Object^ sender, EventArgs^ e)
        {
            TrySetTarget(0, 1000 * 4);  // Set the target of channel 0 to 1000 microseconds.
        }

        /// <summary>
        /// This functions runs when the user clicks the Target=2000us button.
        /// </summary>
        Void Button2000_Click(Object^ sender, EventArgs^ e)
        {
            TrySetTarget(0, 2000 * 4);  // Set the target of channel 0 to 2000 microseconds.
        }

        /// <summary>
        /// This function runs when the user clicks the Disable button.
        /// </summary>
        Void ButtonDisable_Click(Object^ sender, EventArgs^ e)
        {
            // Set target of channel 0 to 0.  This tells the Maestro to stop
            // transmitting pulses on that channel.  Any servo connected to it
            // should stop trying to maintain its position.
            TrySetTarget(0, 0);
        }
        
        /// <summary>
        /// Attempts to set the target (width of pulses sent) of a channel.
        /// </summary>
        /// <param name="channel">Channel number from 0 to 23.</param>
        /// <param name="target">
        ///   Target, in units of quarter microseconds.  For typical servos,
        ///   6000 is neutral and the acceptable range is 4000-8000.
        /// </param>
        Void TrySetTarget(Byte channel, UInt16 target)
        {
			Usc^ device;
            try
            {
                device = connectToDevice();           // Find a device and connect.
                device->setTarget(channel, target);
            }
            catch (Exception^ exception)  // Handle exceptions by displaying them to the user.
            {
                displayException(exception);
            }
			finally  // Do this no matter what.
			{
				delete device;  // Close the connection so other processes can use the device.
			}
        }

        /// <summary>
        /// Connects to a Maestro using native USB and returns the Usc object
        /// representing that connection.  When you are done with the
        /// connection, you should delete it using the "delete" statement so
		/// that other processes or functions can connect to the device later.
        /// </summary>
        Usc^ connectToDevice()
        {
            // Get a list of all connected devices of this type.
			List<DeviceListItem^>^ connectedDevices = Usc::getConnectedDevices();

            for each (DeviceListItem^ dli in connectedDevices)
            {
                // If you have multiple devices connected and want to select a particular
                // device by serial number, you could simply add a line like this:
                //   if (dli.serialNumber != "00012345"){ continue; }

                Usc^ device = gcnew Usc(dli); // Connect to the device.
                return device;             // Return the device.
            }
            throw gcnew Exception("Could not find device.  Make sure it is plugged in to USB " +
                "and check your Device Manager (Windows) or run lsusb (Linux).");
        }

        /// <summary>
        /// Displays an exception to the user by popping up a message box.
        /// </summary>
        void displayException(Exception^ exception)
        {
            StringBuilder^ stringBuilder = gcnew StringBuilder();
            do
            {
                stringBuilder->Append(exception->Message + "  ");
                if (exception->GetType() == Win32Exception::typeid)
                {
                    stringBuilder->Append("Error code 0x" + ((Win32Exception^)exception)->NativeErrorCode.ToString("x") + ".  ");
                }
                exception = exception->InnerException;
            }
            while (exception != nullptr);
			MessageBox::Show(stringBuilder->ToString(), this->Text, MessageBoxButtons::OK, MessageBoxIcon::Error);
        }

	}; // end class

}}} // end namespaces

