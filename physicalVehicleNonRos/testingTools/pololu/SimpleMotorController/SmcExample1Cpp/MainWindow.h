/*  SmcExample1Cpp:
 *    Simple example GUI for the Pololu Simple Motor Controller,
 *    written in Visual C++.
 *
 *    Features:
 *       Native USB connection using Smc class
 *       Forward button
 *       Reverse button
 *       Stop button
 * 
 *  NOTE: The Input Mode of your Simple Motor Controller must be set to Serial/USB
 *  for this program to work properly.  You must also connect USB, motor power,
 *  and your motor.  If this program does not work, use the Pololu Simple Motor
 *  Control Center to check what errors are occurring.
 */

#pragma once

namespace Pololu {
namespace SimpleMotorController {
namespace SmcExample1Cpp {

	using namespace Pololu::UsbWrapper;
	using namespace Pololu::SimpleMotorController;

	using namespace System;
	using namespace System::ComponentModel;
	using namespace System::Collections::Generic;
	using namespace System::Windows::Forms;
	using namespace System::Data;
	using namespace System::Drawing;
	using namespace System::Text;

	/// <summary>
	/// MainWindow: This is the main window that appears on the screen when this
	/// applicaiton runs.
	///
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
	private: System::Windows::Forms::Button^  reverseButton;
	private: System::Windows::Forms::Button^  forwardButton;
	private: System::Windows::Forms::Button^  stopButton;

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
			this->reverseButton = (gcnew System::Windows::Forms::Button());
			this->forwardButton = (gcnew System::Windows::Forms::Button());
			this->stopButton = (gcnew System::Windows::Forms::Button());
			this->SuspendLayout();
			// 
			// reverseButton
			// 
			this->reverseButton->Location = System::Drawing::Point(12, 37);
			this->reverseButton->Name = L"reverseButton";
			this->reverseButton->Size = System::Drawing::Size(111, 23);
			this->reverseButton->TabIndex = 5;
			this->reverseButton->Text = L"&Reverse";
			this->reverseButton->UseVisualStyleBackColor = true;
			this->reverseButton->Click += gcnew System::EventHandler(this, &MainWindow::reverseButton_Click);
			// 
			// forwardButton
			// 
			this->forwardButton->Location = System::Drawing::Point(273, 37);
			this->forwardButton->Name = L"forwardButton";
			this->forwardButton->Size = System::Drawing::Size(111, 23);
			this->forwardButton->TabIndex = 4;
			this->forwardButton->Text = L"&Forward";
			this->forwardButton->UseVisualStyleBackColor = true;
			this->forwardButton->Click += gcnew System::EventHandler(this, &MainWindow::forwardButton_Click);
			// 
			// stopButton
			// 
			this->stopButton->Location = System::Drawing::Point(144, 37);
			this->stopButton->Name = L"stopButton";
			this->stopButton->Size = System::Drawing::Size(111, 23);
			this->stopButton->TabIndex = 3;
			this->stopButton->Text = L"&Stop";
			this->stopButton->UseVisualStyleBackColor = true;
			this->stopButton->Click += gcnew System::EventHandler(this, &MainWindow::stopButton_Click);
			// 
			// MainWindow
			// 
			this->AutoScaleDimensions = System::Drawing::SizeF(6, 13);
			this->AutoScaleMode = System::Windows::Forms::AutoScaleMode::Font;
			this->ClientSize = System::Drawing::Size(399, 98);
			this->Controls->Add(this->reverseButton);
			this->Controls->Add(this->forwardButton);
			this->Controls->Add(this->stopButton);
			this->Name = L"MainWindow";
			this->Text = L"SmcExample1 in C++";
			this->ResumeLayout(false);

		}
#pragma endregion

	private:
        /// <summary>
        /// This function runs when the user clicks the Forward button.
        /// </summary>
	    Void forwardButton_Click(Object^ sender, EventArgs^ e)
		{
			Smc^ device;
            try
            {
                device = connectToDevice();  // Find a device and connect.
                device->resume();            // Clear as many errors as possible.
                device->setSpeed(3200);      // Set the speed to full forward (+100%).
            }
            catch (Exception^ exception)  // Handle exceptions by displaying them to the user.
            {
                displayException(exception);
            }
			finally
			{
				delete device;  // Close the connection so other processes can use the device.
			}
        }

        /// <summary>
        /// This function runs when the user clicks the Reverse button.
        /// </summary>
		Void reverseButton_Click(Object^ sender, EventArgs^ e)
		{
			Smc^ device;
            try
            {
                device = connectToDevice();  // Find a device and connect.
                device->resume();            // Clear as many errors as possible.
                device->setSpeed(-3200);     // Set the speed to full reverse (-100%).
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
        /// This function runs when the user clicks the Stop button.
        /// </summary>
        Void stopButton_Click(Object^ sender, EventArgs^ e)
		{
			Smc^ device;
            try
            {
                device = connectToDevice();  // Find a device and connect.
                device->stop();  // Activate the USB kill switch.

                // Alternatively you can set the speed to 0 to stop the motor,
                // but that will only stop the motor if the input mode is Serial/USB:
                //    device->setSpeed(0);
            }
            catch (Exception^ exception)  // Handle exceptions by displaying them to the user.
            {
                displayException(exception);
            }
			finally
			{
				delete device;  // Close the connection so other processes can use the device.
			}
		 }


		/// <summary>
        /// Connects to a Simple Motor Controller using native USB and returns the
        /// Smc object representing that connection.  When you are done with the 
        /// connection, you should close it using "delete" statement so that other
        /// processes or functions can connect to the device later.
        /// </summary>
        Smc^ connectToDevice()
        {
            // Get a list of all connected devices of this type.
			List<DeviceListItem^>^ connectedDevices = Smc::getConnectedDevices();

            for each(DeviceListItem^ dli in connectedDevices)
            {
                // If you have multiple devices connected and want to select a particular
                // device by serial number, you could simply add a line like this:
                //   if (dli->serialNumber != "39FF-6806-3054-3036-1128-0743"){ continue; }

                Smc^ device = gcnew Smc(dli); // Connect to the device.
                return device;                // Return the device.
            }
            throw gcnew Exception("Could not find device.  Make sure it is plugged in to USB " +
                "and check your Device Manager (Windows) or run lsusb (Linux).");
        }

        /// <summary>
        /// Displays an exception to the user by popping up a message box.
        /// </summary>
        Void displayException(Exception^ exception)
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

}; // end the MainWindow class declaration

}}} // end the 3 nested namespace declarations

