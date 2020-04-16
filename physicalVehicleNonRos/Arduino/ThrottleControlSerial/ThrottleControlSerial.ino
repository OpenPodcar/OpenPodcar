/*****************FA*********************************************************/
/*!
    @file     MobilityScooterThrottle.pde
    @author   Chris Waltham - 24-07-2018

    This Code allows a user to control the throttle circuit of a mobility scooter.

    The DAC use the following address.
    MCP4725A0 address is 0x60 HEX
    
*/
/**************************************************************************/
#include <math.h>
#include <Wire.h>                     // include the Wire Library - needed to communicate with the DAC
#include <Adafruit_MCP4725.h>         // inlcude the DAC library - contains the comms protocol needed to communicate with the DAC

Adafruit_MCP4725 Throttle;            // instantiate a DAC instance called Throttle.

const String FA = "FA";               // ForwardAft Command, format FA:128, colon then byte value for axis 0 to 255, must be terminated with \r\n (carriage return, newline)
const String ST = "ST";               // STOP       Command, format ST, no parameter,  must be terminated with \r\n (carriage return,newline)
const String BV = "BV";               // Battery Voltage       Command, format BV, no parameter,  must be terminated with \r\n (carriage return,newline)

const float RATIO = 0.0537;  // 0.052;              // potential divider ratio found as ratio 0.052 = 55/1023 - 0.0015 

const int analogPin = A0;
int sensorValue = 0;
float batteryVoltage = 0.0;
 
float DACSupplyVoltage        = 5.00; //Included for information only. Could be used to calculate the desired DAC voltage for any DAC Value.

word DACCentre                = 1679;  // new DAC value found in July 2019 -- Charles & Fanta
//word DACCentre                = 1887; // The DAC value which allows the Scooter to switxh on without error or complaint.  
word DAC_Upper_LIMIT          = 2519; // DO NOT CHANGE - COULD DAMAGE SCOOTER CIRCUIT
word DAC_Lower_LIMIT          = 1000; // DO NOT CHANGE - COULD DAMAGE SCOOTER CIRCUIT 
word DACFullScale             = 4095; // 12bit DAC

word ThrottleDACValue         = DACCentre; //Value for ThrottleDACValue after reset/powerup

int ForeAftSerialCmdValue = (int)DACCentre; //Used to calcuate DAC values based on incoming PC parameter values 

bool commandReceived = false;               // Flag to tell non interupt time code to execute newly recieved commands
String fromPC        = "";                  // a String to hold incoming data, no data recieved yet so initialise empty

void setup(){
  
  // Initialise the system
  
  fromPC.reserve(100);                            // Reserve 100 bytes of memory for our serial buffer
  Throttle.begin(0x60);                           // Initialise I2C communications to the DAC board address HEX 60
  delay(100);                                     // Wait 100ms    
  Throttle.setVoltage(ThrottleDACValue, false);   // Set Initial voltage of DAC to the Centre Value.
  Serial.begin(9600);                           // Start the serial port connection to the PC 115200 baud (toggle DTR for remote reset)
  delay(100);                                     // Wait 100ms
  Serial.println("Car Throttle");                 // Announce the controller to the PC  
}

void loop()
{
  
 if(commandReceived == true)                      // This code is executed in non interupt time only when a new command has been recieved                                               // A new command has been recieved when a \n or \r character is recieved.
   processSerialCommand(fromPC);                  // Process the command
   
 
}
 
void UpdateDACs(void)
{            
    Throttle.setVoltage(ThrottleDACValue, false);    //Update the DAC setting.      
}

void serialEvent()                                   // As new characters are recieved by the USART hardware an interupt will fire which executes this code
{                                                    // This code is executed asynchronously in insterupt time.
  if (Serial.available())                            // Check there are characters available.
  {
     char c = Serial.read();                         // read the next available character from the USART buffer
     
     if ((c == '\n' || c == '\r') && fromPC.length() > 0) // check for a line terminator
     {
        fromPC.toUpperCase();                             // converto the command string to upper case
        commandReceived = true;         // set the command pending flag. This tells non interupt time code to execute the command proccesor code. (see: loop() above)
    
     }
     else if (c != ' ' && c != '\n' && c !='\r')          // Ignore spaces.
     {
        fromPC += c;                                      // if the character is not a space, a \n or \r concatenate the character to the fromPC string - build the command charater by character
     }                                                    // This funtion perrforms, in interupt time, a non blocking read of the serial buffer.
  }
}

void processSerialCommand(String command)                 // Simple command processing from the PC to the Scooter Controller - Each command is echoed back to PC along with calucated percentage and DAC value on one lime.
{
                                              // Clear the incoming buffer - current command is passed in 'command' command variable which free's the fromPC variable, so clear it in case more characters are placed in it during interupt time.
                                                     // This may happen now becuase Process Serial Command is now executed in non interupt time.
     
   if (command.startsWith(FA))                            // Is this a FA command? (abbreviation for ForeAft)
   {
       Serial.print(command);                             // Echo the current command to the PC
       Serial.print(" ");
       ForeAftSerialCmdValue = command.substring(3).toInt();       //Extract the value from the command. Structure is 'FA:byte' 0-255, 8 bit throttle value, 127 is the centre value
       float percentageThrottle;       
       percentageThrottle = (float) ForeAftSerialCmdValue / 255;   // Calculate a percentage throttle from the incoming command value
       Serial.print(percentageThrottle);                           // Echo the commanded percentage throttle setting to the PC
       Serial.print(" ");
       ThrottleDACValue =  DAC_Upper_LIMIT * percentageThrottle;   // Calculate based on the DAC Upper Limit const - the value of the command in terms of DAC values (Range the incoming value into 12bits)
       
       if(ThrottleDACValue >= DAC_Lower_LIMIT && ThrottleDACValue <= DAC_Upper_LIMIT) // check the resulting DAC value is higher than the lower limit and lower than the higher limit consts definitions.
         {
          UpdateDACs();                                                               // Call the update DAC funstion to apply the new voltage at the DAC
          Serial.println(ThrottleDACValue);                                           // Echo the new DAC value to the PC
         }
       else
        Serial.println("DAC LIMITS EXCEDED - THROTTLE SETTING UNCHANGED");            // if limits check fails - echo to PC the error
   }
   else if (command.startsWith(ST))                                                   // is the command ST - i.e. STOP
   {
       Serial.print(command);                                                         // Echo the command to STOP
       Serial.print(" ");
       Serial.println("STOP");              
       ThrottleDACValue = DACCentre;                                                  // STOP means set the DAC to the centre position so set the DAC Value to Centre
       UpdateDACs();                                                                  // Call update DAC with the new centre (STOP) value.
   }
   else if (command.startsWith(BV))
   {
      sensorValue = analogRead(analogPin);
      batteryVoltage = sensorValue*RATIO;
      
      Serial.print(command);
      Serial.print(" BV:");
      Serial.println(batteryVoltage);
  
   }
   
  fromPC = "";
  commandReceived = false;                       // Clear the command pending flag.
   
}




