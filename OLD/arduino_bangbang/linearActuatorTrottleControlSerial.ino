/**************************************************************************/
/*!
    @file     MobilityScooterThrottle.pde
    @authors   Chris Waltham  && Fanta Camara  && Yao Chen

    This Code allows a user to control the throttle circuit of a mobility scooter and the linear actuator with a Sabertooth controller.

    These DACS use the following address.
    MCP4725A0 the address is 0x62 (default)
    
*/
/**************************************************************************/

#include <Wire.h>
#include <Adafruit_MCP4725.h>

Adafruit_MCP4725 Throttle;


const String FA = "FA"; // ForwardAft Command, format FA:128, colon then byte value for axis 0 to 255, must be terminated with /r/n (carriage return, newline)
const String LR = "LR"; // LeftRight  Command, format LR:128, colon then byte value for axis 0 to 255, must be terminated with /r/n (carriage return, newline)
const String ST = "ST"; // STOP       Command, format ST, no parameter,  must be terminated with /r/n (carriage return, newline)


int ForeAftSerialCmdValue = 1460; 
int LeftRightSerialCmdValue = 127;

float DACSupplyVoltage        = 5.00;
word DACCentre                = 1500; 
word DACFullScale             = 4095;
word DACLIMIT                 = 2920; // - 3.6v DO NOT CHANGE - COULD DAMAGE SCOOTER CIRCUIT

word ThrottleDACValue          = 1460;

String fromPC                 = "";   // a String to hold incoming data


// **********************  Linear Actuator variablees *****************************************************
// constants won't change. They're used here to set pin numbers:
const int sabertooth = 11;
const int sensorPin = 0;    // select the input pin for the FA210

#define LEFT   505   // actuator for motion to the Left
#define RIGHT   315   // actuator for motion to the Right
#define STRAIGHT  450  // actuator for motion to Straight line

int angle = 0;     // variable that contains the linear actuator's current position
int newAngle = 0;  // variable that contains the linear actuator's goal position 



void setup(){
  
// ********************** Linear Actuator init *****************************************************
  //Serial.begin(9600);
  pinMode(sabertooth, OUTPUT);
  analogWrite(sabertooth, 127);   // initialize actuator to stop mode
  angle = 315;    // initialize actuator's position to full retraction 
  
  // ********************** Speed Code init ********************************************************
  fromPC.reserve(100);
  
  // For MCP4725A1 DACS the address is 0x61 (default) or 0x62 (ADDR pin tied to VCC)  
  Throttle.begin(0x60);
  delay(100);
    
  Throttle.setVoltage(ThrottleDACValue, false);
  
  Serial.begin(115200);
  delay(100);
  
  Serial.println("Car Throttle");
  Serial.println("Written by: C Waltham 25-05-2018  && Fanta Camara 01/06/2018");
  Serial.println("");
}

void loop()
{
  //ThrottleDACValue = 460;   // increase the voltage by about 0.05v per loop
  //delay(50);
  //UpdateDACs();
  //updateActuator();
}

 
void UpdateDACs(void)
{       
  // This limits the output voltage to 3.6v - DO NOT CHANGE
  // Any setting greater than 3.6v will be clamped to DACLIMIT which is currently set to 2215 which is approx 3.6v
  
  if(ThrottleDACValue <= DACLIMIT)
  {
   // Update the DAC
   // Send i2c command to DAC to change set voltage.
    
   Throttle.setVoltage(ThrottleDACValue, false);   
  }
  else
  {
   // if the requested set point is greater than 3.6 volts set the DAC to 3.6v
   // DO NOT CHANGE
   Throttle.setVoltage(DACLIMIT, false);
   //ThrottleDACValue = 0;                  // YOU CAN REMOVE THIS LINE - JUST FOR THIS DEMO - IT ZERO's THE DAC VALUE WHEN ITS INCRIMENTS EXCEEDS THE DACLIMIT
  }   
  //Serial.println(ThrottleDACValue);
}


void updateActuator()
{
  
     //Serial.print("newAngle = " );
     //Serial.print(newAngle);
     if(newAngle >= 315 && newAngle <= 505)
     {
        angle = analogRead(sensorPin);
        while(angle != newAngle)
        {
          if(newAngle < angle)
          {
            analogWrite(sabertooth, 0);
          }
          else if(newAngle > angle)
          {
            analogWrite(sabertooth, 255);
          }
                  
          angle = analogRead(sensorPin);
          Serial.print("Angle while loop = " );
          Serial.print(angle);
          Serial.print("\n" );
       }
       

       angle = analogRead(sensorPin);
       Serial.print("Angle = " );
       Serial.print(angle);
            
       analogWrite(sabertooth, 127);
       //Serial.println("Stop");
       delay(2000);
                                     
    }              
    
    angle = analogRead(sensorPin);
     
  
}

void serialEvent()
{
  if (Serial.available()) 
  {
     char c = Serial.read();   
     if ((c == '\n' || c == '\r') && fromPC.length() > 0)
     {
        fromPC.toUpperCase();
        processSerialCommand(fromPC);
        fromPC = "";
     }
     else if (c != ' ' && c != '\n' && c !='\r')
     {
        // Ignore spaces.
        fromPC += c; 
     }
  }
}



void processSerialCommand(String command)
{
   // Debug - see what the PC is sending // rem out to disable
   Serial.println(command);
   
   // Simple command processing from the PC to the Wheel Chair Controller
   if (command.startsWith(FA))
   {
       Serial.println(command);
       ForeAftSerialCmdValue = command.substring(3).toInt();
       
       float percentageThrottle;
       
       percentageThrottle = (float) ForeAftSerialCmdValue / 255;
       Serial.println(percentageThrottle);
        
       ThrottleDACValue =  DACLIMIT * percentageThrottle;
       UpdateDACs();
       Serial.println(ThrottleDACValue);
   }
   else if (command.startsWith(LR))
   {
       Serial.println(command);
       LeftRightSerialCmdValue = command.substring(3).toInt();
       float percentageActuator;
       percentageActuator = (float) LeftRightSerialCmdValue/255; 
       newAngle = (percentageActuator * 190) + 315;
       updateActuator();
       Serial.println(newAngle); 
   }
   else if (command.startsWith(ST))
   {
       Serial.println(command);
       Serial.println("STOP");              
       ForeAftSerialCmdValue   = 1460; 
       LeftRightSerialCmdValue = 127;
       UpdateDACs();
       updateActuator();
   }
}
