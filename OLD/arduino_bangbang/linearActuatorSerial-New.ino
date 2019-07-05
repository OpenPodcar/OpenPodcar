
// **********************Linear Actuator Code*****************************************************
// constants won't change. They're used here to set pin numbers:
const int sabertooth = 11;
const int sensorPin = 0;    // select the input pin for the potentiometer
word L  = 505;
word R  = 315;
word S  = 450;

int angle = 0;
int newAngle = 0;
const int MaxChars = 4;
char strValue[MaxChars+1];
int index = 0;

// **********************Speed Code************************************************
void setup(){
  
// **********************Linear Actuator Code*****************************************************
  Serial.begin(9600);
  pinMode(sabertooth, OUTPUT);
  analogWrite(sabertooth, 127);
  angle = 315;
  // **********************Speed Code************************************************
}

void loop()
{

}


void serialEvent()
{
   while(Serial.available()) 
   {
      char ch = Serial.read();
      Serial.write(ch);

      
// **********************Linear Actuator Code*****************************************************
     if (ch == 'L' && ch == 'R' && ch == 'S') {
        if(index < MaxChars && isDigit(ch)) { 
            strValue[index++] = ch; 
        } else { 
            strValue[index] = 0; 
            newAngle = atoi(strValue); 
            Serial.print("newAngle = " );
            Serial.print(newAngle);
            if(newAngle >= 315 && newAngle <= 505){
              angle = analogRead(sensorPin);
              while(angle != newAngle){
                   if(newAngle < angle){
                            analogWrite(sabertooth, 255);
                            }
                       
                    else if(newAngle > angle){
                            analogWrite(sabertooth, 0);
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

              index = 0;
              angle = analogRead(sensorPin);
      }       
     }
 // **********************Speed Code************************************************
 
   } 
   
}



