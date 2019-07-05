// constants won't change. They're used here to set pin numbers:
const int sabertooth = 11;
const int sensorPin = 0;    // select the input pin for the potentiometer


int sensorValue = 0;  // variable to store the value coming from the sensor

int goalPosition = 315;
int CurrentPosition = 0;


void setup() {

  //start serial connection
  Serial.begin(9600);

  pinMode(sabertooth, OUTPUT);
  analogWrite(sabertooth, 127);

}

void loop() {

  printf("Hello Yao !!!!!!!!!!!");

 /**** Control the motor to the goalPosition *****/


  // read the value from the sensor:
  CurrentPosition = analogRead(sensorPin);

  // print the results to the serial monitor:
  Serial.print("Current = " );
  Serial.print(CurrentPosition);
  Serial.print("\t Goal = ");
  Serial.println(goalPosition);


// if we are too far from the goalPosition and want to move forward, we use full speed
    if (goalPosition > CurrentPosition + 3) {
      analogWrite(sabertooth, 0);
      Serial.println("Extending full speed");
    }

/*
// if we are close enough to goalPosition and want to move forward, we use low speed
    else if (goalPosition > CurrentPosition + 3 && goalPosition <= CurrentPosition + 15) {
      analogWrite(sabertooth, 64);
      Serial.println("Extending low speed");
    }*/
/*
// if we are close enough to goalPosition and want to move backward, we use low speed
    else if (goalPosition < CurrentPosition - 3 && goalPosition >= CurrentPosition - 15) {
      analogWrite(sabertooth, 191);
      Serial.println("Retracting low speed");
    }*/

// if we are too far from goalPosition and want to go backward, we use full speed
    else if (goalPosition < CurrentPosition - 3) {
      analogWrite(sabertooth, 255);
      Serial.println("Retracting full speed");
    }

// if we are at the goalPosition, we stop the actuator for 2 sec. to stabililize the motion
    else {
      analogWrite(sabertooth, 127);
      Serial.println("Stop");
      delay(2000);
      
    }

/**** First Simple Test for forward and backward motion with stop in between *****/

/*
    analogWrite(sabertooth,0);
    Serial.println("Extending\n");
    delay(1000);
    // read the value from the sensor:
  CurrentPosition = analogRead(sensorPin);
  Serial.print("Current = " );
  Serial.print(CurrentPosition);
  analogWrite(sabertooth, 127);
  Serial.println("Stop\n");
  delay(2000);
  analogWrite(sabertooth,255);
  Serial.println("Rectract\n");
  delay(1000);
    // read the value from the sensor:
  CurrentPosition = analogRead(sensorPin);
  Serial.print("Current = " );
  Serial.print(CurrentPosition);
  analogWrite(sabertooth, 127);
  Serial.println("\nStop");
  delay(2000);
  */
}
